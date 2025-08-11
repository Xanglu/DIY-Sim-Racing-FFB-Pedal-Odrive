using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Newtonsoft.Json.Linq;



/// <summary>
/// Finds valid start-of-frame (SOF) and end-of-frame (EOF) pairs for a specific message type.
/// </summary>
/// <param name="sofIndices">The list of SOF indices for this message type.</param>
/// <param name="eofIndices">The list of all found EOF indices.</param>
/// <param name="expectedLength">The expected size of the struct (e.g., sizeof(DAP_state_basic_st)).</param>
/// <param name="validPairs">The list where valid (SOF, EOF) index pairs will be added.</param>
/// <param name="sofReceivedWithoutEof">A flag that is set to true if the last SOF has no matching EOF.</param>
private void FindValidMessagePairs(
    List<int> sofIndices,
    List<int> eofIndices,
    int expectedLength,
    List<Tuple<int, int>> validPairs,
    ref bool sofReceivedWithoutEof)
{
    // Iterate through each potential start-of-frame for this message type
    for (int i = 0; i < sofIndices.Count; i++)
    {
        int indSof = sofIndices[i];
        bool pairFound = false;

        // Try to find a matching end-of-frame
        foreach (int indEof in eofIndices)
        {
            // The data length is the distance between markers, plus EOF bytes
            int dataLength = indEof - indSof + 2;

            if (dataLength < 0)
            {
                // EOF is before SOF, so it can't be a match for this SOF.
                // Continue to check the next EOF.
                continue;
            }

            if (dataLength > expectedLength)
            {
                // This EOF is too far away. Since indices are sorted,
                // no subsequent EOF will match either.
                pairFound = true; // Prevents flagging 'sofReceivedWithoutEof' incorrectly
                break;
            }

            if (dataLength == expectedLength)
            {
                // --- SUCCESS: Found a valid pair! ---
                validPairs.Add(new Tuple<int, int>(indSof, indSof + dataLength));
                pairFound = true;
                // A SOF can only have one matching EOF, so we can stop searching.
                break;
            }
        }

        // Check if this was the last SOF index and it had no matching EOF
        if (!pairFound && i == sofIndices.Count - 1)
        {
            sofReceivedWithoutEof = true;
        }
    }
}




namespace User.PluginSdkDemo
{
    public partial class DIYFFBPedalControlUI : System.Windows.Controls.UserControl
    {
        int[] appendedBufferOffset = { 0, 0, 0, 0 };
        static int bufferSize = 10000;
        static int destBufferSize = 1000;
        byte[][] buffer_appended = { new byte[bufferSize], new byte[bufferSize], new byte[bufferSize], new byte[bufferSize] };
        byte[][] buffer_appended_clone = { new byte[bufferSize], new byte[bufferSize], new byte[bufferSize], new byte[bufferSize] };
        unsafe public void timerCallback_serial(object sender, EventArgs e)
        {

            //action here 
            Simhub_action_update();




            int pedalSelected = Int32.Parse((sender as System.Windows.Forms.Timer).Tag.ToString());
            //int pedalSelected = (int)(sender as System.Windows.Forms.Timer).Tag;

            bool pedalStateHasAlreadyBeenUpdated_b = false;

            // once the pedal has identified, go ahead
            if (pedalSelected < 3)
            //if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {



                // Create a Stopwatch instance
                Stopwatch stopwatch = new Stopwatch();

                // Start the stopwatch
                stopwatch.Start();



                SerialPort sp = Plugin._serialPort[pedalSelected];



                // https://stackoverflow.com/questions/9732709/the-calling-thread-cannot-access-this-object-because-a-different-thread-owns-it


                //int length = sizeof(DAP_config_st);




                if (sp.IsOpen)
                {
                    if (Plugin.Settings.Serial_auto_clean)
                    {
                        /*
                        if (TextBox_serialMonitor.LineCount > 300)
                        {
                            TextBox_serialMonitor.Clear();
                        }
                        */
                        if (_serial_monitor_window != null && _serial_monitor_window.TextBox_SerialMonitor.LineCount > 300)
                        {
                            _serial_monitor_window.TextBox_SerialMonitor.Clear();
                        }
                    }

                    int receivedLength = 0;
                    try
                    {
                        receivedLength = sp.BytesToRead;
                    }
                    catch (Exception ex)
                    {
                        TextBox2.Text = ex.Message;
                        //ConnectToPedal.IsChecked = false;
                        return;
                    }



                    if (receivedLength > 0)
                    {

                        //TextBox_serialMonitor.Text += "Received:" + receivedLength + "\n";
                        //TextBox_serialMonitor.ScrollToEnd();


                        timeCntr[pedalSelected] += 1;


                        // determine byte sequence which is defined as message end --> crlf
                        byte[] byteToFind = System.Text.Encoding.GetEncoding(28591).GetBytes(STOPCHAR[0].ToCharArray());
                        int stop_char_length = byteToFind.Length;

                        // check if buffer is large enough otherwise discard in buffer and set offset to 0
                        //if ((bufferSize > currentBufferLength) && (appendedBufferOffset[pedalSelected] >= 0))
                        // Copy all bytes
                        Buffer.BlockCopy(buffer_appended[pedalSelected], 0, buffer_appended_clone[pedalSelected], 0, bufferSize);

                        int prevOffset = appendedBufferOffset[pedalSelected];

                        if (appendedBufferOffset[pedalSelected] > 0)
                        {
                            int tmp = 5;
                        }

                        bool inBufferDicarded = false;
                        int currentBufferLength = 0;
                        if (bufferSize > currentBufferLength) 
                        {
                            receivedLength = sp.Read(buffer_appended[pedalSelected], appendedBufferOffset[pedalSelected], receivedLength);

                            // calculate current buffer length
                            appendedBufferOffset[pedalSelected] += receivedLength;
                            currentBufferLength = appendedBufferOffset[pedalSelected];
                        }
                        else
                        {
                            inBufferDicarded = true;
                            sp.DiscardInBuffer();
                            appendedBufferOffset[pedalSelected] = 0;
                            return;
                        }


                        if ( !((buffer_appended[pedalSelected][0] == 170) && (buffer_appended[pedalSelected][1] == 85)) )
                        {
                            int tmp = 5;
                        }





                        // copy to local buffer
                        //byte[] localBuffer = new byte[currentBufferLength];

                        //Buffer.BlockCopy(buffer_appended[pedalSelected], 0, localBuffer, 0, currentBufferLength);


                        // find all occurences of crlf as they indicate message end
                        List<int> indices = FindAllOccurrences(buffer_appended[pedalSelected], byteToFind, currentBufferLength);


                        List<int> indices_sof = FindAllOccurrences(buffer_appended[pedalSelected], STARTOFFRAMCHAR, currentBufferLength);
                        List<int> indices_sof_extended_struct = FindAllOccurrences(buffer_appended[pedalSelected], STARTOFFRAME_EXTENDED_STRUCT, currentBufferLength);
                        List<int> indices_sof_basic_struct = FindAllOccurrences(buffer_appended[pedalSelected], STARTOFFRAME_BASIC_STRUCT, currentBufferLength);
                        List<int> indices_sof_config = FindAllOccurrences(buffer_appended[pedalSelected], STARTOFFRAME_CONFIG, currentBufferLength);
                        List<int> indices_eof = FindAllOccurrences(buffer_appended[pedalSelected], ENDOFFRAMCHAR, currentBufferLength);

                        var validPairsExtendedStruct = new List<Tuple<int, int>>();
                        var validPairsBasicStruct = new List<Tuple<int, int>>();
                        var validPairsConfig = new List<Tuple<int, int>>();

                        bool sofHasBeenReceivedEofNotYet = false;


                        // Search for the basic struct
                        FindValidMessagePairs(
                            indices_sof_basic_struct,
                            indices_eof,
                            sizeof(DAP_state_basic_st),
                            validPairsBasicStruct,
                            ref sofHasBeenReceivedEofNotYet);

                        // Search for the extended struct
                        FindValidMessagePairs(
                            indices_sof_extended_struct,
                            indices_eof,
                            sizeof(DAP_state_extended_st),
                            validPairsExtendedStruct,
                            ref sofHasBeenReceivedEofNotYet);
                        
                        // Search for the config struct
                        FindValidMessagePairs(
                            indices_sof_config,
                            indices_eof,
                            sizeof(DAP_config_st),
                            validPairsConfig,
                            ref sofHasBeenReceivedEofNotYet);

                        // check if at least SOF1 byte was received, but EOF was not for last packet
                        List<int> indices_sof1 = FindAllOccurrences(buffer_appended[pedalSelected], STARTOFFRAMCHAR_SOF_byte0, currentBufferLength);
                        if (currentBufferLength - indices_sof1.Last<int>() < 3 )
                        {
                            sofHasBeenReceivedEofNotYet = true;
                        }

                        // Todo: 
                        // Make "bufferByteAssignedToStruct" to hold states
                        // 0: not assigned
                        // 1: basic struct
                        // 2: extended struct
                        // 3: config struct
                        // 4: not assigned struct

                        // CRC check inside of "FindValidMessagePairs(...)"
                        
                        // provide "bufferByteAssignedToStruct" to "FindValidMessagePairs(...)" to label the data.

                        


                        



                        

                        // Destination array
                        byte[] destinationArray = new byte[destBufferSize];
                        bool[] bufferByteAssignedToStruct = new bool[bufferSize];
                        int lastTrueElementIndex = 0;

                        
                        
                        bool newMethod = true;

                        if (newMethod)
                        {
                            // extended struct
                            for (int pairId = 0; pairId < validPairsExtendedStruct.Count; pairId++)
                            {
                                int srcBufferOffset_0 = validPairsExtendedStruct[pairId].Item1;
                                int srcBufferOffset_1 = validPairsExtendedStruct[pairId].Item2;

                                // copy bytes to subarray
                                Buffer.BlockCopy(buffer_appended[pedalSelected], srcBufferOffset_0, destinationArray, 0, sizeof(DAP_state_extended_st));

                                int destBuffLength = srcBufferOffset_1 - srcBufferOffset_0;

                                // check for pedal extended state struct
                                if ((destBuffLength == sizeof(DAP_state_extended_st)))
                                {

                                    // parse byte array as config struct
                                    DAP_state_extended_st pedalState_ext_read_st = getStateExtFromBytes(destinationArray);

                                    // check whether receive struct is plausible
                                    DAP_state_extended_st* v_state = &pedalState_ext_read_st;
                                    byte* p_state = (byte*)v_state;

                                    // payload type check
                                    bool check_payload_state_b = false;
                                    if (pedalState_ext_read_st.payloadHeader_.payloadType == Constants.pedalStateExtendedPayload_type)
                                    {
                                        check_payload_state_b = true;
                                    }

                                    // CRC check
                                    bool check_crc_state_b = false;
                                    if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Extended)) == pedalState_ext_read_st.payloadFooter_.checkSum)
                                    {
                                        check_crc_state_b = true;
                                    }

                                    if ((check_payload_state_b) && check_crc_state_b)
                                    {

                                        bufferByteAssignedToStruct.AsSpan(srcBufferOffset_0, sizeof(DAP_state_extended_st)).Fill(true);
                                        lastTrueElementIndex = Math.Max(lastTrueElementIndex, srcBufferOffset_0 + sizeof(DAP_state_extended_st));

                                        if (indexOfSelectedPedal_u == pedalSelected)
                                        {

                                            if (Plugin._calculations.dumpPedalToResponseFile[indexOfSelectedPedal_u])
                                            //if (dumpPedalToResponseFile[indexOfSelectedPedal_u])
                                            {
                                                // Specify the path to the file
                                                string currentDirectory = Directory.GetCurrentDirectory();
                                                string filePath = currentDirectory + "\\PluginsData\\Common" + "\\DiyFfbPedalStateLog_" + indexOfSelectedPedal_u.ToString() + ".txt";


                                                // delete file 
                                                if (true == Plugin._calculations.dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u])
                                                {
                                                    Plugin._calculations.dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u] = false;
                                                    File.Delete(filePath);
                                                }


                                                // write header
                                                if (!File.Exists(filePath))
                                                {
                                                    using (StreamWriter writer = new StreamWriter(filePath, true))
                                                    {
                                                        // Write the content to the file
                                                        writer.Write("cycleCtr");
                                                        writer.Write(", time_InUs");
                                                        writer.Write(", forceRaw_InKg");
                                                        writer.Write(", forceFiltered_InKg");
                                                        writer.Write(", forceVelocity_InKgPerSec");
                                                        writer.Write(", servoPos_InSteps");
                                                        writer.Write(", servoPosEsp_InSteps");
                                                        writer.Write(", servoPosError_InSteps");
                                                        writer.Write(", servoCurrent_InPercent");
                                                        writer.Write(", servoVoltage_InV");
                                                        writer.Write(", angleSensorOutput");
                                                        writer.Write(", brakeResistorState_b");
                                                        writer.Write(", servoPosEstimated_InSteps");
                                                        //writer.Write(", servoPositionEstimated_stepperPos_i16");
                                                        writer.Write("\n");
                                                    }

                                                }


                                                // Use StreamWriter to write to the file
                                                using (StreamWriter writer = new StreamWriter(filePath, true))
                                                {
                                                    // Write the content to the file
                                                    writeCntr++;
                                                    writer.Write(writeCntr);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.timeInUs_u32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_raw_fl32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_filtered_fl32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.forceVel_est_fl32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPosition_i16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionTarget_i16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servo_position_error_i16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servo_current_percent_i16);
                                                    writer.Write(", ");
                                                    writer.Write(((float)pedalState_ext_read_st.payloadPedalExtendedState_.servo_voltage_0p1V_i16) / 10.0);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.angleSensorOutput_ui16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.brakeResistorState_b);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionEstimated_i16);
                                                    //writer.Write(", ");
                                                    //writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionEstimated_stepperPos_i16);
                                                    writer.Write("\n");
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // basic struct
                            for (int pairId = 0; pairId < validPairsBasicStruct.Count; pairId++)
                            {
                                int srcBufferOffset_0 = validPairsBasicStruct[pairId].Item1;
                                int srcBufferOffset_1 = validPairsBasicStruct[pairId].Item2;

                                int destBuffLength = srcBufferOffset_1 - srcBufferOffset_0;

                                // check for pedal extended state struct
                                if ((destBuffLength == sizeof(DAP_state_basic_st)))
                                {

                                    // copy bytes to subarray
                                    Buffer.BlockCopy(buffer_appended[pedalSelected], srcBufferOffset_0, destinationArray, 0, sizeof(DAP_state_basic_st));

                                    // parse byte array as config struct
                                    DAP_state_basic_st pedalState_read_st = getStateFromBytes(destinationArray);

                                    // check whether receive struct is plausible
                                    DAP_state_basic_st* v_state = &pedalState_read_st;
                                    byte* p_state = (byte*)v_state;

                                    // payload type check
                                    bool check_payload_state_b = false;
                                    if (pedalState_read_st.payloadHeader_.payloadType == Constants.pedalStateBasicPayload_type)
                                    {
                                        check_payload_state_b = true;
                                    }

                                    //Pedal version and Plugin DAP version check
                                    Pedal_version[pedalSelected] = pedalState_read_st.payloadHeader_.version;


                                    // CRC check
                                    bool check_crc_state_b = false;
                                    if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Basic)) == pedalState_read_st.payloadFooter_.checkSum)
                                    {
                                        check_crc_state_b = true;
                                    }

                                    if ((check_payload_state_b) && check_crc_state_b)
                                    {

                                        bufferByteAssignedToStruct.AsSpan(srcBufferOffset_0, sizeof(DAP_state_basic_st)).Fill(true);
                                        lastTrueElementIndex = Math.Max(lastTrueElementIndex, srcBufferOffset_0 + sizeof(DAP_state_basic_st));

                                        // write vJoy data
                                        Pedal_position_reading[pedalSelected] = pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16;
                                        //if (Plugin.Rudder_enable_flag == false)
                                        //{
                                        if (Plugin.Settings.vjoy_output_flag == 1)
                                        {
                                            switch (pedalSelected)
                                            {

                                                case 0:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RX);  // Center X axis
                                                    Plugin._calculations._joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);   // HID_USAGES Enums
                                                    break;
                                                case 1:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RY);  // Center X axis
                                                    Plugin._calculations._joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);   // HID_USAGES Enums
                                                    break;
                                                case 2:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RZ);  // Center X axis
                                                    Plugin._calculations._joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);   // HID_USAGES Enums
                                                    break;
                                                default:
                                                    break;
                                            }

                                        }

                                        //check servo status change
                                        if (Plugin._calculations.ServoStatus[pedalSelected] == (byte)enumServoStatus.On && pedalState_read_st.payloadPedalBasicState_.servoStatus == (byte)enumServoStatus.Idle)
                                        {
                                            string tmp = "Pedal:" + pedalSelected + " Servo idle reach timeout, power cutoff, please restart pedal to wake it up";
                                            ToastNotification("Wireless Connection", tmp);
                                        }
                                        // Force stop action
                                        if (Plugin._calculations.ServoStatus[pedalSelected] == (byte)enumServoStatus.On && pedalState_read_st.payloadPedalBasicState_.servoStatus == (byte)enumServoStatus.ForceStop)
                                        {
                                            string tmp = "Pedal:" + pedalSelected + " force Stopped";
                                            ToastNotification("Wireless Connection", tmp);
                                        }

                                        //fill servo status

                                        Plugin._calculations.ServoStatus[pedalSelected] = pedalState_read_st.payloadPedalBasicState_.servoStatus;



                                        // GUI update
                                        if ((pedalStateHasAlreadyBeenUpdated_b == false) && (indexOfSelectedPedal_u == pedalSelected))
                                        {
                                            //TextBox_debugOutput.Text = "Pedal pos: " + pedalState_read_st.payloadPedalState_.pedalPosition_u16;
                                            //TextBox_debugOutput.Text += "Pedal force: " + pedalState_read_st.payloadPedalState_.pedalForce_u16;
                                            //TextBox_debugOutput.Text += ",  Servo pos targe: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;
                                            //TextBox_debugOutput.Text += ",  Servo pos: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;

                                            PedalForceTravel_Tab.updatePedalState(pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16, pedalState_read_st.payloadPedalBasicState_.pedalForce_u16);

                                            pedalStateHasAlreadyBeenUpdated_b = true;

                                            double control_rect_value_max = 65535;


                                            if (Plugin.Settings.advanced_b)
                                            {
                                                int round_x = (int)(100 * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 / control_rect_value_max) - 1;
                                                int x_showed = round_x + 1;

                                                current_pedal_travel_state = x_showed;
                                                Plugin.pedal_state_in_ratio = (byte)current_pedal_travel_state;

                                            }
                                            else
                                            {
                                                int round_x = (int)(100 * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 / control_rect_value_max) - 1;
                                                int x_showed = round_x + 1;
                                                round_x = Math.Max(0, Math.Min(round_x, 99));
                                                current_pedal_travel_state = x_showed;
                                                Plugin.pedal_state_in_ratio = (byte)current_pedal_travel_state;
                                            }
                                            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.travelAsJoystickOutput_u8 == 1)
                                            {
                                                PedalJoystick_Tab.JoystickStateUpdate(pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16);
                                            }
                                            else
                                            {
                                                PedalJoystick_Tab.JoystickStateUpdate(pedalState_read_st.payloadPedalBasicState_.pedalForce_u16);
                                            }
                                            for (int i = 0; i < 3; i++)
                                            {
                                                //PedalFirmwareVersion[pedalSelected, i] = pedalState_read_st.payloadPedalBasicState_.pedalFirmwareVersion_u8[i];
                                                Plugin._calculations.PedalFirmwareVersion[pedalSelected, i] = pedalState_read_st.payloadPedalBasicState_.pedalFirmwareVersion_u8[i];
                                            }
                                        }
                                    }
                                }
                            }

                            // config struct
                            for (int pairId = 0; pairId < validPairsConfig.Count; pairId++)
                            {
                                int srcBufferOffset_0 = validPairsConfig[pairId].Item1;
                                int srcBufferOffset_1 = validPairsConfig[pairId].Item2;

                                // copy bytes to subarray
                                Buffer.BlockCopy(buffer_appended[pedalSelected], srcBufferOffset_0, destinationArray, 0, sizeof(DAP_config_st));

                                int destBuffLength = srcBufferOffset_1 - srcBufferOffset_0;

                                // decode into config struct
                                if ((waiting_for_pedal_config[pedalSelected]) && (destBuffLength == sizeof(DAP_config_st)))
                                {

                                    // parse byte array as config struct
                                    DAP_config_st pedalConfig_read_st = getConfigFromBytes(destinationArray);

                                    // check whether receive struct is plausible
                                    DAP_config_st* v_config = &pedalConfig_read_st;
                                    byte* p_config = (byte*)v_config;

                                    // payload type check
                                    bool check_payload_config_b = false;
                                    if (pedalConfig_read_st.payloadHeader_.payloadType == Constants.pedalConfigPayload_type)
                                    {
                                        check_payload_config_b = true;
                                    }

                                    // CRC check
                                    bool check_crc_config_b = false;
                                    if (Plugin.checksumCalc(p_config, sizeof(payloadHeader) + sizeof(payloadPedalConfig)) == pedalConfig_read_st.payloadFooter_.checkSum)
                                    {
                                        check_crc_config_b = true;
                                    }

                                    if ((check_payload_config_b) && check_crc_config_b)
                                    {

                                        bufferByteAssignedToStruct.AsSpan(srcBufferOffset_0, sizeof(DAP_config_st)).Fill(true);
                                        lastTrueElementIndex = Math.Max(lastTrueElementIndex, srcBufferOffset_0 + sizeof(DAP_config_st));

                                        waiting_for_pedal_config[pedalSelected] = false;
                                        dap_config_st[pedalSelected] = pedalConfig_read_st;
                                        Plugin.PedalConfigRead_b[pedalSelected] = true;
                                        updateTheGuiFromConfig();

                                        continue;
                                    }
                                    else
                                    {
                                        TextBox2.Text = "Payload config test 1: " + check_payload_config_b;
                                        TextBox2.Text += "Payload config test 2: " + check_crc_config_b;
                                    }

                                }

                            }


                            // print all non identified structs to serial monitor
                            // If non known array datatype was received, assume a text message was received and print it
                            // only print debug messages when debug mode is active as it degrades performance
                            if (/*Debug_check.IsChecked == true|| */_serial_monitor_window != null)
                            {
                                // Create a list to hold filtered elements
                                List<byte> filteredList = new List<byte>();

                                // Todo: dont print any bytes, where SOF has started, but no EOF was received yet
                                if (!sofHasBeenReceivedEofNotYet)
                                {
                                    for (int i = 0; i < currentBufferLength; i++)
                                    {
                                        if (!bufferByteAssignedToStruct[i]) // copy only if not true
                                        {
                                            filteredList.Add(buffer_appended[pedalSelected][i]);
                                        }
                                    }

                                    // observation:
                                    // 1) last packet not finished yet. Its printed in the monitor though.
                                    // 2) first two bytes are 170 and 86 --> EOF from last frame?
                                    // 3) FIXED on 11.08.2025: Some corrupted packets in the middle of the buffer, although SOF and EOF are visible. Exp datalength = 44, measured data length = 307 - 264 + 1 = 44



                                    // Convert to array
                                    byte[] newArray = filteredList.ToArray();
                                    string resultString = Encoding.GetEncoding(28591).GetString(newArray);
                                    if (_serial_monitor_window != null)
                                    {
                                        _serial_monitor_window.TextBox_SerialMonitor.Text += resultString + "\n";
                                        _serial_monitor_window.TextBox_SerialMonitor.ScrollToEnd();
                                    }
                                }
                            }


                            // remove elements from buffer
                            //lastTrueElementIndex
                            int lastTrueIndex = -1; // -1 means "not found"
                            for (int i = currentBufferLength - 1; i >= 0; i--)
                            {
                                if (bufferByteAssignedToStruct[i])
                                {
                                    lastTrueIndex = i;
                                    break;
                                }
                            }

                            if (lastTrueIndex > (-1))
                            {
                                int remainingMessageLength = currentBufferLength - (lastTrueIndex + 1);
                                if (remainingMessageLength > 0)
                                {
                                    appendedBufferOffset[pedalSelected] = remainingMessageLength;

                                    Buffer.BlockCopy(buffer_appended[pedalSelected], lastTrueIndex + 1, buffer_appended[pedalSelected], 0, remainingMessageLength);
                                    Array.Clear(buffer_appended[pedalSelected], appendedBufferOffset[pedalSelected], bufferSize - appendedBufferOffset[pedalSelected]); // 120 - 20 + 1 = 101 elements



                                    if (!((buffer_appended[pedalSelected][0] == 170) && (buffer_appended[pedalSelected][1] == 85)))
                                    {
                                        int tmp = 5;
                                    }
                                }
                                else
                                {
                                    appendedBufferOffset[pedalSelected] = 0;
                                }
                            }
                            else
                            {
                                appendedBufferOffset[pedalSelected] += receivedLength;
                            }




                        }
                        else
                        {
                            int srcBufferOffset = 0;
                            // decode every message
                            //foreach (int number in indices)
                            for (int msgId = 0; msgId < indices.Count; msgId++)
                            {
                                // computes the length of bytes to read
                                int destBuffLength = 0; //number - srcBufferOffset;

                                if (msgId == 0)
                                {
                                    srcBufferOffset = 0;
                                    destBuffLength = indices.ElementAt(msgId);
                                }
                                else
                                {
                                    srcBufferOffset = indices.ElementAt(msgId - 1) + stop_char_length;
                                    destBuffLength = indices.ElementAt(msgId) - srcBufferOffset;
                                }

                                // check if dest buffer length is within valid length
                                if ((destBuffLength <= 0) | (destBuffLength > destBufferSize))
                                {
                                    continue;
                                }

                                // copy bytes to subarray
                                Buffer.BlockCopy(buffer_appended[pedalSelected], srcBufferOffset, destinationArray, 0, destBuffLength);

                                // check for pedal state struct
                                if ((destBuffLength == sizeof(DAP_state_basic_st)))
                                {

                                    // parse byte array as config struct
                                    DAP_state_basic_st pedalState_read_st = getStateFromBytes(destinationArray);

                                    // check whether receive struct is plausible
                                    DAP_state_basic_st* v_state = &pedalState_read_st;
                                    byte* p_state = (byte*)v_state;

                                    // payload type check
                                    bool check_payload_state_b = false;
                                    if (pedalState_read_st.payloadHeader_.payloadType == Constants.pedalStateBasicPayload_type)
                                    {
                                        check_payload_state_b = true;
                                    }

                                    //Pedal version and Plugin DAP version check
                                    Pedal_version[pedalSelected] = pedalState_read_st.payloadHeader_.version;
                                    /*
                                    if (Pedal_version[pedalSelected] != Constants.pedalConfigPayload_version && pedalState_read_st.payloadHeader_.payloadType == Constants.pedalStateBasicPayload_type)
                                    {
                                        if (!Version_warning_first_show_b[pedalSelected])
                                        {
                                            Version_warning_first_show_b[pedalSelected] = true;
                                            if (Pedal_version[pedalSelected] > Constants.pedalConfigPayload_version)
                                            {
                                                String MSG_tmp;
                                                MSG_tmp = "Pedal: " + pedalState_read_st.payloadHeader_.PedalTag + " Pedal Dap version: " + Pedal_version[pedalSelected] + ", Plugin DAP version: " + Constants.pedalConfigPayload_version + ". Please update Simhub Plugin.";
                                                System.Windows.MessageBox.Show(MSG_tmp, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                                            }
                                            else
                                            {
                                                String MSG_tmp;
                                                MSG_tmp = "Pedal: " + pedalState_read_st.payloadHeader_.PedalTag + " Pedal Dap version: " + Pedal_version[pedalSelected] + ", Plugin DAP version: " + Constants.pedalConfigPayload_version + ". Please update Pedal Firmware.";
                                                System.Windows.MessageBox.Show(MSG_tmp, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                                            }
                                        }
                                    }
                                    */


                                    // CRC check
                                    bool check_crc_state_b = false;
                                    if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Basic)) == pedalState_read_st.payloadFooter_.checkSum)
                                    {
                                        check_crc_state_b = true;
                                    }

                                    if ((check_payload_state_b) && check_crc_state_b)
                                    {

                                        // write vJoy data
                                        Pedal_position_reading[pedalSelected] = pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16;
                                        //if (Plugin.Rudder_enable_flag == false)
                                        //{
                                        if (Plugin.Settings.vjoy_output_flag == 1)
                                        {
                                            switch (pedalSelected)
                                            {

                                                case 0:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RX);  // Center X axis
                                                    Plugin._calculations._joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);   // HID_USAGES Enums
                                                    break;
                                                case 1:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RY);  // Center X axis
                                                    Plugin._calculations._joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);   // HID_USAGES Enums
                                                    break;
                                                case 2:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RZ);  // Center X axis
                                                    Plugin._calculations._joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);   // HID_USAGES Enums
                                                    break;
                                                default:
                                                    break;
                                            }

                                        }

                                        //check servo status change
                                        if (Plugin._calculations.ServoStatus[pedalSelected] == (byte)enumServoStatus.On && pedalState_read_st.payloadPedalBasicState_.servoStatus == (byte)enumServoStatus.Idle)
                                        {
                                            string tmp = "Pedal:" + pedalSelected + " Servo idle reach timeout, power cutoff, please restart pedal to wake it up";
                                            ToastNotification("Wireless Connection", tmp);
                                        }
                                        // Force stop action
                                        if (Plugin._calculations.ServoStatus[pedalSelected] == (byte)enumServoStatus.On && pedalState_read_st.payloadPedalBasicState_.servoStatus == (byte)enumServoStatus.ForceStop)
                                        {
                                            string tmp = "Pedal:" + pedalSelected + " force Stopped";
                                            ToastNotification("Wireless Connection", tmp);
                                        }

                                        //fill servo status

                                        Plugin._calculations.ServoStatus[pedalSelected] = pedalState_read_st.payloadPedalBasicState_.servoStatus;



                                        // GUI update
                                        if ((pedalStateHasAlreadyBeenUpdated_b == false) && (indexOfSelectedPedal_u == pedalSelected))
                                        {
                                            //TextBox_debugOutput.Text = "Pedal pos: " + pedalState_read_st.payloadPedalState_.pedalPosition_u16;
                                            //TextBox_debugOutput.Text += "Pedal force: " + pedalState_read_st.payloadPedalState_.pedalForce_u16;
                                            //TextBox_debugOutput.Text += ",  Servo pos targe: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;
                                            //TextBox_debugOutput.Text += ",  Servo pos: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;

                                            PedalForceTravel_Tab.updatePedalState(pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16, pedalState_read_st.payloadPedalBasicState_.pedalForce_u16);

                                            pedalStateHasAlreadyBeenUpdated_b = true;

                                            double control_rect_value_max = 65535;


                                            if (Plugin.Settings.advanced_b)
                                            {
                                                int round_x = (int)(100 * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 / control_rect_value_max) - 1;
                                                int x_showed = round_x + 1;

                                                current_pedal_travel_state = x_showed;
                                                Plugin.pedal_state_in_ratio = (byte)current_pedal_travel_state;

                                            }
                                            else
                                            {
                                                int round_x = (int)(100 * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 / control_rect_value_max) - 1;
                                                int x_showed = round_x + 1;
                                                round_x = Math.Max(0, Math.Min(round_x, 99));
                                                current_pedal_travel_state = x_showed;
                                                Plugin.pedal_state_in_ratio = (byte)current_pedal_travel_state;
                                            }
                                            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.travelAsJoystickOutput_u8 == 1)
                                            {
                                                PedalJoystick_Tab.JoystickStateUpdate(pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16);
                                            }
                                            else
                                            {
                                                PedalJoystick_Tab.JoystickStateUpdate(pedalState_read_st.payloadPedalBasicState_.pedalForce_u16);
                                            }
                                            for (int i = 0; i < 3; i++)
                                            {
                                                //PedalFirmwareVersion[pedalSelected, i] = pedalState_read_st.payloadPedalBasicState_.pedalFirmwareVersion_u8[i];
                                                Plugin._calculations.PedalFirmwareVersion[pedalSelected, i] = pedalState_read_st.payloadPedalBasicState_.pedalFirmwareVersion_u8[i];
                                            }


                                        }


                                        continue;
                                    }

                                }

                                // check for pedal extended state struct
                                if ((destBuffLength == sizeof(DAP_state_extended_st)))
                                {

                                    // parse byte array as config struct
                                    DAP_state_extended_st pedalState_ext_read_st = getStateExtFromBytes(destinationArray);

                                    // check whether receive struct is plausible
                                    DAP_state_extended_st* v_state = &pedalState_ext_read_st;
                                    byte* p_state = (byte*)v_state;

                                    // payload type check
                                    bool check_payload_state_b = false;
                                    if (pedalState_ext_read_st.payloadHeader_.payloadType == Constants.pedalStateExtendedPayload_type)
                                    {
                                        check_payload_state_b = true;
                                    }

                                    // CRC check
                                    bool check_crc_state_b = false;
                                    if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Extended)) == pedalState_ext_read_st.payloadFooter_.checkSum)
                                    {
                                        check_crc_state_b = true;
                                    }

                                    if ((check_payload_state_b) && check_crc_state_b)
                                    {




                                        if (indexOfSelectedPedal_u == pedalSelected)
                                        {

                                            if (Plugin._calculations.dumpPedalToResponseFile[indexOfSelectedPedal_u])
                                            //if (dumpPedalToResponseFile[indexOfSelectedPedal_u])
                                            {
                                                // Specify the path to the file
                                                string currentDirectory = Directory.GetCurrentDirectory();
                                                string filePath = currentDirectory + "\\PluginsData\\Common" + "\\DiyFfbPedalStateLog_" + indexOfSelectedPedal_u.ToString() + ".txt";


                                                // delete file 
                                                if (true == Plugin._calculations.dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u])
                                                {
                                                    Plugin._calculations.dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u] = false;
                                                    File.Delete(filePath);
                                                }


                                                // write header
                                                if (!File.Exists(filePath))
                                                {
                                                    using (StreamWriter writer = new StreamWriter(filePath, true))
                                                    {
                                                        // Write the content to the file
                                                        writer.Write("cycleCtr");
                                                        writer.Write(", time_InUs");
                                                        writer.Write(", forceRaw_InKg");
                                                        writer.Write(", forceFiltered_InKg");
                                                        writer.Write(", forceVelocity_InKgPerSec");
                                                        writer.Write(", servoPos_InSteps");
                                                        writer.Write(", servoPosEsp_InSteps");
                                                        writer.Write(", servoPosError_InSteps");
                                                        writer.Write(", servoCurrent_InPercent");
                                                        writer.Write(", servoVoltage_InV");
                                                        writer.Write(", angleSensorOutput");
                                                        writer.Write(", brakeResistorState_b");
                                                        writer.Write(", servoPosEstimated_InSteps");
                                                        //writer.Write(", servoPositionEstimated_stepperPos_i16");
                                                        writer.Write("\n");
                                                    }

                                                }


                                                // Use StreamWriter to write to the file
                                                using (StreamWriter writer = new StreamWriter(filePath, true))
                                                {
                                                    // Write the content to the file
                                                    writeCntr++;
                                                    writer.Write(writeCntr);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.timeInUs_u32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_raw_fl32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_filtered_fl32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.forceVel_est_fl32);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPosition_i16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionTarget_i16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servo_position_error_i16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servo_current_percent_i16);
                                                    writer.Write(", ");
                                                    writer.Write(((float)pedalState_ext_read_st.payloadPedalExtendedState_.servo_voltage_0p1V_i16) / 10.0);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.angleSensorOutput_ui16);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.brakeResistorState_b);
                                                    writer.Write(", ");
                                                    writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionEstimated_i16);
                                                    //writer.Write(", ");
                                                    //writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionEstimated_stepperPos_i16);
                                                    writer.Write("\n");
                                                }
                                            }
                                        }




                                        continue;
                                    }
                                }

                                // decode into config struct
                                if ((waiting_for_pedal_config[pedalSelected]) && (destBuffLength == sizeof(DAP_config_st)))
                                {

                                    // parse byte array as config struct
                                    DAP_config_st pedalConfig_read_st = getConfigFromBytes(destinationArray);

                                    // check whether receive struct is plausible
                                    DAP_config_st* v_config = &pedalConfig_read_st;
                                    byte* p_config = (byte*)v_config;

                                    // payload type check
                                    bool check_payload_config_b = false;
                                    if (pedalConfig_read_st.payloadHeader_.payloadType == Constants.pedalConfigPayload_type)
                                    {
                                        check_payload_config_b = true;
                                    }

                                    // CRC check
                                    bool check_crc_config_b = false;
                                    if (Plugin.checksumCalc(p_config, sizeof(payloadHeader) + sizeof(payloadPedalConfig)) == pedalConfig_read_st.payloadFooter_.checkSum)
                                    {
                                        check_crc_config_b = true;
                                    }

                                    if ((check_payload_config_b) && check_crc_config_b)
                                    {
                                        waiting_for_pedal_config[pedalSelected] = false;
                                        dap_config_st[pedalSelected] = pedalConfig_read_st;
                                        Plugin.PedalConfigRead_b[pedalSelected] = true;
                                        updateTheGuiFromConfig();

                                        continue;
                                    }
                                    else
                                    {
                                        TextBox2.Text = "Payload config test 1: " + check_payload_config_b;
                                        TextBox2.Text += "Payload config test 2: " + check_crc_config_b;
                                    }

                                }


                                // If non known array datatype was received, assume a text message was received and print it
                                // only print debug messages when debug mode is active as it degrades performance
                                if (/*Debug_check.IsChecked == true|| */_serial_monitor_window != null)
                                {
                                    byte[] destinationArray_sub = new byte[destBuffLength];
                                    Buffer.BlockCopy(destinationArray, 0, destinationArray_sub, 0, destBuffLength);
                                    string resultString = Encoding.GetEncoding(28591).GetString(destinationArray_sub);
                                    if (_serial_monitor_window != null)
                                    {
                                        _serial_monitor_window.TextBox_SerialMonitor.Text += resultString + "\n";
                                        _serial_monitor_window.TextBox_SerialMonitor.ScrollToEnd();
                                    }
                                    /*
                                    TextBox_serialMonitor.Text += resultString + "\n";
                                    TextBox_serialMonitor.ScrollToEnd();
                                    */
                                }






                                // When only a few messages are received, make the counter greater than N thus every message is printed
                                //if (destBuffLength < 100)
                                //{
                                //    printCtr = 600;
                                //}

                                //if (printCtr++ > 200)
                                //{
                                //    printCtr = 0;
                                //    TextBox_serialMonitor.Text += dataToSend + "\n";
                                //    TextBox_serialMonitor.ScrollToEnd();
                                //}





                            }

                            // copy the last not finished buffer element to begining of next cycles buffer
                            // and determine buffer offset
                            if (indices.Count > 0)
                            {
                                // If at least one crlf was detected, check whether it arrieved at the last bytes
                                int lastElement = indices.Last<int>();
                                int remainingMessageLength = currentBufferLength - (lastElement + stop_char_length);
                                if (remainingMessageLength > 0)
                                {
                                    appendedBufferOffset[pedalSelected] = remainingMessageLength;

                                    Buffer.BlockCopy(buffer_appended[pedalSelected], lastElement + stop_char_length, buffer_appended[pedalSelected], 0, remainingMessageLength);
                                }
                                else
                                {
                                    appendedBufferOffset[pedalSelected] = 0;
                                }
                            }
                            else
                            {
                                appendedBufferOffset[pedalSelected] += receivedLength;
                            }
                        }


                        






                        // Stop the stopwatch
                        stopwatch.Stop();

                        // Get the elapsed time
                        /*
                        TimeSpan elapsedTime = stopwatch.Elapsed;

                        timeCollector[pedalSelected] += elapsedTime.TotalMilliseconds;

                        if (timeCntr[pedalSelected] >= 50)
                        {


                            double avgTime = timeCollector[pedalSelected] / timeCntr[pedalSelected];
                            if (Plugin.Settings.advanced_b)
                            {
                                TextBox_debugOutput.Text = "Serial callback time in ms: " + avgTime.ToString();
                            }
                            timeCntr[pedalSelected] = 0;
                            timeCollector[pedalSelected] = 0;
                        }
                        */
                    }

                }
            }
        }
    }
}
