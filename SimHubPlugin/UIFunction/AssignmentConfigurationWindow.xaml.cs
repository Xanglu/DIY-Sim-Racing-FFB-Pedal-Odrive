using SimHub.Plugins.OutputPlugins.GraphicalDash.Behaviors.DoubleText.Imp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace User.PluginSdkDemo.UIFunction
{
    /// <summary>
    /// AssignmentConfigurationWindow.xaml 的互動邏輯
    /// </summary>
    public partial class AssignmentConfigurationWindow : Window
    {
        public class PedalItem
        {
            public string Name { get; set; }
            public int Id { get; set; }
        }
        private List<PedalItem> _pedalList;
        private List<PedalItem> _unassignedPedalList;
        private DIY_FFB_Pedal _plugin;
        private int _pedalSelect;
        private int _pedalTargetAssignment;
        private string[] _pedalName= new string[4] {"NA", "Clutch", "Brake", "Throttle"};
        private int[] _pedalActionId = new int[4] { (int)PedalSystemAction.NONE, (int)PedalSystemAction.SET_ASSIGNMENT_0, (int)PedalSystemAction.SET_ASSIGNMENT_1, (int)PedalSystemAction.SET_ASSIGNMENT_2 };
        private string[] _unassignedPedalName = new string[4] {"NA", "#1", "#2", "#3" };
        private int[] _unassignedPedalId = new int[4] { (int)PedalIdEnum.PEDAL_ID_UNKNOWN, (int)PedalIdEnum.PEDAL_ID_TEMP_1, (int)PedalIdEnum.PEDAL_ID_TEMP_2, (int)PedalIdEnum.PEDAL_ID_TEMP_3 };
        public AssignmentConfigurationWindow(DIY_FFB_Pedal Plugin)
        {
            InitializeComponent();
            _plugin = Plugin;
            _pedalList = new List<PedalItem>();
            _unassignedPedalList = new List<PedalItem>();
            for (int i = 0; i < 4; i++)
            {
                if (i != 0)
                {
                    
                    if (!Plugin._calculations.PedalAvailability[i-1])
                    {
                        PedalItem item = new PedalItem();
                        item.Name = _pedalName[i];
                        item.Id = _pedalActionId[i];
                        _pedalList.Add(item);
                    }
                }
                else
                {
                    PedalItem item = new PedalItem();
                    item.Name = _pedalName[i];
                    item.Id = _pedalActionId[i];
                    _pedalList.Add(item);
                }

            }
            for (int i = 0; i < Plugin._calculations.unassignedPedalCount+1;i++)
            {
                PedalItem item = new PedalItem();
                item.Name = _unassignedPedalName[i];
                item.Id = _unassignedPedalId[i];
                _unassignedPedalList.Add(item);
            }
            _pedalSelect = -1;
            _pedalTargetAssignment = -1;
            ComboBoxPedalSlot.ItemsSource = _pedalList;
            ComboBoxPedalSlot.DisplayMemberPath = "Name";
            ComboBoxPedalSlot.SelectedValuePath = "Id";
            ComboBoxUnassignedPedal.ItemsSource = _unassignedPedalList;
            ComboBoxUnassignedPedal.DisplayMemberPath = "Name";
            ComboBoxUnassignedPedal.SelectedValuePath = "Id";
            ComboBoxUnassignedPedal.SelectedIndex = 0;
            ComboBoxPedalSlot.SelectedIndex = 0;
            /*
            Binding bindingPedal = new Binding("_pedalTargetAssignment");
            bindingPedal.Source = this; 
            bindingPedal.Mode = BindingMode.TwoWay;
            BindingOperations.SetBinding(ComboBoxPedalSlot, ComboBox.SelectedValueProperty, bindingPedal);
            Binding bindingUnassignedPedal = new Binding("_pedalSelect");
            bindingUnassignedPedal.Source = this;
            bindingUnassignedPedal.Mode = BindingMode.TwoWay;
            BindingOperations.SetBinding(ComboBoxUnassignedPedal, ComboBox.SelectedValueProperty, bindingUnassignedPedal);
            */

        }

        unsafe private void Btn_Beep_Click(object sender, RoutedEventArgs e)
        {
            DAP_action_st tmp_action = default;
            tmp_action.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            tmp_action.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;
            tmp_action.payloadHeader_.PedalTag = (byte)_pedalSelect;
            //tmp_action.payloadHeader_.PedalTag = (byte)PedalIdEnum.PEDAL_ID_TEMP_1;
            tmp_action.payloadPedalAction_.system_action_u8 = (byte)PedalSystemAction.ASSIGNMENT_CHECK_BEEP;
            _plugin.SendPedalActionWireless(tmp_action, (byte)_pedalSelect);
            //_plugin.SendPedalActionWireless(tmp_action, (byte)PedalIdEnum.PEDAL_ID_TEMP_1);

        }

        private void Btn_Assign_Click(object sender, RoutedEventArgs e)
        {
            if (_pedalSelect != (int)PedalIdEnum.PEDAL_ID_UNKNOWN && _pedalTargetAssignment != (int)PedalSystemAction.NONE)
            {
                DAP_action_st tmp_action = default;
                tmp_action.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
                tmp_action.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;
                tmp_action.payloadHeader_.PedalTag = (byte)_pedalSelect;
                //tmp_action.payloadHeader_.PedalTag = (byte)PedalIdEnum.PEDAL_ID_TEMP_1;
                tmp_action.payloadPedalAction_.system_action_u8 = (byte)_pedalTargetAssignment;
                _plugin.SendPedalActionWireless(tmp_action, (byte)_pedalSelect);
            }

            this.Close();

        }

        private void ComboBoxPedalSlot_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox cbo = (ComboBox)sender;
            if (cbo.SelectedValue != null)
            {
                _pedalTargetAssignment = (int)cbo.SelectedValue;
            }
            else
            {
                _pedalTargetAssignment = -2;
            }
            //_pedalSelect = (int)ComboBoxUnassignedPedal.SelectedValue;
            Label_debug.Content = "Unassigned:"+_pedalSelect+" To:"+_pedalTargetAssignment;
        }

        private void ComboBoxUnassignedPedal_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox cbo = (ComboBox)sender;
            if (cbo.SelectedValue != null)
            {
                _pedalSelect = (int)cbo.SelectedValue;
            }
            else
            {
                _pedalSelect = -2;
            }

            Label_debug.Content = "Unassigned:" + _pedalSelect + " To:" + _pedalTargetAssignment;
        }
    }
}
