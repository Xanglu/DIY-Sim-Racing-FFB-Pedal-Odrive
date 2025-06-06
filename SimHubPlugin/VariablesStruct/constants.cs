using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace User.PluginSdkDemo
{
    static class Constants
    {
        // payload revisiom
        public const uint pedalConfigPayload_version = 149;


        // pyload types
        public const uint pedalConfigPayload_type = 100;
        public const uint pedalActionPayload_type = 110;
        public const uint pedalStateBasicPayload_type = 120;
        public const uint pedalStateExtendedPayload_type = 130;
        public const uint bridgeStatePayloadType = 210;
        public const uint Basic_Wifi_info_type = 220;
        public const string pluginVersion = "0.90.04";
    }

    public enum enumServoStatus
    {
        Off,
        On,
        Idle
    }
    public enum bridgeAction
    {
        BRIDGE_ACTION_NONE,
        BRIDGE_ACTION_ENABLE_PAIRING,
        BRIDGE_ACTION_RESTART,
        BRIDGE_ACTION_DOWNLOAD_MODE,
        BRIDGE_ACTION_DEBUG,
        BRIDGE_ACTION_JOYSTICK_FLASHING_MODE,
        BRIDGE_ACTION_JOYSTICK_DEBUG
    };
}
