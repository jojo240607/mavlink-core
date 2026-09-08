//! 地面站兼容层：`MavHeader` + `MavMessage` 枚举 + `*_DATA` 结构体 + 编解码入口。
//!
//! 命名与官方 `mavlink` crate（0.11，`common` 方言）对齐，使 groundctrl 迁移
//! 只需改 import 路径（`::mavlink` → `mavlink_core`），消息结构体、枚举、
//! `write_v2_msg` / `read_v2_msg` 用法保持一致。
//!
//! 字节序统一为标准 common.xml 字段顺序（与 [`crate::codec`] / 固件端一致）——
//! 因此 groundctrl 原先针对官方 crate 字段序差异做的 `try_parse_heartbeat_std` /
//! `try_parse_param_value_std` / `encode_std_command_long` 兼容垫片可全部删除。
//!
//! `no_std` + `alloc`：本层仅在需要堆的 host 侧使用（`Vec` 输出），固件不触碰。

//! MAVLink 标准消息/枚举命名均为 SCREAMING_SNAKE_CASE，与官方 `mavlink` crate 保持一致，
//! 故关闭对应的风格告警。

#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use crate::frame::{
    crc16_x25, MAVLINK_MAGIC, CRC_EXTRA, MAX_FRAME_LEN, msg_id,
};

// ── 帧头 ───────────────────────────────────────────────

/// MAVLink v2 帧头（`read_v2_msg` / `write_v2_msg` 共用）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MavHeader {
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
}

// ── 位域（手写最小 bitflags，替代官方 bitflags crate） ──

macro_rules! bitflags_u {
    ($name:ident, $ty:ty) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
        pub struct $name(pub $ty);
        impl $name {
            pub const fn empty() -> Self {
                Self(0)
            }
            pub const fn bits(&self) -> $ty {
                self.0
            }
            pub fn from_bits(bits: $ty) -> Option<Self> {
                Some(Self(bits))
            }
            pub fn from_bits_truncate(bits: $ty) -> Self {
                Self(bits)
            }
            pub fn is_empty(&self) -> bool {
                self.0 == 0
            }
            pub fn contains(&self, other: Self) -> bool {
                (self.0 & other.0) == other.0
            }
            pub fn insert(&mut self, other: Self) {
                self.0 |= other.0;
            }
            pub fn remove(&mut self, other: Self) {
                self.0 &= !other.0;
            }
        }
        impl core::ops::BitOr for $name {
            type Output = Self;
            fn bitor(self, rhs: Self) -> Self {
                Self(self.0 | rhs.0)
            }
        }
        impl core::ops::BitOrAssign for $name {
            fn bitor_assign(&mut self, rhs: Self) {
                self.0 |= rhs.0;
            }
        }
    };
}

bitflags_u!(MavModeFlag, u8);
bitflags_u!(MavSysStatusSensor, u32);

impl MavModeFlag {
    pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: MavModeFlag = MavModeFlag(0x80);
    pub const MAV_MODE_FLAG_TEST_ENABLED: MavModeFlag = MavModeFlag(0x02);
    pub const MAV_MODE_FLAG_STABILIZE_ENABLED: MavModeFlag = MavModeFlag(0x04);
    pub const MAV_MODE_FLAG_GUIDED_ENABLED: MavModeFlag = MavModeFlag(0x08);
    pub const MAV_MODE_FLAG_AUTO_ENABLED: MavModeFlag = MavModeFlag(0x10);
    pub const MAV_MODE_FLAG_HIL_ENABLED: MavModeFlag = MavModeFlag(0x20);
    pub const MAV_MODE_FLAG_SAFETY_ARMED: MavModeFlag = MavModeFlag(0x80);
}

impl MavSysStatusSensor {
    pub const MAV_SYS_STATUS_SENSOR_3D_GYRO: MavSysStatusSensor = MavSysStatusSensor(1 << 0);
    pub const MAV_SYS_STATUS_SENSOR_3D_ACCEL: MavSysStatusSensor = MavSysStatusSensor(1 << 1);
    pub const MAV_SYS_STATUS_SENSOR_3D_MAG: MavSysStatusSensor = MavSysStatusSensor(1 << 2);
    pub const MAV_SYS_STATUS_SENSOR_GPS: MavSysStatusSensor = MavSysStatusSensor(1 << 5);
    pub const MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE: MavSysStatusSensor = MavSysStatusSensor(1 << 8);
    pub const MAV_SYS_STATUS_SENSOR_BATTERY: MavSysStatusSensor = MavSysStatusSensor(1 << 11);
    pub const MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL: MavSysStatusSensor = MavSysStatusSensor(1 << 17);
    pub const MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION: MavSysStatusSensor = MavSysStatusSensor(1 << 18);
    pub const MAV_SYS_STATUS_SENSOR_YAW_POSITION: MavSysStatusSensor = MavSysStatusSensor(1 << 20);
    pub const MAV_SYS_STATUS_SENSOR_RC_RECEIVER: MavSysStatusSensor = MavSysStatusSensor(1 << 22);
}

// ── 枚举（repr 对齐 common.xml，供 `transmute` / `as u8` / `from_*` 使用） ──

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum MavType {
    #[default]
    MAV_TYPE_GENERIC = 0,
    MAV_TYPE_FIXED_WING = 1,
    MAV_TYPE_QUADROTOR = 2,
    MAV_TYPE_COAXIAL = 3,
    MAV_TYPE_HELICOPTER = 4,
    MAV_TYPE_ANTENNA_TRACKER = 5,
    MAV_TYPE_GCS = 6,
    MAV_TYPE_AIRSHIP = 7,
    MAV_TYPE_FREE_BALLOON = 8,
    MAV_TYPE_ROCKET = 9,
    MAV_TYPE_GROUND_ROVER = 10,
    MAV_TYPE_SURFACE_BOAT = 11,
    MAV_TYPE_SUBMARINE = 12,
    MAV_TYPE_HEXAROTOR = 13,
    MAV_TYPE_OCTOROTOR = 14,
    MAV_TYPE_TRICOPTER = 15,
    MAV_TYPE_FLAPPING_WING = 16,
    MAV_TYPE_KITE = 17,
    MAV_TYPE_ONBOARD_CONTROLLER = 18,
    MAV_TYPE_VTOL_TAILSITTER_DUOROTOR = 19,
    MAV_TYPE_VTOL_TAILSITTER_QUADROTOR = 20,
    MAV_TYPE_VTOL_TILTROTOR = 21,
}

impl MavType {
    pub fn from_u8(v: u8) -> Option<Self> {
        if (v as usize) < 22 { Some(unsafe { core::mem::transmute(v) }) } else { None }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum MavAutopilot {
    #[default]
    MAV_AUTOPILOT_GENERIC = 0,
    MAV_AUTOPILOT_RESERVED = 1,
    MAV_AUTOPILOT_SLUGS = 2,
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
    MAV_AUTOPILOT_OPENPILOT = 4,
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
    MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,
    MAV_AUTOPILOT_INVALID = 8,
    MAV_AUTOPILOT_PPZ = 9,
    MAV_AUTOPILOT_UDB = 10,
    MAV_AUTOPILOT_FP = 11,
    MAV_AUTOPILOT_PX4 = 12,
    MAV_AUTOPILOT_DEV = 13,
}

impl MavAutopilot {
    pub fn from_u8(v: u8) -> Option<Self> {
        if (v as usize) < 14 { Some(unsafe { core::mem::transmute(v) }) } else { None }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum MavState {
    #[default]
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT = 1,
    MAV_STATE_CALIBRATING = 2,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_CRITICAL = 5,
    MAV_STATE_EMERGENCY = 6,
    MAV_STATE_POWEROFF = 7,
    MAV_STATE_FLIGHT_TERMINATION = 8,
}

impl MavState {
    pub fn from_u8(v: u8) -> Option<Self> {
        if (v as usize) < 9 { Some(unsafe { core::mem::transmute(v) }) } else { None }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MavParamType {
    MAV_PARAM_TYPE_UINT8 = 1,
    MAV_PARAM_TYPE_INT8 = 2,
    MAV_PARAM_TYPE_UINT16 = 3,
    MAV_PARAM_TYPE_INT16 = 4,
    MAV_PARAM_TYPE_UINT32 = 5,
    MAV_PARAM_TYPE_INT32 = 6,
    MAV_PARAM_TYPE_UINT64 = 7,
    MAV_PARAM_TYPE_INT64 = 8,
    MAV_PARAM_TYPE_REAL32 = 9,
    MAV_PARAM_TYPE_REAL64 = 10,
}

impl Default for MavParamType {
    fn default() -> Self {
        MavParamType::MAV_PARAM_TYPE_UINT8
    }
}

impl MavParamType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(MavParamType::MAV_PARAM_TYPE_UINT8),
            2 => Some(MavParamType::MAV_PARAM_TYPE_INT8),
            3 => Some(MavParamType::MAV_PARAM_TYPE_UINT16),
            4 => Some(MavParamType::MAV_PARAM_TYPE_INT16),
            5 => Some(MavParamType::MAV_PARAM_TYPE_UINT32),
            6 => Some(MavParamType::MAV_PARAM_TYPE_INT32),
            7 => Some(MavParamType::MAV_PARAM_TYPE_UINT64),
            8 => Some(MavParamType::MAV_PARAM_TYPE_INT64),
            9 => Some(MavParamType::MAV_PARAM_TYPE_REAL32),
            10 => Some(MavParamType::MAV_PARAM_TYPE_REAL64),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum MavResult {
    #[default]
    MAV_RESULT_ACCEPTED = 0,
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    MAV_RESULT_DENIED = 2,
    MAV_RESULT_UNSUPPORTED = 3,
    MAV_RESULT_FAILED = 4,
    MAV_RESULT_IN_PROGRESS = 5,
    MAV_RESULT_CANCELLED = 6,
}

impl MavResult {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(MavResult::MAV_RESULT_ACCEPTED),
            1 => Some(MavResult::MAV_RESULT_TEMPORARILY_REJECTED),
            2 => Some(MavResult::MAV_RESULT_DENIED),
            3 => Some(MavResult::MAV_RESULT_UNSUPPORTED),
            4 => Some(MavResult::MAV_RESULT_FAILED),
            5 => Some(MavResult::MAV_RESULT_IN_PROGRESS),
            6 => Some(MavResult::MAV_RESULT_CANCELLED),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GpsFixType {
    #[default]
    GPS_FIX_TYPE_NO_GPS = 0,
    GPS_FIX_TYPE_NO_FIX = 1,
    GPS_FIX_TYPE_2D_FIX = 2,
    GPS_FIX_TYPE_3D_FIX = 3,
    GPS_FIX_TYPE_DGPS = 4,
    GPS_FIX_TYPE_RTK_FLOAT = 5,
    GPS_FIX_TYPE_RTK_FIXED = 6,
    GPS_FIX_TYPE_STATIC = 7,
    GPS_FIX_TYPE_PPP = 8,
}

impl GpsFixType {
    pub fn from_u8(v: u8) -> Option<Self> {
        if (v as usize) < 9 { Some(unsafe { core::mem::transmute(v) }) } else { None }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum FenceBreach {
    #[default]
    FENCE_BREACH_NONE = 0,
    FENCE_BREACH_MIN_ALT = 1,
    FENCE_BREACH_MAX_ALT = 2,
    FENCE_BREACH_BOUNDARY = 3,
}

impl FenceBreach {
    pub fn from_u8(v: u8) -> Option<Self> {
        if (v as usize) < 4 { Some(unsafe { core::mem::transmute(v) }) } else { None }
    }
}

/// 坐标系（MISSION_ITEM_INT.frame / Waypoint.frame）。
/// `repr(u8)`：地面站代码用 `unsafe transmute(u8)` 转换，必须与 u8 同布局。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum MavFrame {
    #[default]
    MAV_FRAME_GLOBAL = 0,
    MAV_FRAME_LOCAL_NED = 1,
    MAV_FRAME_MISSION = 2,
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
    MAV_FRAME_LOCAL_ENU = 4,
    MAV_FRAME_GLOBAL_INT = 5,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    MAV_FRAME_LOCAL_OFFSET_NED = 7,
    MAV_FRAME_BODY_NED = 8,
    MAV_FRAME_BODY_OFFSET_NED = 9,
    MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11,
}

impl MavFrame {
    pub fn from_u8(v: u8) -> Option<Self> {
        if (v as usize) < 12 { Some(unsafe { core::mem::transmute(v) }) } else { None }
    }
}

/// MAV_CMD（COMMAND_LONG.command / MISSION_ITEM_INT.command）。
/// `repr(u16)`：地面站代码用 `unsafe transmute(u16)` 转换，必须与 u16 同布局。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u16)]
pub enum MavCmd {
    #[default]
    MAV_CMD_NAV_WAYPOINT = 16,
    MAV_CMD_NAV_LOITER_UNLIM = 17,
    MAV_CMD_NAV_LOITER_TURNS = 18,
    MAV_CMD_NAV_LOITER_TIME = 19,
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
    MAV_CMD_NAV_LAND = 21,
    MAV_CMD_NAV_TAKEOFF = 22,
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
    MAV_CMD_NAV_LOITER_TO_ALT = 31,
    MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
    MAV_CMD_DO_SET_MODE = 176,
    MAV_CMD_DO_JUMP = 177,
    MAV_CMD_DO_CHANGE_SPEED = 178,
    MAV_CMD_DO_SET_HOME = 179,
    MAV_CMD_DO_SET_PARAMETER = 180,
    MAV_CMD_DO_SET_RELAY = 181,
    MAV_CMD_DO_REPEAT_RELAY = 182,
    MAV_CMD_DO_SET_SERVO = 183,
    MAV_CMD_DO_REPEAT_SERVO = 184,
    MAV_CMD_DO_FLIGHTTERMINATION = 185,
    MAV_CMD_DO_SET_MISSION_CURRENT = 224,
    MAV_CMD_DO_CONTROL_VIDEO = 200,
    MAV_CMD_DO_SET_ROI = 201,
    MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
    MAV_CMD_SET_MESSAGE_INTERVAL = 511,
    MAV_CMD_REQUEST_MESSAGE = 512,
    MAV_CMD_PREFLIGHT_CALIBRATION = 241,
    MAV_CMD_MISSION_START = 300,
    MAV_CMD_COMPONENT_ARM_DISARM = 400,
    MAV_CMD_GET_HOME_POSITION = 410,
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
    MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
}

impl MavCmd {
    /// 从原始 u16 命令字解析（未知命令返回 None，与官方 crate 行为一致）。
    pub fn from_u16(v: u16) -> Option<Self> {
        match v {
            16 => Some(MavCmd::MAV_CMD_NAV_WAYPOINT),
            17 => Some(MavCmd::MAV_CMD_NAV_LOITER_UNLIM),
            18 => Some(MavCmd::MAV_CMD_NAV_LOITER_TURNS),
            19 => Some(MavCmd::MAV_CMD_NAV_LOITER_TIME),
            20 => Some(MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH),
            21 => Some(MavCmd::MAV_CMD_NAV_LAND),
            22 => Some(MavCmd::MAV_CMD_NAV_TAKEOFF),
            30 => Some(MavCmd::MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT),
            31 => Some(MavCmd::MAV_CMD_NAV_LOITER_TO_ALT),
            82 => Some(MavCmd::MAV_CMD_NAV_SPLINE_WAYPOINT),
            176 => Some(MavCmd::MAV_CMD_DO_SET_MODE),
            177 => Some(MavCmd::MAV_CMD_DO_JUMP),
            178 => Some(MavCmd::MAV_CMD_DO_CHANGE_SPEED),
            179 => Some(MavCmd::MAV_CMD_DO_SET_HOME),
            180 => Some(MavCmd::MAV_CMD_DO_SET_PARAMETER),
            181 => Some(MavCmd::MAV_CMD_DO_SET_RELAY),
            182 => Some(MavCmd::MAV_CMD_DO_REPEAT_RELAY),
            183 => Some(MavCmd::MAV_CMD_DO_SET_SERVO),
            184 => Some(MavCmd::MAV_CMD_DO_REPEAT_SERVO),
            185 => Some(MavCmd::MAV_CMD_DO_FLIGHTTERMINATION),
            224 => Some(MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT),
            200 => Some(MavCmd::MAV_CMD_DO_CONTROL_VIDEO),
            201 => Some(MavCmd::MAV_CMD_DO_SET_ROI),
            206 => Some(MavCmd::MAV_CMD_DO_SET_CAM_TRIGG_DIST),
            511 => Some(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL),
            512 => Some(MavCmd::MAV_CMD_REQUEST_MESSAGE),
            241 => Some(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION),
            300 => Some(MavCmd::MAV_CMD_MISSION_START),
            400 => Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
            410 => Some(MavCmd::MAV_CMD_GET_HOME_POSITION),
            520 => Some(MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES),
            519 => Some(MavCmd::MAV_CMD_REQUEST_PROTOCOL_VERSION),
            _ => None,
        }
    }
}

/// 数据流（REQUEST_DATA_STREAM.req_stream_id）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum MavDataStream {
    #[default]
    MAV_DATA_STREAM_ALL = 0,
    MAV_DATA_STREAM_RAW_SENSORS = 1,
    MAV_DATA_STREAM_EXTENDED_STATUS = 2,
    MAV_DATA_STREAM_RC_CHANNELS = 3,
    MAV_DATA_STREAM_RAW_CONTROLLER = 4,
    MAV_DATA_STREAM_POSITION = 6,
    MAV_DATA_STREAM_EXTRA1 = 10,
    MAV_DATA_STREAM_EXTRA2 = 11,
    MAV_DATA_STREAM_EXTRA3 = 12,
}

impl MavDataStream {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(MavDataStream::MAV_DATA_STREAM_ALL),
            1 => Some(MavDataStream::MAV_DATA_STREAM_RAW_SENSORS),
            2 => Some(MavDataStream::MAV_DATA_STREAM_EXTENDED_STATUS),
            3 => Some(MavDataStream::MAV_DATA_STREAM_RC_CHANNELS),
            4 => Some(MavDataStream::MAV_DATA_STREAM_RAW_CONTROLLER),
            6 => Some(MavDataStream::MAV_DATA_STREAM_POSITION),
            10 => Some(MavDataStream::MAV_DATA_STREAM_EXTRA1),
            11 => Some(MavDataStream::MAV_DATA_STREAM_EXTRA2),
            12 => Some(MavDataStream::MAV_DATA_STREAM_EXTRA3),
            _ => None,
        }
    }
}

// ── 消息数据体（与官方 crate 结构体同名字段，均带 Default） ──

#[derive(Debug, Clone, PartialEq, Default)]
pub struct HEARTBEAT_DATA {
    pub custom_mode: u32,
    pub mavtype: MavType,
    pub autopilot: MavAutopilot,
    pub base_mode: MavModeFlag,
    pub system_status: MavState,
    pub mavlink_version: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct SYS_STATUS_DATA {
    pub onboard_control_sensors_present: MavSysStatusSensor,
    pub onboard_control_sensors_enabled: MavSysStatusSensor,
    pub onboard_control_sensors_health: MavSysStatusSensor,
    pub load: u16,
    pub voltage_battery: u16,
    pub current_battery: i16,
    pub battery_remaining: i8,
    pub drop_rate_comm: u16,
    pub errors_comm: u16,
    pub errors_count1: u16,
    pub errors_count2: u16,
    pub errors_count3: u16,
    pub errors_count4: u16,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct ATTITUDE_DATA {
    pub time_boot_ms: u32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct GLOBAL_POSITION_INT_DATA {
    pub time_boot_ms: u32,
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub relative_alt: i32,
    pub vx: i16,
    pub vy: i16,
    pub vz: i16,
    pub hdg: u16,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct GPS_RAW_INT_DATA {
    pub time_usec: u64,
    pub fix_type: GpsFixType,
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub eph: u16,
    pub epv: u16,
    pub vel: u16,
    pub cog: u16,
    pub satellites_visible: u8,
    pub alt_ellipsoid: i32,
    pub h_acc: u32,
    pub v_acc: u32,
    pub vel_acc: u32,
    pub hdg_acc: u32,
    pub yaw: u16,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct VFR_HUD_DATA {
    pub airspeed: f32,
    pub groundspeed: f32,
    pub heading: i16,
    pub throttle: u16,
    pub alt: f32,
    pub climb: f32,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct RC_CHANNELS_DATA {
    pub time_boot_ms: u32,
    pub chan1_raw: u16,
    pub chan2_raw: u16,
    pub chan3_raw: u16,
    pub chan4_raw: u16,
    pub chan5_raw: u16,
    pub chan6_raw: u16,
    pub chan7_raw: u16,
    pub chan8_raw: u16,
    pub chan9_raw: u16,
    pub chan10_raw: u16,
    pub chan11_raw: u16,
    pub chan12_raw: u16,
    pub chan13_raw: u16,
    pub chan14_raw: u16,
    pub chan15_raw: u16,
    pub chan16_raw: u16,
    pub chan17_raw: u16,
    pub chan18_raw: u16,
    pub chancount: u8,
    pub rssi: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct RC_CHANNELS_OVERRIDE_DATA {
    pub chan1_raw: u16,
    pub chan2_raw: u16,
    pub chan3_raw: u16,
    pub chan4_raw: u16,
    pub chan5_raw: u16,
    pub chan6_raw: u16,
    pub chan7_raw: u16,
    pub chan8_raw: u16,
    pub chan9_raw: u16,
    pub chan10_raw: u16,
    pub chan11_raw: u16,
    pub chan12_raw: u16,
    pub chan13_raw: u16,
    pub chan14_raw: u16,
    pub chan15_raw: u16,
    pub chan16_raw: u16,
    pub chan17_raw: u16,
    pub chan18_raw: u16,
    pub target_system: u8,
    pub target_component: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct FENCE_STATUS_DATA {
    pub breach_status: u8,
    pub breach_count: u16,
    pub breach_type: FenceBreach,
    pub breach_time: u32,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct PARAM_VALUE_DATA {
    pub param_id: [u8; 16],
    pub param_value: f32,
    pub param_type: MavParamType,
    pub param_count: u16,
    pub param_index: u16,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct PARAM_REQUEST_LIST_DATA {
    pub target_system: u8,
    pub target_component: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct PARAM_REQUEST_READ_DATA {
    pub param_id: [u8; 16],
    pub param_index: i16,
    pub target_system: u8,
    pub target_component: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct PARAM_SET_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub param_id: [u8; 16],
    pub param_value: f32,
    pub param_type: MavParamType,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct COMMAND_LONG_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub command: MavCmd,
    pub confirmation: u8,
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub param5: f32,
    pub param6: f32,
    pub param7: f32,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct COMMAND_ACK_DATA {
    pub command: MavCmd,
    pub result: MavResult,
    pub progress: u8,
    pub result_param2: i32,
    pub target_system: u8,
    pub target_component: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct MISSION_ITEM_INT_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub seq: u16,
    pub frame: MavFrame,
    pub command: MavCmd,
    pub current: u8,
    pub autocontinue: u8,
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub x: i32,
    pub y: i32,
    pub z: f32,
    pub mission_type: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct MISSION_REQUEST_LIST_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub mission_type: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct MISSION_REQUEST_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub mission_type: u8,
    pub seq: u16,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct MISSION_CLEAR_ALL_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub mission_type: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct MISSION_COUNT_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub count: u16,
    pub mission_type: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct REQUEST_DATA_STREAM_DATA {
    pub target_system: u8,
    pub target_component: u8,
    pub req_stream_id: MavDataStream,
    pub req_message_rate: u16,
    pub start_stop: u8,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct AUTOPILOT_VERSION_DATA {
    pub capabilities: u64,
    pub flight_sw_version: u32,
    pub middleware_sw_version: u32,
    pub os_sw_version: u32,
    pub board_version: u32,
    pub vendor_id: u16,
    pub product_id: u16,
    pub uid: [u8; 18],
}

// ── 消息枚举 ────────────────────────────────────────────

/// 项目统一消息类型（对应官方 crate 的 `common::MavMessage` 扁平枚举）。
#[derive(Debug, Clone, PartialEq)]
pub enum MavMessage {
    HEARTBEAT(HEARTBEAT_DATA),
    SYS_STATUS(SYS_STATUS_DATA),
    ATTITUDE(ATTITUDE_DATA),
    GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA),
    GPS_RAW_INT(GPS_RAW_INT_DATA),
    VFR_HUD(VFR_HUD_DATA),
    RC_CHANNELS(RC_CHANNELS_DATA),
    RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA),
    FENCE_STATUS(FENCE_STATUS_DATA),
    PARAM_VALUE(PARAM_VALUE_DATA),
    PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA),
    PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA),
    PARAM_SET(PARAM_SET_DATA),
    COMMAND_LONG(COMMAND_LONG_DATA),
    COMMAND_ACK(COMMAND_ACK_DATA),
    MISSION_ITEM_INT(MISSION_ITEM_INT_DATA),
    MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA),
    MISSION_REQUEST(MISSION_REQUEST_DATA),
    MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA),
    MISSION_COUNT(MISSION_COUNT_DATA),
    REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA),
    AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA),
}

impl MavMessage {
    /// 消息的 CRC_EXTRA（按 msgid，标准 common.xml 常量）。
    pub fn extra_crc(id: u32) -> u8 {
        if (id as usize) < CRC_EXTRA.len() {
            CRC_EXTRA[id as usize]
        } else {
            0
        }
    }
}

// ── 小端读写（本地辅助，frame 层只暴露到 u16/i16/f32/i32） ──

fn rd_u8(p: &[u8], off: usize) -> u8 {
    *p.get(off).unwrap_or(&0)
}
fn rd_u16(p: &[u8], off: usize) -> u16 {
    if p.len() >= off + 2 {
        u16::from_le_bytes([p[off], p[off + 1]])
    } else {
        0
    }
}
fn rd_i16(p: &[u8], off: usize) -> i16 {
    rd_u16(p, off) as i16
}
fn rd_u32(p: &[u8], off: usize) -> u32 {
    if p.len() >= off + 4 {
        u32::from_le_bytes([p[off], p[off + 1], p[off + 2], p[off + 3]])
    } else {
        0
    }
}
fn rd_i32(p: &[u8], off: usize) -> i32 {
    rd_u32(p, off) as i32
}
fn rd_u64(p: &[u8], off: usize) -> u64 {
    if p.len() >= off + 8 {
        let mut b = [0u8; 8];
        b.copy_from_slice(&p[off..off + 8]);
        u64::from_le_bytes(b)
    } else {
        0
    }
}
fn rd_f32(p: &[u8], off: usize) -> f32 {
    f32::from_bits(rd_u32(p, off))
}
fn put_u32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}
fn put_u64(buf: &mut [u8], off: usize, v: u64) {
    buf[off..off + 8].copy_from_slice(&v.to_le_bytes());
}
fn put_i8(buf: &mut [u8], off: usize, v: i8) {
    buf[off] = v as u8;
}

// ── 消息 <-> payload（标准 common.xml 字段顺序） ──

/// 把消息编码为 payload + msgid。返回 (msgid, 实际写入长度)。
fn encode_payload(msg: &MavMessage, pl: &mut [u8; 255]) -> (u32, usize) {
    match msg {
        MavMessage::HEARTBEAT(d) => {
            pl[0] = d.mavtype as u8;
            pl[1] = d.autopilot as u8;
            pl[2] = d.base_mode.bits();
            put_u32(pl, 3, d.custom_mode);
            pl[7] = d.system_status as u8;
            pl[8] = d.mavlink_version;
            (msg_id::HEARTBEAT, 9)
        }
        MavMessage::SYS_STATUS(d) => {
            put_u32(pl, 0, d.onboard_control_sensors_present.bits());
            put_u32(pl, 4, d.onboard_control_sensors_enabled.bits());
            put_u32(pl, 8, d.onboard_control_sensors_health.bits());
            crate::frame::put_u16(pl, 12, d.load);
            crate::frame::put_u16(pl, 14, d.voltage_battery);
            crate::frame::put_i16(pl, 16, d.current_battery);
            put_i8(pl, 18, d.battery_remaining);
            crate::frame::put_u16(pl, 19, d.drop_rate_comm);
            crate::frame::put_u16(pl, 21, d.errors_comm);
            crate::frame::put_u16(pl, 23, d.errors_count1);
            crate::frame::put_u16(pl, 25, d.errors_count2);
            crate::frame::put_u16(pl, 27, d.errors_count3);
            crate::frame::put_u16(pl, 29, d.errors_count4);
            (msg_id::SYS_STATUS, 31)
        }
        MavMessage::ATTITUDE(d) => {
            put_u32(pl, 0, d.time_boot_ms);
            crate::frame::put_f32(pl, 4, d.roll);
            crate::frame::put_f32(pl, 8, d.pitch);
            crate::frame::put_f32(pl, 12, d.yaw);
            crate::frame::put_f32(pl, 16, d.rollspeed);
            crate::frame::put_f32(pl, 20, d.pitchspeed);
            crate::frame::put_f32(pl, 24, d.yawspeed);
            (msg_id::ATTITUDE, 28)
        }
        MavMessage::GLOBAL_POSITION_INT(d) => {
            put_u32(pl, 0, d.time_boot_ms);
            crate::frame::put_i32(pl, 4, d.lat);
            crate::frame::put_i32(pl, 8, d.lon);
            crate::frame::put_i32(pl, 12, d.alt);
            crate::frame::put_i32(pl, 16, d.relative_alt);
            crate::frame::put_i16(pl, 20, d.vx);
            crate::frame::put_i16(pl, 22, d.vy);
            crate::frame::put_i16(pl, 24, d.vz);
            crate::frame::put_u16(pl, 26, d.hdg);
            (msg_id::GLOBAL_POSITION_INT, 28)
        }
        MavMessage::GPS_RAW_INT(d) => {
            put_u64(pl, 0, d.time_usec);
            pl[8] = d.fix_type as u8;
            crate::frame::put_i32(pl, 9, d.lat);
            crate::frame::put_i32(pl, 13, d.lon);
            crate::frame::put_i32(pl, 17, d.alt);
            crate::frame::put_u16(pl, 21, d.eph);
            crate::frame::put_u16(pl, 23, d.epv);
            crate::frame::put_u16(pl, 25, d.vel);
            crate::frame::put_u16(pl, 27, d.cog);
            pl[29] = d.satellites_visible;
            crate::frame::put_i32(pl, 30, d.alt_ellipsoid);
            put_u32(pl, 34, d.h_acc);
            put_u32(pl, 38, d.v_acc);
            put_u32(pl, 42, d.vel_acc);
            put_u32(pl, 46, d.hdg_acc);
            crate::frame::put_u16(pl, 50, d.yaw);
            (msg_id::GPS_RAW_INT, 52)
        }
        MavMessage::VFR_HUD(d) => {
            crate::frame::put_f32(pl, 0, d.airspeed);
            crate::frame::put_f32(pl, 4, d.groundspeed);
            crate::frame::put_i16(pl, 8, d.heading);
            crate::frame::put_u16(pl, 10, d.throttle);
            crate::frame::put_f32(pl, 12, d.alt);
            crate::frame::put_f32(pl, 16, d.climb);
            (msg_id::VFR_HUD, 20)
        }
        MavMessage::RC_CHANNELS(d) => {
            put_u32(pl, 0, d.time_boot_ms);
            let chans = [
                d.chan1_raw, d.chan2_raw, d.chan3_raw, d.chan4_raw,
                d.chan5_raw, d.chan6_raw, d.chan7_raw, d.chan8_raw,
                d.chan9_raw, d.chan10_raw, d.chan11_raw, d.chan12_raw,
                d.chan13_raw, d.chan14_raw, d.chan15_raw, d.chan16_raw,
                d.chan17_raw, d.chan18_raw,
            ];
            for (i, c) in chans.iter().enumerate() {
                crate::frame::put_u16(pl, 4 + i * 2, *c);
            }
            pl[40] = d.chancount;
            pl[41] = d.rssi;
            (msg_id::RC_CHANNELS, 42)
        }
        MavMessage::RC_CHANNELS_OVERRIDE(d) => {
            let chans = [
                d.chan1_raw, d.chan2_raw, d.chan3_raw, d.chan4_raw,
                d.chan5_raw, d.chan6_raw, d.chan7_raw, d.chan8_raw,
                d.chan9_raw, d.chan10_raw, d.chan11_raw, d.chan12_raw,
                d.chan13_raw, d.chan14_raw, d.chan15_raw, d.chan16_raw,
                d.chan17_raw, d.chan18_raw,
            ];
            for (i, c) in chans.iter().enumerate() {
                crate::frame::put_u16(pl, i * 2, *c);
            }
            pl[36] = d.target_system;
            pl[37] = d.target_component;
            (msg_id::RC_CHANNELS_OVERRIDE, 38)
        }
        MavMessage::FENCE_STATUS(d) => {
            pl[0] = d.breach_status;
            crate::frame::put_u16(pl, 1, d.breach_count);
            pl[3] = d.breach_type as u8;
            put_u32(pl, 4, d.breach_time);
            (msg_id::FENCE_STATUS, 8)
        }
        MavMessage::PARAM_VALUE(d) => {
            pl[..16].copy_from_slice(&d.param_id);
            crate::frame::put_f32(pl, 16, d.param_value);
            pl[20] = d.param_type as u8;
            crate::frame::put_u16(pl, 21, d.param_count);
            crate::frame::put_u16(pl, 23, d.param_index);
            (msg_id::PARAM_VALUE, 25)
        }
        MavMessage::PARAM_REQUEST_LIST(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            (msg_id::PARAM_REQUEST_LIST, 2)
        }
        MavMessage::PARAM_REQUEST_READ(d) => {
            pl[..16].copy_from_slice(&d.param_id);
            crate::frame::put_i16(pl, 16, d.param_index);
            pl[18] = d.target_system;
            pl[19] = d.target_component;
            (msg_id::PARAM_REQUEST_READ, 20)
        }
        MavMessage::PARAM_SET(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            pl[2..18].copy_from_slice(&d.param_id);
            crate::frame::put_f32(pl, 18, d.param_value);
            pl[22] = d.param_type as u8;
            (msg_id::PARAM_SET, 23)
        }
        MavMessage::COMMAND_LONG(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            crate::frame::put_u16(pl, 2, d.command as u16);
            pl[4] = d.confirmation;
            crate::frame::put_f32(pl, 5, d.param1);
            crate::frame::put_f32(pl, 9, d.param2);
            crate::frame::put_f32(pl, 13, d.param3);
            crate::frame::put_f32(pl, 17, d.param4);
            crate::frame::put_f32(pl, 21, d.param5);
            crate::frame::put_f32(pl, 25, d.param6);
            crate::frame::put_f32(pl, 29, d.param7);
            (msg_id::COMMAND_LONG, 33)
        }
        MavMessage::COMMAND_ACK(d) => {
            crate::frame::put_u16(pl, 0, d.command as u16);
            pl[2] = d.result as u8;
            pl[3] = d.progress;
            crate::frame::put_i32(pl, 4, d.result_param2);
            pl[8] = d.target_system;
            pl[9] = d.target_component;
            (msg_id::COMMAND_ACK, 10)
        }
        MavMessage::MISSION_ITEM_INT(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            crate::frame::put_u16(pl, 2, d.seq);
            pl[4] = d.frame as u8;
            crate::frame::put_u16(pl, 5, d.command as u16);
            pl[7] = d.current;
            pl[8] = d.autocontinue;
            crate::frame::put_f32(pl, 9, d.param1);
            crate::frame::put_f32(pl, 13, d.param2);
            crate::frame::put_f32(pl, 17, d.param3);
            crate::frame::put_f32(pl, 21, d.param4);
            crate::frame::put_i32(pl, 25, d.x);
            crate::frame::put_i32(pl, 29, d.y);
            crate::frame::put_f32(pl, 33, d.z);
            pl[37] = d.mission_type;
            (msg_id::MISSION_ITEM_INT, 38)
        }
        MavMessage::MISSION_REQUEST_LIST(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            pl[2] = d.mission_type;
            (msg_id::MISSION_REQUEST_LIST, 3)
        }
        MavMessage::MISSION_REQUEST(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            pl[2] = d.mission_type;
            crate::frame::put_u16(pl, 3, d.seq);
            (msg_id::MISSION_REQUEST, 5)
        }
        MavMessage::MISSION_CLEAR_ALL(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            pl[2] = d.mission_type;
            (msg_id::MISSION_CLEAR_ALL, 3)
        }
        MavMessage::MISSION_COUNT(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            crate::frame::put_u16(pl, 2, d.count);
            pl[4] = d.mission_type;
            (msg_id::MISSION_COUNT, 5)
        }
        MavMessage::REQUEST_DATA_STREAM(d) => {
            pl[0] = d.target_system;
            pl[1] = d.target_component;
            pl[2] = d.req_stream_id as u8;
            crate::frame::put_u16(pl, 3, d.req_message_rate);
            pl[5] = d.start_stop;
            (msg_id::REQUEST_DATA_STREAM, 6)
        }
        MavMessage::AUTOPILOT_VERSION(d) => {
            put_u64(pl, 0, d.capabilities);
            put_u32(pl, 8, d.flight_sw_version);
            put_u32(pl, 12, d.middleware_sw_version);
            put_u32(pl, 16, d.os_sw_version);
            put_u32(pl, 20, d.board_version);
            crate::frame::put_u16(pl, 24, d.vendor_id);
            crate::frame::put_u16(pl, 26, d.product_id);
            pl[28..46].copy_from_slice(&d.uid);
            (msg_id::AUTOPILOT_VERSION, 46)
        }
    }
}

/// 从 payload 解码消息（按 msgid 分发；未知/畸形返回 None）。
/// 对短 payload 宽容（缺省字段补 0），兼容固件端不含 mission_type 的旧 MISSION 帧。
fn decode_payload(msgid: u32, p: &[u8]) -> Option<MavMessage> {
    match msgid {
        msg_id::HEARTBEAT => {
            let mut pl = [0u8; 9];
            pl[..p.len().min(9)].copy_from_slice(&p[..p.len().min(9)]);
            Some(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: rd_u32(&pl, 3),
                mavtype: MavType::from_u8(rd_u8(&pl, 0)).unwrap_or_default(),
                autopilot: MavAutopilot::from_u8(rd_u8(&pl, 1)).unwrap_or_default(),
                base_mode: MavModeFlag::from_bits_truncate(rd_u8(&pl, 2)),
                system_status: MavState::from_u8(rd_u8(&pl, 7)).unwrap_or_default(),
                mavlink_version: rd_u8(&pl, 8),
            }))
        }
        msg_id::SYS_STATUS => Some(MavMessage::SYS_STATUS(SYS_STATUS_DATA {
            onboard_control_sensors_present: MavSysStatusSensor::from_bits_truncate(rd_u32(p, 0)),
            onboard_control_sensors_enabled: MavSysStatusSensor::from_bits_truncate(rd_u32(p, 4)),
            onboard_control_sensors_health: MavSysStatusSensor::from_bits_truncate(rd_u32(p, 8)),
            load: rd_u16(p, 12),
            voltage_battery: rd_u16(p, 14),
            current_battery: rd_i16(p, 16),
            battery_remaining: rd_u8(p, 18) as i8,
            drop_rate_comm: rd_u16(p, 19),
            errors_comm: rd_u16(p, 21),
            errors_count1: rd_u16(p, 23),
            errors_count2: rd_u16(p, 25),
            errors_count3: rd_u16(p, 27),
            errors_count4: rd_u16(p, 29),
        })),
        msg_id::ATTITUDE => Some(MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: rd_u32(p, 0),
            roll: rd_f32(p, 4),
            pitch: rd_f32(p, 8),
            yaw: rd_f32(p, 12),
            rollspeed: rd_f32(p, 16),
            pitchspeed: rd_f32(p, 20),
            yawspeed: rd_f32(p, 24),
        })),
        msg_id::GLOBAL_POSITION_INT => Some(MavMessage::GLOBAL_POSITION_INT(
            GLOBAL_POSITION_INT_DATA {
                time_boot_ms: rd_u32(p, 0),
                lat: rd_i32(p, 4),
                lon: rd_i32(p, 8),
                alt: rd_i32(p, 12),
                relative_alt: rd_i32(p, 16),
                vx: rd_i16(p, 20),
                vy: rd_i16(p, 22),
                vz: rd_i16(p, 24),
                hdg: rd_u16(p, 26),
            },
        )),
        msg_id::GPS_RAW_INT => Some(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: rd_u64(p, 0),
            fix_type: GpsFixType::from_u8(rd_u8(p, 8)).unwrap_or_default(),
            lat: rd_i32(p, 9),
            lon: rd_i32(p, 13),
            alt: rd_i32(p, 17),
            eph: rd_u16(p, 21),
            epv: rd_u16(p, 23),
            vel: rd_u16(p, 25),
            cog: rd_u16(p, 27),
            satellites_visible: rd_u8(p, 29),
            alt_ellipsoid: rd_i32(p, 30),
            h_acc: rd_u32(p, 34),
            v_acc: rd_u32(p, 38),
            vel_acc: rd_u32(p, 42),
            hdg_acc: rd_u32(p, 46),
            yaw: rd_u16(p, 50),
        })),
        msg_id::VFR_HUD => Some(MavMessage::VFR_HUD(VFR_HUD_DATA {
            airspeed: rd_f32(p, 0),
            groundspeed: rd_f32(p, 4),
            heading: rd_i16(p, 8),
            throttle: rd_u16(p, 10),
            alt: rd_f32(p, 12),
            climb: rd_f32(p, 16),
        })),
        msg_id::RC_CHANNELS => {
            let chans: Vec<u16> = (0..18).map(|i| rd_u16(p, 4 + i * 2)).collect();
            Some(MavMessage::RC_CHANNELS(RC_CHANNELS_DATA {
                time_boot_ms: rd_u32(p, 0),
                chan1_raw: chans[0], chan2_raw: chans[1], chan3_raw: chans[2],
                chan4_raw: chans[3], chan5_raw: chans[4], chan6_raw: chans[5],
                chan7_raw: chans[6], chan8_raw: chans[7], chan9_raw: chans[8],
                chan10_raw: chans[9], chan11_raw: chans[10], chan12_raw: chans[11],
                chan13_raw: chans[12], chan14_raw: chans[13], chan15_raw: chans[14],
                chan16_raw: chans[15], chan17_raw: chans[16], chan18_raw: chans[17],
                chancount: rd_u8(p, 40),
                rssi: rd_u8(p, 41),
            }))
        }
        msg_id::RC_CHANNELS_OVERRIDE => {
            let chans: Vec<u16> = (0..18).map(|i| rd_u16(p, i * 2)).collect();
            Some(MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
                chan1_raw: chans[0], chan2_raw: chans[1], chan3_raw: chans[2],
                chan4_raw: chans[3], chan5_raw: chans[4], chan6_raw: chans[5],
                chan7_raw: chans[6], chan8_raw: chans[7], chan9_raw: chans[8],
                chan10_raw: chans[9], chan11_raw: chans[10], chan12_raw: chans[11],
                chan13_raw: chans[12], chan14_raw: chans[13], chan15_raw: chans[14],
                chan16_raw: chans[15], chan17_raw: chans[16], chan18_raw: chans[17],
                target_system: rd_u8(p, 36),
                target_component: rd_u8(p, 37),
            }))
        }
        msg_id::FENCE_STATUS => Some(MavMessage::FENCE_STATUS(FENCE_STATUS_DATA {
            breach_status: rd_u8(p, 0),
            breach_count: rd_u16(p, 1),
            breach_type: FenceBreach::from_u8(rd_u8(p, 3)).unwrap_or_default(),
            breach_time: rd_u32(p, 4),
        })),
        msg_id::PARAM_VALUE => {
            let mut param_id = [0u8; 16];
            param_id[..p.len().min(16)].copy_from_slice(&p[..p.len().min(16)]);
            Some(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id,
                param_value: rd_f32(p, 16),
                param_type: MavParamType::from_u8(rd_u8(p, 20)).unwrap_or_default(),
                param_count: rd_u16(p, 21),
                param_index: rd_u16(p, 23),
            }))
        }
        msg_id::PARAM_REQUEST_LIST => Some(MavMessage::PARAM_REQUEST_LIST(
            PARAM_REQUEST_LIST_DATA {
                target_system: rd_u8(p, 0),
                target_component: rd_u8(p, 1),
            },
        )),
        msg_id::PARAM_REQUEST_READ => {
            let mut param_id = [0u8; 16];
            param_id[..p.len().min(16)].copy_from_slice(&p[..p.len().min(16)]);
            Some(MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                param_id,
                param_index: rd_i16(p, 16),
                target_system: rd_u8(p, 18),
                target_component: rd_u8(p, 19),
            }))
        }
        msg_id::PARAM_SET => {
            let mut param_id = [0u8; 16];
            let n = p.len().saturating_sub(2).min(16);
            param_id[..n].copy_from_slice(&p[2..2 + n]);
            Some(MavMessage::PARAM_SET(PARAM_SET_DATA {
                target_system: rd_u8(p, 0),
                target_component: rd_u8(p, 1),
                param_id,
                param_value: rd_f32(p, 18),
                param_type: MavParamType::from_u8(rd_u8(p, 22)).unwrap_or_default(),
            }))
        }
        msg_id::COMMAND_LONG => Some(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: rd_u8(p, 0),
            target_component: rd_u8(p, 1),
            command: MavCmd::from_u16(rd_u16(p, 2)).unwrap_or_default(),
            confirmation: rd_u8(p, 4),
            param1: rd_f32(p, 5),
            param2: rd_f32(p, 9),
            param3: rd_f32(p, 13),
            param4: rd_f32(p, 17),
            param5: rd_f32(p, 21),
            param6: rd_f32(p, 25),
            param7: rd_f32(p, 29),
        })),
        msg_id::COMMAND_ACK => Some(MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command: MavCmd::from_u16(rd_u16(p, 0)).unwrap_or_default(),
            result: MavResult::from_u8(rd_u8(p, 2)).unwrap_or_default(),
            progress: rd_u8(p, 3),
            result_param2: rd_i32(p, 4),
            target_system: rd_u8(p, 8),
            target_component: rd_u8(p, 9),
        })),
        msg_id::MISSION_ITEM_INT => Some(MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
            target_system: rd_u8(p, 0),
            target_component: rd_u8(p, 1),
            seq: rd_u16(p, 2),
            frame: MavFrame::from_u8(rd_u8(p, 4)).unwrap_or_default(),
            command: MavCmd::from_u16(rd_u16(p, 5)).unwrap_or_default(),
            current: rd_u8(p, 7),
            autocontinue: rd_u8(p, 8),
            param1: rd_f32(p, 9),
            param2: rd_f32(p, 13),
            param3: rd_f32(p, 17),
            param4: rd_f32(p, 21),
            x: rd_i32(p, 25),
            y: rd_i32(p, 29),
            z: rd_f32(p, 33),
            mission_type: rd_u8(p, 37),
        })),
        msg_id::MISSION_REQUEST_LIST => Some(MavMessage::MISSION_REQUEST_LIST(
            MISSION_REQUEST_LIST_DATA {
                target_system: rd_u8(p, 0),
                target_component: rd_u8(p, 1),
                mission_type: rd_u8(p, 2),
            },
        )),
        msg_id::MISSION_REQUEST => Some(MavMessage::MISSION_REQUEST(MISSION_REQUEST_DATA {
            target_system: rd_u8(p, 0),
            target_component: rd_u8(p, 1),
            mission_type: rd_u8(p, 2),
            seq: rd_u16(p, 3),
        })),
        msg_id::MISSION_CLEAR_ALL => Some(MavMessage::MISSION_CLEAR_ALL(
            MISSION_CLEAR_ALL_DATA {
                target_system: rd_u8(p, 0),
                target_component: rd_u8(p, 1),
                mission_type: rd_u8(p, 2),
            },
        )),
        msg_id::MISSION_COUNT => Some(MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            target_system: rd_u8(p, 0),
            target_component: rd_u8(p, 1),
            count: rd_u16(p, 2),
            mission_type: rd_u8(p, 4),
        })),
        msg_id::REQUEST_DATA_STREAM => Some(MavMessage::REQUEST_DATA_STREAM(
            REQUEST_DATA_STREAM_DATA {
                target_system: rd_u8(p, 0),
                target_component: rd_u8(p, 1),
                req_stream_id: MavDataStream::from_u8(rd_u8(p, 2)).unwrap_or_default(),
                req_message_rate: rd_u16(p, 3),
                start_stop: rd_u8(p, 5),
            },
        )),
        msg_id::AUTOPILOT_VERSION => {
            let mut uid = [0u8; 18];
            uid[..p.len().saturating_sub(28).min(18)].copy_from_slice(
                &p[28..28 + p.len().saturating_sub(28).min(18)],
            );
            Some(MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
                capabilities: rd_u64(p, 0),
                flight_sw_version: rd_u32(p, 8),
                middleware_sw_version: rd_u32(p, 12),
                os_sw_version: rd_u32(p, 16),
                board_version: rd_u32(p, 20),
                vendor_id: rd_u16(p, 24),
                product_id: rd_u16(p, 26),
                uid,
            }))
        }
        _ => None,
    }
}

// ── 帧级编解码入口（对应官方 crate 的 write_v2_msg / read_v2_msg） ──

/// 组一帧 MAVLink v2 报文（自定义 sys/comp，标准 common.xml 字段顺序）。
fn encode_full(
    msgid: u32,
    sys: u8,
    comp: u8,
    seq: u8,
    payload: &[u8],
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let plen = payload.len().min(255);
    out[0] = MAVLINK_MAGIC;
    out[1] = plen as u8;
    out[2] = 0; // incompat_flags
    out[3] = 0; // compat_flags
    out[4] = seq;
    out[5] = sys;
    out[6] = comp;
    out[7] = msgid as u8;
    out[8] = (msgid >> 8) as u8;
    out[9] = (msgid >> 16) as u8;
    out[10..10 + plen].copy_from_slice(&payload[..plen]);
    let mut crc = crc16_x25(0xFFFF, &out[1..10 + plen]);
    crc = crc16_x25(crc, &[CRC_EXTRA[msgid as usize]]);
    out[10 + plen] = (crc & 0xFF) as u8;
    out[10 + plen + 1] = (crc >> 8) as u8;
    10 + plen + 2
}

/// 由裸载荷组一帧 MAVLink v2（自定义 sys/comp/seq）。
/// 等价于 `write_v2_msg`，仅跳过消息结构体，供手写编解码的厂商扩展消息使用
/// （如地面站按原始 u16 命令字编码 COMMAND_LONG、FENCE_POINT、FILE_TRANSFER_PROTOCOL），
/// 保证与标准 common.xml 字段顺序 / CRC 字节级一致。
pub fn build_v2(msgid: u32, sys: u8, comp: u8, seq: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = [0u8; MAX_FRAME_LEN];
    let n = encode_full(msgid, sys, comp, seq, payload, &mut out);
    out[..n].to_vec()
}

/// 编码一条消息为完整 MAVLink v2 帧字节（等价于官方 crate `write_v2_msg`）。
pub fn write_v2_msg(header: &MavHeader, msg: &MavMessage) -> Result<Vec<u8>, String> {
    let mut pl = [0u8; 255];
    let (msgid, plen) = encode_payload(msg, &mut pl);
    let mut out = [0u8; MAX_FRAME_LEN];
    let n = encode_full(
        msgid,
        header.system_id,
        header.component_id,
        header.sequence,
        &pl[..plen],
        &mut out,
    );
    Ok(out[..n].to_vec())
}

/// 从字节缓冲解析第一帧。返回 `(header, msg, 消耗字节数)`；无完整合法帧返回 None。
/// 与官方 crate `read_v2_msg` 语义对齐（但返回已消耗长度，便于流式解析）。
pub fn read_v2_msg(bytes: &[u8]) -> Option<(MavHeader, MavMessage, usize)> {
    let mut pos = 0;
    while pos < bytes.len() {
        if bytes[pos] != MAVLINK_MAGIC {
            pos += 1;
            continue;
        }
        let avail = bytes.len() - pos;
        if avail < 12 {
            return None; // 头部未收全，等更多数据
        }
        let plen = bytes[pos + 1] as usize;
        let frame_len = 10 + plen + 2;
        if frame_len > avail {
            return None; // payload/crc 未收全，等更多数据
        }
        let hdr = &bytes[pos..pos + 10];
        let payload = &bytes[pos + 10..pos + 10 + plen];
        let msgid = hdr[7] as u32 | (hdr[8] as u32) << 8 | (hdr[9] as u32) << 16;
        if (msgid as usize) >= CRC_EXTRA.len() {
            pos += 1; // 未知消息 ID，继续找 magic
            continue;
        }
        let mut crc = crc16_x25(0xFFFF, &hdr[1..]);
        crc = crc16_x25(crc, payload);
        crc = crc16_x25(crc, &[CRC_EXTRA[msgid as usize]]);
        let crc_hi = bytes[pos + frame_len - 1] as u16;
        let crc_lo = bytes[pos + frame_len - 2] as u16;
        let got = (crc_hi << 8) | crc_lo;
        if crc != got {
            pos += 1; // CRC 不通过，跳过该 magic 字节继续找
            continue;
        }
        let msg = decode_payload(msgid, payload)?;
        let header = MavHeader {
            system_id: hdr[5],
            component_id: hdr[6],
            sequence: hdr[4],
        };
        return Some((header, msg, frame_len));
    }
    None
}

// ── 测试：标准字段顺序 + 往返一致性 ──

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn heartbeat_standard_layout() {
        let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        let h = MavHeader { system_id: 255, component_id: 190, sequence: 0 };
        let bytes = write_v2_msg(&h, &hb).unwrap();
        // 标准顺序：type, autopilot, base_mode, custom_mode(LE), system_status, version
        assert_eq!(&bytes[10..19], &[2, 3, 0x80, 0, 0, 0, 0, 4, 3]);
        assert_eq!(bytes[0], MAVLINK_MAGIC);
    }

    #[test]
    fn command_long_roundtrip() {
        let cmd = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });
        let h = MavHeader { system_id: 255, component_id: 190, sequence: 7 };
        let bytes = write_v2_msg(&h, &cmd).unwrap();
        let (rh, rmsg, _used) = read_v2_msg(&bytes).unwrap();
        assert_eq!(rh.system_id, 255);
        assert_eq!(rh.sequence, 7);
        match rmsg {
            MavMessage::COMMAND_LONG(d) => {
                assert_eq!(d.command, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
                assert_eq!(d.param1, 1.0);
                assert_eq!(d.target_system, 1);
            }
            other => panic!("wrong msg: {other:?}"),
        }
    }

    #[test]
    fn param_value_roundtrip() {
        let msg = MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_id: *b"WP_RADIUS\0\0\0\0\0\0\0",
            param_value: 12.5,
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
            param_count: 3,
            param_index: 1,
        });
        let h = MavHeader { system_id: 255, component_id: 190, sequence: 0 };
        let msg_id_bytes = *b"WP_RADIUS\0\0\0\0\0\0\0";
        let bytes = write_v2_msg(&h, &msg).unwrap();
        // 标准字段顺序：param_id(16) param_value(f32) param_type(u8) param_count(u16) param_index(u16)
        assert_eq!(&bytes[10..26], b"WP_RADIUS\0\0\0\0\0\0\0");
        assert_eq!(f32::from_le_bytes(bytes[26..30].try_into().unwrap()), 12.5);
        let (_, rmsg, _) = read_v2_msg(&bytes).unwrap();
        match rmsg {
            MavMessage::PARAM_VALUE(d) => {
                assert_eq!(&d.param_id[..], &msg_id_bytes);
                assert_eq!(d.param_count, 3);
                assert_eq!(d.param_index, 1);
            }
            other => panic!("wrong msg: {other:?}"),
        }
    }

    #[test]
    fn gps_raw_int_standard_layout() {
        let msg = MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: 1_700_000_000_000_000,
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            lat: 31_230_000,
            lon: 121_470_000,
            alt: 50_000,
            eph: 120,
            epv: 160,
            vel: 500,
            cog: 9000,
            satellites_visible: 12,
            ..Default::default()
        });
        let h = MavHeader { system_id: 255, component_id: 190, sequence: 0 };
        let bytes = write_v2_msg(&h, &msg).unwrap();
        let (_, rmsg, _) = read_v2_msg(&bytes).unwrap();
        match rmsg {
            MavMessage::GPS_RAW_INT(d) => {
                assert_eq!(d.lat, 31_230_000);
                assert_eq!(d.lon, 121_470_000);
                assert_eq!(d.fix_type, GpsFixType::GPS_FIX_TYPE_3D_FIX);
                assert_eq!(d.satellites_visible, 12);
            }
            other => panic!("wrong msg: {other:?}"),
        }
    }

    #[test]
    fn stream_parse_partial_then_full() {
        let msg = MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: 1,
            roll: 0.1,
            pitch: -0.2,
            yaw: 1.5,
            rollspeed: 0.01,
            pitchspeed: 0.02,
            yawspeed: 0.03,
        });
        let h = MavHeader { system_id: 1, component_id: 1, sequence: 0 };
        let bytes = write_v2_msg(&h, &msg).unwrap();
        // 分两次投喂：先给不完整前缀，再补剩余字节，验证流式解析可跨缓冲续接。
        let cut = bytes.len() / 2;
        assert!(read_v2_msg(&bytes[..cut]).is_none());
        let (rh, rmsg, used) = read_v2_msg(&bytes).unwrap();
        assert_eq!(used, bytes.len());
        assert_eq!(rh.system_id, 1);
        match rmsg {
            MavMessage::ATTITUDE(d) => {
                assert_eq!(d.roll, 0.1);
                assert_eq!(d.yaw, 1.5);
            }
            other => panic!("wrong msg: {other:?}"),
        }
    }

    #[test]
    fn unknown_msgid_skipped() {
        // 构造一个本实现未知的 msgid（CRC_EXTRA 表中为 0 也会被解码层拒绝）。
        let mut buf = vec![0xFD, 1, 0, 0, 0, 1, 1, 0xFF, 0, 0, 0xAB, 0xCD, 0x00, 0x00];
        // 追加一个合法 HEARTBEAT 帧，验证解码器能跳过垃圾继续找到有效帧。
        let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        let h = MavHeader { system_id: 255, component_id: 190, sequence: 0 };
        buf.extend_from_slice(&write_v2_msg(&h, &hb).unwrap());
        let (_, rmsg, _) = read_v2_msg(&buf).unwrap();
        assert!(matches!(rmsg, MavMessage::HEARTBEAT(_)));
    }
}
