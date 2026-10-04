//! 具体消息编解码（函数式 API，直接写固定缓冲、零堆）。
//!
//! 覆盖 QGC/groundctrl 常用消息：HEARTBEAT、SYS_STATUS、ATTITUDE、LOCAL_POSITION_NED、
//! GLOBAL_POSITION_INT、VFR_HUD、COMMAND_LONG/ACK、PARAM_*、AUTOPILOT_VERSION、
//! MISSION_*、RC_CHANNELS_OVERRIDE、FENCE_*、REQUEST_DATA_STREAM/DATA_STREAM。
//!
//! 与 `flyctrl-core` 旧版区别：本层**不依赖 VehicleState**，凡是从状态对象取字段的
//! 编码器都改为"原始标量签名"（`*_raw`），由调用方负责换算（如 NED→经纬度、姿态→欧拉角）。
//! 这样本 crate 才能被固件/仿真器/地面站/App 无差别复用。

use crate::frame::*;

// ── 心跳（HEARTBEAT） ────────────────────────────────────────────

/// HEARTBEAT：声明飞控存活 + 当前模式（标准字段，QGC 可识别）。
/// `mode` 为自定义飞行模式自定义码（与 `flightmode::FlightMode` 映射），`armed` 反映解锁态。
pub fn encode_heartbeat(mode: u8, armed: bool, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    // 默认 autopilot 用 ARDUPILOT，使地面站（QGC/groundctrl）按 ArduCopter 自定义模式解码模式名。
    encode_heartbeat_ap(mode, enums::MAV_AUTOPILOT_ARDUPILOTMEGA, armed, enums::MAV_STATE_ACTIVE, enums::MAV_STATE_STANDBY, false, seq, out)
}

/// HIL 心跳：base_mode 额外置 `MAV_MODE_FLAG_HIL_ENABLED`(0x20)，
/// 使地面站/QGC 识别「当前为硬件在环仿真模式」，避免把仿真当真飞。
pub fn encode_heartbeat_hil(mode: u8, armed: bool, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    encode_heartbeat_ap(mode, enums::MAV_AUTOPILOT_ARDUPILOTMEGA, armed, enums::MAV_STATE_ACTIVE, enums::MAV_STATE_STANDBY, true, seq, out)
}

/// 完整版心跳：允许指定 autopilot 与 system_status（激活/待命）。
/// `hil` 为 true 时 base_mode 置 `MAV_MODE_FLAG_HIL_ENABLED`(0x20)。
pub fn encode_heartbeat_ap(
    mode: u8,
    autopilot: u8,
    armed: bool,
    state_active: u8,
    state_standby: u8,
    hil: bool,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    use enums::*;
    let mut payload = [0u8; 9];
    payload[0] = MAV_TYPE_QUADROTOR; // type
    payload[1] = autopilot;          // autopilot
    payload[2] = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | if hil { MAV_MODE_FLAG_HIL_ENABLED } else { 0 }
        | if armed { MAV_MODE_FLAG_SAFETY_ARMED } else { 0 };
    // custom_mode 是 uint32（标准 common.xml），占 [3..7]。
    payload[3..7].copy_from_slice(&(mode as u32).to_le_bytes());
    payload[7] = if armed { state_active } else { state_standby }; // system_status
    payload[8] = 3; // mavlink_version (v3)
    encode(msg_id::HEARTBEAT, seq, &payload, out)
}

// ── 指令（COMMAND_LONG / COMMAND_ACK） ───────────────────────────

/// COMMAND_LONG：地面站下发的通用指令（含 SET_MODE / 解锁 / 起降 / RTL）。
/// `command` 见 [`enums::MAV_CMD_*`]；`p1..p7` 为 7 个 f32 参数，`confirmation` 为确认计数。
pub fn encode_command_long(
    command: u16,
    p1: f32, p2: f32, p3: f32, p4: f32, p5: f32, p6: f32, p7: f32,
    confirmation: u8,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 33];
    // target_system, target_component, command(u16), confirmation, param1-7 (f32)
    p[0] = 0; // broadcast target_system
    p[1] = 0; // broadcast target_component
    p[2..4].copy_from_slice(&command.to_le_bytes());
    p[4] = confirmation;
    put_f32(&mut p, 5, p1);
    put_f32(&mut p, 9, p2);
    put_f32(&mut p, 13, p3);
    put_f32(&mut p, 17, p4);
    put_f32(&mut p, 21, p5);
    put_f32(&mut p, 25, p6);
    put_f32(&mut p, 29, p7);
    encode(msg_id::COMMAND_LONG, seq, &p, out)
}

/// COMMAND_LONG 解码结果（地面站 -> 飞控）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandLong {
    pub target_system: u8,
    pub target_component: u8,
    pub command: u16,
    pub confirmation: u8,
    pub params: [f32; 7],
}

/// 从 payload 解析 COMMAND_LONG（需先经 [`decode`] 取出 payload）。
pub fn decode_command_long(payload: &[u8]) -> Option<CommandLong> {
    if payload.len() < 33 {
        return None;
    }
    let command = u16::from_le_bytes([payload[2], payload[3]]);
    let rd = |o: usize| f32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    Some(CommandLong {
        target_system: payload[0],
        target_component: payload[1],
        command,
        confirmation: payload[4],
        params: [rd(5), rd(9), rd(13), rd(17), rd(21), rd(25), rd(29)],
    })
}

/// 把 f32 参数打包进 COMMAND_LONG 的便利函数（用于板端构造应答/测试）。
pub fn command_long_params(p: [f32; 7]) -> [f32; 7] {
    p
}

/// COMMAND_ACK：飞控对地面站指令的应答（确认/拒绝）。
/// `command` 为被应答的 MAV_CMD；`result` 见 [`enums::MAV_RESULT_*`]；
/// `progress` 为完成进度(0-100)，`result_param2` 为附加结果。
pub fn encode_command_ack(
    command: u16,
    result: u8,
    progress: u8,
    result_param2: i32,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 11];
    p[0..2].copy_from_slice(&command.to_le_bytes());
    p[2] = result;
    p[3] = progress;
    p[4..8].copy_from_slice(&result_param2.to_le_bytes());
    // target_system, target_component（应答回地面站）
    p[8] = 0; // broadcast
    p[9] = 0; // broadcast
    p[10] = 0; // mavlink_version reserved
    encode(msg_id::COMMAND_ACK, seq, &p, out)
}

/// COMMAND_ACK 解码（地面站 -> 飞控，用于请求重发/确认链路）。
pub fn decode_command_ack(payload: &[u8]) -> Option<(u16, u8)> {
    if payload.len() < 3 {
        return None;
    }
    let command = u16::from_le_bytes([payload[0], payload[1]]);
    Some((command, payload[2]))
}

// ── 参数（PARAM_*） ──────────────────────────────────────────────

/// PARAM_VALUE：飞控向地面站回传单个参数（QGC 参数表读取）。
/// `id` 为 16 字节 NUL 结尾参数名；`value` 为 f32；`param_type` 见 [`enums::MAV_PARAM_TYPE_REAL32`]；
/// `param_count`/`param_index` 为参数表总数/当前索引。
pub fn encode_param_value(
    id: &[u8; 16],
    value: f32,
    param_type: u8,
    param_count: u16,
    param_index: u16,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 25];
    p[0..16].copy_from_slice(id);
    put_f32(&mut p, 16, value);
    p[20] = param_type;
    p[21..23].copy_from_slice(&param_count.to_le_bytes());
    p[23..25].copy_from_slice(&param_index.to_le_bytes());
    encode(msg_id::PARAM_VALUE, seq, &p, out)
}

/// PARAM_VALUE 解码（地面站 -> 飞控，用于参数写入回执校验）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamValue {
    pub id: [u8; 16],
    pub value: f32,
    pub param_type: u8,
    pub param_count: u16,
    pub param_index: u16,
}

pub fn decode_param_value(payload: &[u8]) -> Option<ParamValue> {
    if payload.len() < 25 {
        return None;
    }
    let mut id = [0u8; 16];
    id.copy_from_slice(&payload[0..16]);
    Some(ParamValue {
        id,
        value: f32::from_le_bytes(payload[16..20].try_into().unwrap()),
        param_type: payload[20],
        param_count: u16::from_le_bytes([payload[21], payload[22]]),
        param_index: u16::from_le_bytes([payload[23], payload[24]]),
    })
}

/// PARAM_REQUEST_LIST 解码（地面站 -> 飞控，请求参数表全量流水）。
/// 该消息无 payload 字段（仅头部），只需识别 msg_id 即可。
pub fn decode_param_request_list(_payload: &[u8]) -> Option<()> {
    Some(())
}

/// PARAM_REQUEST_LIST：飞控向地面站广播"开始参数表流水"。payload 为 `target_system(1) + target_component(1)`。
pub fn encode_param_request_list(
    target_system: u8,
    target_component: u8,
    _req_comp_id: u8,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 2];
    p[0] = target_system;
    p[1] = target_component;
    encode(msg_id::PARAM_REQUEST_LIST, seq, &p, out)
}

/// PARAM_SET 解码（地面站 -> 飞控，写入单个参数）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamSet {
    pub id: [u8; 16],
    pub value: f32,
    pub param_type: u8,
}

pub fn decode_param_set(payload: &[u8]) -> Option<ParamSet> {
    if payload.len() < 21 {
        return None;
    }
    let mut id = [0u8; 16];
    id.copy_from_slice(&payload[0..16]);
    Some(ParamSet {
        id,
        value: f32::from_le_bytes(payload[16..20].try_into().unwrap()),
        param_type: payload[20],
    })
}

/// PARAM_REQUEST_READ 解码（地面站 -> 飞控，按参数名点读单个参数）。
/// payload：`param_id[16] + param_index i16`（index=-1 表示按名查找）。
pub fn decode_param_request_read(payload: &[u8]) -> Option<([u8; 16], i16)> {
    if payload.len() < 18 {
        return None;
    }
    let mut id = [0u8; 16];
    id.copy_from_slice(&payload[0..16]);
    let idx = i16::from_le_bytes([payload[16], payload[17]]);
    Some((id, idx))
}

/// AUTOPILOT_VERSION：飞控向地面站上报固件/能力信息（响应 REQUEST_AUTOPILOT_CAPABILITIES）。
/// 标准布局（60B），本实现填最小有效子集 + 能力位（MAV_PROTOCOL_CAPABILITY_MAVLINK2 + PARAM_FLOAT）。
pub fn encode_autopilot_version(capabilities: u64, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 60];
    // flight_sw_version = 1.0.0 (0x010000)
    p[0..4].copy_from_slice(&0x0100_0000u32.to_le_bytes());
    // board_version = 0x407 (STM32F407)
    p[12..16].copy_from_slice(&0x0407u32.to_le_bytes());
    // vendor_id / product_id
    p[48..50].copy_from_slice(&0x4A4F_u16.to_le_bytes()); // "JO"
    p[50..52].copy_from_slice(&0x4352_u16.to_le_bytes()); // "CR"
    // capabilities u64 @ offset 52
    p[52..60].copy_from_slice(&capabilities.to_le_bytes());
    encode(msg_id::AUTOPILOT_VERSION, seq, &p, out)
}

// ── 遥测（ATTITUDE / LOCAL_POSITION_NED / SYS_STATUS / VFR_HUD / GLOBAL_POSITION_INT） ──

/// ATTITUDE：姿态欧拉角 + 角速度（rad/s）。标准布局（28B）。
/// 入参为已换算好的标量（roll/pitch/yaw + p/q/r），不感知 VehicleState。
pub fn encode_attitude_raw(
    time_boot_ms: i32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    p: f32,
    q: f32,
    r: f32,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut pl = [0u8; 28];
    put_i32(&mut pl, 0, time_boot_ms);
    put_f32(&mut pl, 4, roll);
    put_f32(&mut pl, 8, pitch);
    put_f32(&mut pl, 12, yaw);
    put_f32(&mut pl, 16, p);
    put_f32(&mut pl, 20, q);
    put_f32(&mut pl, 24, r);
    encode(msg_id::ATTITUDE, seq, &pl, out)
}

/// ★design.md §9：**NAMED_VALUE_FLOAT**（id 251）—— 一条「名称 + 浮点值」诊断对。
/// 标准布局（18B，按 MAVLink 字段排序：u32 time_boot_ms, f32 value, char[10] name）。
/// 用途：把 L1/L2/L3 的可观测量（执行时间、周期抖动、队列利用、CPU 负载…）以**地面站
/// 或 NSH 可直接查看**的形式下发 ✓（§9「可通过 NSH 命令或 MAVLink 实时查看」✓）。
pub fn encode_named_value_float_raw(
    time_boot_ms: u32,
    value: f32,
    name: &[u8],
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut pl = [0u8; 18];
    put_i32(&mut pl, 0, time_boot_ms as i32); // 位布局等同 u32 ✓（MAVLink 字段类型只影响 CRC ✓）
    put_f32(&mut pl, 4, value);
    let n = name.len().min(10);
    pl[8..8 + n].copy_from_slice(&name[..n]);
    encode(msg_id::NAMED_VALUE_FLOAT, seq, &pl, out)
}

/// LOCAL_POSITION_NED：NED 位置 + 速度（m, m/s）。标准布局（28B）。
pub fn encode_local_pos_raw(
    time_boot_ms: i32,
    x: f32,
    y: f32,
    z: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    encode_local_pos_from_raw(SYS_ID, time_boot_ms, x, y, z, vx, vy, vz, seq, out)
}

/// 同上，但允许指定 `sys_id`（多机协同：每架飞机用各自 sys_id 广播自身状态）。
/// 内部 helper：构造完整帧（含标准 CRC_EXTRA）后覆盖 sys_id 并重算头部 CRC。
pub fn encode_local_pos_from_raw(
    sys_id: u8,
    time_boot_ms: i32,
    x: f32,
    y: f32,
    z: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 28];
    put_i32(&mut p, 0, time_boot_ms);
    put_f32(&mut p, 4, x);
    put_f32(&mut p, 8, y);
    put_f32(&mut p, 12, z);
    put_f32(&mut p, 16, vx);
    put_f32(&mut p, 20, vy);
    put_f32(&mut p, 24, vz);
    // 直接构造到 out（避免中转栈缓冲）。v2 头部 9 字节。
    out[0] = MAVLINK_MAGIC;
    out[1] = p.len() as u8;
    out[2] = 0; // incompat_flags
    out[3] = 0; // compat_flags
    out[4] = seq;
    out[5] = sys_id; // 自定义 sys_id
    out[6] = COMP_ID;
    // msgid 以小端写入 3 字节（v2 扩展消息 ID）。
    out[7] = msg_id::LOCAL_POSITION_NED as u8;
    out[8] = (msg_id::LOCAL_POSITION_NED >> 8) as u8;
    out[9] = (msg_id::LOCAL_POSITION_NED >> 16) as u8;
    out[10..10 + p.len()].copy_from_slice(&p[..]);
    // 标准 MAVLink v2 CRC（含 CRC_EXTRA），与 encode() 同算法。
    let mut crc = crc16_x25(0xFFFF, &out[1..10 + p.len()]);
    crc = crc16_x25(crc, &[CRC_EXTRA[msg_id::LOCAL_POSITION_NED as usize]]);
    out[10 + p.len()] = (crc & 0xFF) as u8;
    out[10 + p.len() + 1] = (crc >> 8) as u8;
    10 + p.len() + 2
}

/// ATTITUDE(30) 解码结果（飞控 -> 仿真器/地面站：姿态欧拉角 + 角速度）。标准布局（28B）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    pub time_boot_ms: i32,
    pub roll: f32,  // rad
    pub pitch: f32, // rad
    pub yaw: f32,   // rad
    pub rollspeed: f32,  // rad/s (p)
    pub pitchspeed: f32, // rad/s (q)
    pub yawspeed: f32,   // rad/s (r)
}

/// 从 payload 解析 ATTITUDE（需先经 [`decode`] 取出 payload）。
pub fn decode_attitude(payload: &[u8]) -> Option<Attitude> {
    if payload.len() < 28 {
        return None;
    }
    let rd = |o: usize| f32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    Some(Attitude {
        time_boot_ms: i32::from_le_bytes(payload[0..4].try_into().unwrap()),
        roll: rd(4),
        pitch: rd(8),
        yaw: rd(12),
        rollspeed: rd(16),
        pitchspeed: rd(20),
        yawspeed: rd(24),
    })
}

/// LOCAL_POSITION_NED(32) 解码结果（飞控 -> 仿真器/地面站：NED 位置 + 速度）。标准布局（28B）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalPositionNed {
    pub time_boot_ms: i32,
    pub x: f32, // NED 北（m）
    pub y: f32, // NED 东（m）
    pub z: f32, // NED 下为正（m）
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
}

/// 从 payload 解析 LOCAL_POSITION_NED（需先经 [`decode`] 取出 payload）。
pub fn decode_local_position_ned(payload: &[u8]) -> Option<LocalPositionNed> {
    if payload.len() < 28 {
        return None;
    }
    let rd = |o: usize| f32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    Some(LocalPositionNed {
        time_boot_ms: i32::from_le_bytes(payload[0..4].try_into().unwrap()),
        x: rd(4),
        y: rd(8),
        z: rd(12),
        vx: rd(16),
        vy: rd(20),
        vz: rd(24),
    })
}

/// SYS_STATUS：健康位（取 FDIR 健康；此处仅填传感器位）。标准布局（31B）。
pub fn encode_sys_status(sensors_ok: bool, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 31];
    let sensor_bits: i32 = if sensors_ok { 0x1F } else { 0 };
    put_i32(&mut p, 0, sensor_bits); // onboard_control_sensors_present
    put_i32(&mut p, 4, sensor_bits); // enabled
    put_i32(&mut p, 8, sensor_bits); // health
    // load (u16), voltage_battery (i16 mV/1000), current_battery (i16 cA), battery_remaining (i8 %)
    put_u16(&mut p, 12, 100); // 10% CPU load placeholder
    put_i16(&mut p, 14, 12000); // 12.0V battery (mV/1000)
    put_i16(&mut p, 16, 0); // 0 cA current
    p[18] = 80; // 80% remaining
    encode(msg_id::SYS_STATUS, seq, &p, out)
}

/// VFR_HUD：空速/地速/高度/航向/油门（标准 HUD 主盘字段）。标准布局（20B）。
/// 入参为已换算好的标量：`groundspeed`（m/s）、`heading_cdeg`（厘度）、`throttle_pct`（0..100）、
/// `alt`（相对高度 m）、`climb`（爬升率 m/s）。平方根等数值计算由调用方完成（保持本层无浮点依赖）。
pub fn encode_vfr_hud_raw(
    groundspeed: f32,
    heading_cdeg: i16,
    throttle_pct: u16,
    alt: f32,
    climb: f32,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 20];
    put_f32(&mut p, 0, 0.0); // airspeed
    put_f32(&mut p, 4, groundspeed); // groundspeed
    put_i16(&mut p, 8, heading_cdeg); // heading (centidegrees)
    put_u16(&mut p, 10, throttle_pct); // throttle (%)
    put_f32(&mut p, 12, alt); // alt (relative)
    put_f32(&mut p, 16, climb); // climb rate
    encode(msg_id::VFR_HUD, seq, &p, out)
}

/// GLOBAL_POSITION_INT：GPS 全局位置（lat/lon 为 1e7 整数度）。标准布局（28B）。
/// 入参为已换算好的标量：`alt_rel_mm`（相对高度 mm）、`vx/vy/vz_cm`（cm/s）、`heading_cdeg`。
pub fn encode_global_position_int_raw(
    time_boot_ms: i32,
    alt_rel_mm: i32,
    vx_cm: i16,
    vy_cm: i16,
    vz_cm: i16,
    heading_cdeg: u16,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 28];
    put_i32(&mut p, 0, time_boot_ms);
    put_i32(&mut p, 4, 0); // lat (no GPS)
    put_i32(&mut p, 8, 0); // lon
    put_i32(&mut p, 12, alt_rel_mm); // relative alt mm
    put_i32(&mut p, 16, alt_rel_mm); // alt_amsl mm (no GPS datum)
    put_i16(&mut p, 20, vx_cm);
    put_i16(&mut p, 22, vy_cm);
    put_i16(&mut p, 24, vz_cm);
    put_u16(&mut p, 26, heading_cdeg); // heading cdeg
    encode(msg_id::GLOBAL_POSITION_INT, seq, &p, out)
}

// ── 航点（MISSION）系列编解码 ─────────────────────────────────────

/// 单条航点（与标准 MAVLink `MISSION_ITEM_INT` 字段对齐）。
/// 坐标 lat/lon 为 1e7 整数度，alt 为米（f32）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionItem {
    pub target_system: u8,
    pub target_component: u8,
    pub seq: u16,
    pub command: u16,
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub x: i32, // lat * 1e7
    pub y: i32, // lon * 1e7
    pub z: f32, // alt (m)
    pub frame: u8,
    pub current: u8,
    pub autocontinue: u8,
    pub mission_type: u8, // MAV_MISSION_TYPE（如 0=ALL, 1=PLAN, 2=FENCE...），MISSION_ITEM_INT 末端 1B
}

/// MISSION_ITEM_INT 解码（地面站 -> 飞控，上传的单条航点）。标准布局（38B）。
pub fn decode_mission_item_int(payload: &[u8]) -> Option<MissionItem> {
    if payload.len() < 38 {
        return None;
    }
    let rd = |o: usize| f32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    Some(MissionItem {
        target_system: payload[0],
        target_component: payload[1],
        seq: u16::from_le_bytes([payload[2], payload[3]]),
        frame: payload[4],
        command: u16::from_le_bytes([payload[5], payload[6]]),
        current: payload[7],
        autocontinue: payload[8],
        param1: rd(9),
        param2: rd(13),
        param3: rd(17),
        param4: rd(21),
        x: i32::from_le_bytes([payload[25], payload[26], payload[27], payload[28]]),
        y: i32::from_le_bytes([payload[29], payload[30], payload[31], payload[32]]),
        z: rd(33),
        mission_type: payload[37],
    })
}

/// MISSION_REQUEST 解码（地面站 -> 飞控，请求某条航点）。标准布局（4B）。
pub fn decode_mission_request(payload: &[u8]) -> Option<u16> {
    if payload.len() < 4 {
        return None;
    }
    Some(u16::from_le_bytes([payload[2], payload[3]]))
}

/// MISSION_COUNT 解码（地面站 -> 飞控，宣布上传航点总数）。标准布局（4B）。
pub fn decode_mission_count(payload: &[u8]) -> Option<u16> {
    if payload.len() < 4 {
        return None;
    }
    Some(u16::from_le_bytes([payload[2], payload[3]]))
}

/// MISSION_REQUEST_LIST 解码（地面站 -> 飞控，请求下载全部航点）。标准布局（2B）。
pub fn decode_mission_request_list(_payload: &[u8]) -> Option<()> {
    Some(())
}

/// MISSION_REQUEST 编码（飞控 -> 地面站，请求某条航点）。
pub fn encode_mission_request(seq: u16, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 4];
    p[2..4].copy_from_slice(&seq.to_le_bytes());
    encode(msg_id::MISSION_REQUEST, 0, &p, out)
}

/// MISSION_COUNT 编码（飞控 -> 地面站，宣布航点总数）。
pub fn encode_mission_count(count: u16, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 4];
    p[2..4].copy_from_slice(&count.to_le_bytes());
    encode(msg_id::MISSION_COUNT, 0, &p, out)
}

/// MISSION_ITEM_INT 编码（飞控 -> 地面站，下载的单条航点）。
pub fn encode_mission_item_int(item: &MissionItem, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 38];
    p[0] = item.target_system;
    p[1] = item.target_component;
    p[2..4].copy_from_slice(&item.seq.to_le_bytes());
    p[4] = item.frame;
    p[5..7].copy_from_slice(&item.command.to_le_bytes());
    p[7] = item.current;
    p[8] = item.autocontinue;
    put_f32(&mut p, 9, item.param1);
    put_f32(&mut p, 13, item.param2);
    put_f32(&mut p, 17, item.param3);
    put_f32(&mut p, 21, item.param4);
    p[25..29].copy_from_slice(&item.x.to_le_bytes());
    p[29..33].copy_from_slice(&item.y.to_le_bytes());
    put_f32(&mut p, 33, item.z);
    p[37] = item.mission_type;
    encode(msg_id::MISSION_ITEM_INT, 0, &p, out)
}

/// MISSION_ACK 编码（飞控 -> 地面站，握手结束确认）。
/// `ack_type`：0=ACCEPTED, 1=ERROR, 4=NO_SPACE, 5=INVALID_SEQUENCE 等（MAV_MISSION_RESULT）。
pub fn encode_mission_ack(ack_type: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 4];
    p[2] = ack_type; // type
    p[3] = 0; // mission_type (ALL=0)
    encode(msg_id::MISSION_ACK, 0, &p, out)
}

// ── RC 通道覆盖 ──────────────────────────────────────────────────

/// RC_CHANNELS_OVERRIDE 解码（地面站 -> 飞控，手动操控通道）。标准布局（21B）。
/// 通道为 PWM 微秒值（典型 1000-2000），板子侧需归一化到 0.0-1.0。
pub fn decode_rc_channels_override(payload: &[u8]) -> Option<[u16; 8]> {
    if payload.len() < 21 {
        return None;
    }
    let mut ch = [0u16; 8];
    for i in 0..8 {
        ch[i] = u16::from_le_bytes([payload[2 + i * 2], payload[3 + i * 2]]);
    }
    Some(ch)
}

// ── 围栏（FENCE）系列编解码 ─────────────────────────────────────

/// 单条围栏顶点（与标准 MAVLink `FENCE_POINT` 字段对齐）。坐标 lat/lon 为 1e7 整数度。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FencePoint {
    pub target_system: u8,
    pub target_component: u8,
    pub idx: u8,
    pub count: u8,
    pub lat: i32, // * 1e7
    pub lon: i32, // * 1e7
}

/// FENCE_POINT 解码（地面站 -> 飞控，上传的单条围栏顶点）。标准布局（12B）。
pub fn decode_fence_point(payload: &[u8]) -> Option<FencePoint> {
    if payload.len() < 12 {
        return None;
    }
    Some(FencePoint {
        target_system: payload[0],
        target_component: payload[1],
        idx: payload[2],
        count: payload[3],
        lat: i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
        lon: i32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
    })
}

/// FENCE_FETCH_POINT 解码（地面站 -> 飞控，请求下载某条围栏顶点）。标准布局（3B）。
pub fn decode_fence_fetch_point(payload: &[u8]) -> Option<u8> {
    if payload.len() < 3 {
        return None;
    }
    Some(payload[2])
}

/// FENCE_POINT 编码（飞控 -> 地面站，下载的单条围栏顶点）。
pub fn encode_fence_point(pt: &FencePoint, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 12];
    p[0] = pt.target_system;
    p[1] = pt.target_component;
    p[2] = pt.idx;
    p[3] = pt.count;
    p[4..8].copy_from_slice(&pt.lat.to_le_bytes());
    p[8..12].copy_from_slice(&pt.lon.to_le_bytes());
    encode(msg_id::FENCE_POINT, 0, &p, out)
}

// ── 数据流速率控制（REQUEST_DATA_STREAM / DATA_STREAM） ─────────────

/// REQUEST_DATA_STREAM 解码（地面站 -> 飞控）。标准布局（6B）。
pub fn decode_request_data_stream(payload: &[u8]) -> Option<(u8, u16, u8)> {
    if payload.len() < 6 {
        return None;
    }
    let stream_id = payload[0];
    let rate_hz = u16::from_le_bytes([payload[1], payload[2]]);
    let start_stop = payload[5];
    Some((stream_id, rate_hz, start_stop))
}

/// DATA_STREAM 编码（飞控 -> 地面站，对 REQUEST_DATA_STREAM 的应答）。标准布局（7B）。
pub fn encode_data_stream(stream_id: u8, rate_hz: u16, on_off: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 8];
    p[0] = stream_id;
    p[1..3].copy_from_slice(&rate_hz.to_le_bytes());
    p[3] = 1; // target_system
    p[4] = 1; // target_component
    p[5..7].copy_from_slice(&0u16.to_le_bytes()); // messages_sent（累计发送数，未知填 0）
    p[7] = on_off; // 0=off 1=on
    encode(msg_id::DATA_STREAM, 0, &p, out)
}

// ── HIL（硬件在环，仿真器 -> 飞控） ───────────────────────────────

/// HIL_SENSOR 解码（仿真器 -> 飞控，传感器真值）。标准布局（HIL_SENSOR(107)，62B）：
/// time_usec(8) + accel/gyro/mag 各 3*f32(36) + abs/diff/alt 3*f32(12) + temp(i16,2) + fields_updated(u32,4)。
/// 返回 (time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag,
///         abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated)。
#[allow(clippy::too_many_arguments)]
pub fn decode_hil_sensor(payload: &[u8]) -> Option<(u64, f32, f32, f32, f32, f32, f32, f32, f32, f32, f32, f32, f32, i16, u32)> {
    if payload.len() < 62 {
        return None;
    }
    let rd_f = |o: usize| f32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    let rd_u64 = |o: usize| {
        u64::from_le_bytes(payload[o..o + 8].try_into().unwrap())
    };
    Some((
        rd_u64(0),  // time_usec
        rd_f(8),    // xacc
        rd_f(12),   // yacc
        rd_f(16),   // zacc
        rd_f(20),   // xgyro
        rd_f(24),   // ygyro
        rd_f(28),   // zgyro
        rd_f(32),   // xmag
        rd_f(36),   // ymag
        rd_f(40),   // zmag
        rd_f(44),   // abs_pressure
        rd_f(48),   // diff_pressure
        rd_f(52),   // pressure_alt
        i16::from_le_bytes([payload[56], payload[57]]), // temperature
        u32::from_le_bytes([payload[58], payload[59], payload[60], payload[61]]), // fields_updated
    ))
}

/// 编码 HIL_SENSOR(107)（仿真器 -> 飞控：传感器真值）。标准布局（62B），CRC_EXTRA=108。
/// 字段顺序与 [`decode_hil_sensor`] 严格一致：time_usec + accel/gyro/mag 各 3*f32 +
/// abs/diff/alt 3*f32 + temp(i16) + fields_updated(u32)。fields_updated 为 0 表示全部字段有效。
#[allow(clippy::too_many_arguments)]
pub fn encode_hil_sensor(
    time_usec: u64,
    xacc: f32, yacc: f32, zacc: f32,
    xgyro: f32, ygyro: f32, zgyro: f32,
    xmag: f32, ymag: f32, zmag: f32,
    abs_pressure: f32,
    diff_pressure: f32,
    pressure_alt: f32,
    temperature: i16,
    fields_updated: u32,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 62];
    p[0..8].copy_from_slice(&time_usec.to_le_bytes());
    put_f32(&mut p, 8, xacc);
    put_f32(&mut p, 12, yacc);
    put_f32(&mut p, 16, zacc);
    put_f32(&mut p, 20, xgyro);
    put_f32(&mut p, 24, ygyro);
    put_f32(&mut p, 28, zgyro);
    put_f32(&mut p, 32, xmag);
    put_f32(&mut p, 36, ymag);
    put_f32(&mut p, 40, zmag);
    put_f32(&mut p, 44, abs_pressure);
    put_f32(&mut p, 48, diff_pressure);
    put_f32(&mut p, 52, pressure_alt);
    p[56..58].copy_from_slice(&temperature.to_le_bytes());
    p[58..62].copy_from_slice(&fields_updated.to_le_bytes());
    encode(msg_id::HIL_SENSOR, seq, &p, out)
}

/// HIL_GPS 解码（仿真器 -> 飞控，GPS 真值）。标准布局（HIL_GPS(113)，36B）：
/// time_usec(8) + fix_type(1) + lat/lon/alt 3*i32(12) + eph/epv 2*u16(4) + vel(u16,2)
/// + vn/ve/vd 3*i16(6) + cog(u16,2) + satellites_visible(1)。
///
/// 本闭环 HIL 把 lat/lon/alt/vn/ve/vd 重解释为 **NED 局部系位置/速度真值**（与
/// SET_POSITION 的 NED 约定一致）：lat=x·1e7、lon=y·1e7、alt=z·1e3(mm)、vn/ve/vd=v·1e2(cm/s)。
/// 飞控侧解码后按此缩放还原（见 uplink.rs HIL_GPS 分支）。
#[allow(clippy::too_many_arguments)]
pub fn decode_hil_gps(
    payload: &[u8],
) -> Option<(u64, u8, i32, i32, i32, u16, u16, u16, i16, i16, i16, u16, u8)> {
    if payload.len() < 36 {
        return None;
    }
    let rd = |o: usize| u32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    Some((
        u64::from_le_bytes(payload[0..8].try_into().unwrap()), // time_usec
        payload[8],                                            // fix_type
        rd(9) as i32,                                          // lat（重解释：NED x·1e7）
        rd(13) as i32,                                         // lon（重解释：NED y·1e7）
        rd(17) as i32,                                         // alt（重解释：NED z·1e3，mm）
        u16::from_le_bytes([payload[21], payload[22]]),        // eph
        u16::from_le_bytes([payload[23], payload[24]]),        // epv
        u16::from_le_bytes([payload[25], payload[26]]),        // vel（幅值，cm/s）
        i16::from_le_bytes([payload[27], payload[28]]),        // vn（NED vx·1e2）
        i16::from_le_bytes([payload[29], payload[30]]),        // ve（NED vy·1e2）
        i16::from_le_bytes([payload[31], payload[32]]),        // vd（NED vz·1e2）
        u16::from_le_bytes([payload[33], payload[34]]),        // cog
        payload[35],                                           // satellites_visible
    ))
}

/// 编码 HIL_GPS(113)（仿真器 -> 飞控：GPS 位置/速度真值）。标准布局（36B），CRC_EXTRA=55。
/// 字段与 [`decode_hil_gps`] 一致：NED 位置/速度（`pos`/`vel`，米制）经缩放装入
/// lat/lon/alt/vn/ve/vd 传输；其余字段（fix_type=3D 定位、eph/epv/vel/cog 等）按悬停
/// 真值填零。lat/lon 以 1e7 缩放（i32 范围 ±214m 内均无溢出），alt 以 mm、速度以 cm/s。
#[allow(clippy::too_many_arguments)]
pub fn encode_hil_gps(
    time_usec: u64,
    pos: [f32; 3],
    vel: [f32; 3],
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 36];
    p[0..8].copy_from_slice(&time_usec.to_le_bytes());
    p[8] = 3; // fix_type=3D fix（GPS 定位有效）
    p[9..13].copy_from_slice(&((pos[0] * 1e7) as i32).to_le_bytes());   // lat ← NED x
    p[13..17].copy_from_slice(&((pos[1] * 1e7) as i32).to_le_bytes());  // lon ← NED y
    p[17..21].copy_from_slice(&((pos[2] * 1e3) as i32).to_le_bytes());  // alt ← NED z(mm)
    // eph/epv 填 0（真值源无误差模型）
    p[25..27].copy_from_slice(&(0u16).to_le_bytes());                    // vel 幅值
    p[27..29].copy_from_slice(&((vel[0] * 1e2) as i16).to_le_bytes());   // vn ← vx
    p[29..31].copy_from_slice(&((vel[1] * 1e2) as i16).to_le_bytes());   // ve ← vy
    p[31..33].copy_from_slice(&((vel[2] * 1e2) as i16).to_le_bytes());   // vd ← vz
    // cog 填 0
    p[35] = 12; // satellites_visible（真值源恒“可见”）
    encode(msg_id::HIL_GPS, seq, &p, out)
}

/// HIL_RC_INPUTS_RAW 解码（仿真器 -> 飞控，RC 通道真值）。标准布局（HIL_RC_INPUTS_RAW(92)）。
/// 返回 (chan_raw[16], rssi, rc_fail)。
pub fn decode_hil_rc_inputs_raw(payload: &[u8]) -> Option<([u16; 16], u8, u8)> {
    if payload.len() < 37 {
        return None;
    }
    let mut ch = [0u16; 16];
    for i in 0..16 {
        ch[i] = u16::from_le_bytes([payload[i * 2], payload[i * 2 + 1]]);
    }
    Some((ch, payload[33], payload[34])) // rssi @33, rc_fail @34 (其余忽略)
}

/// SET_POSITION_TARGET_LOCAL_NED(84) 解码结果（仿真器 -> 飞控：设定点 + NED 位置真值）。
/// HIL 场景下 PC 用该消息同时下发「期望状态」与「机体位置真值」。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetPositionTargetLocalNed {
    pub time_boot_ms: u32,
    pub type_mask: u16, // POSITION_TARGET_TYPEMASK（位 0=忽略位置，位 1=忽略速度，…）
    pub x: f32,         // 位置 NED X（北，m）
    pub y: f32,         // 位置 NED Y（东，m）
    pub z: f32,         // 位置 NED Z（向下为正，m）
    pub vx: f32,        // 速度 NED X（m/s）
    pub vy: f32,
    pub vz: f32,
    pub afx: f32,       // 加速度 NED X（m/s²）
    pub afy: f32,
    pub afz: f32,
    pub yaw: f32,       // 偏航（rad）
    pub yaw_rate: f32,  // 偏航角速度（rad/s）
}

/// 编码 SET_POSITION_TARGET_LOCAL_NED(84)。标准布局（51B），CRC_EXTRA=143。
/// 坐标系固定 `MAV_FRAME_LOCAL_NED`。
#[allow(clippy::too_many_arguments)]
pub fn encode_set_position_target_local_ned(
    time_boot_ms: u32,
    type_mask: u16,
    x: f32, y: f32, z: f32,
    vx: f32, vy: f32, vz: f32,
    afx: f32, afy: f32, afz: f32,
    yaw: f32, yaw_rate: f32,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 51];
    p[0..4].copy_from_slice(&time_boot_ms.to_le_bytes());
    p[4] = enums::MAV_FRAME_LOCAL_NED; // coordinate_frame
    p[5..7].copy_from_slice(&type_mask.to_le_bytes());
    put_f32(&mut p, 7, x);
    put_f32(&mut p, 11, y);
    put_f32(&mut p, 15, z);
    put_f32(&mut p, 19, vx);
    put_f32(&mut p, 23, vy);
    put_f32(&mut p, 27, vz);
    put_f32(&mut p, 31, afx);
    put_f32(&mut p, 35, afy);
    put_f32(&mut p, 39, afz);
    put_f32(&mut p, 43, yaw);
    put_f32(&mut p, 47, yaw_rate);
    encode(msg_id::SET_POSITION_TARGET_LOCAL_NED, seq, &p, out)
}

/// 从 payload 解析 SET_POSITION_TARGET_LOCAL_NED（需先经 [`decode`] 取出 payload）。
pub fn decode_set_position_target_local_ned(payload: &[u8]) -> Option<SetPositionTargetLocalNed> {
    if payload.len() < 51 {
        return None;
    }
    let rd = |o: usize| f32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    Some(SetPositionTargetLocalNed {
        time_boot_ms: u32::from_le_bytes(payload[0..4].try_into().unwrap()),
        type_mask: u16::from_le_bytes([payload[5], payload[6]]),
        x: rd(7),
        y: rd(11),
        z: rd(15),
        vx: rd(19),
        vy: rd(23),
        vz: rd(27),
        afx: rd(31),
        afy: rd(35),
        afz: rd(39),
        yaw: rd(43),
        yaw_rate: rd(47),
    })
}

/// HIL_ACTUATOR_CONTROLS(93) 解码结果（飞控 -> 仿真器：执行器推力回传）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HilActuatorControls {
    pub time_usec: u64,
    pub controls: [f32; 16], // controls[0..4] 为 4 路电机归一化推力 [0,1]
    pub mode: u8,            // MAV_MODE_FLAG 位（含 HIL_ENABLED）
    pub flags: u64,          // 编码标志位（本实现置 0）
}

/// 编码 HIL_ACTUATOR_CONTROLS(93)。标准布局（81B），CRC_EXTRA=47。
pub fn encode_hil_actuator_controls(
    time_usec: u64,
    controls: &[f32; 16],
    mode: u8,
    flags: u64,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 81];
    p[0..8].copy_from_slice(&time_usec.to_le_bytes());
    for i in 0..16 {
        put_f32(&mut p, 8 + i * 4, controls[i]);
    }
    p[72] = mode;
    p[73..81].copy_from_slice(&flags.to_le_bytes());
    encode(msg_id::HIL_ACTUATOR_CONTROLS, seq, &p, out)
}

/// 从 payload 解析 HIL_ACTUATOR_CONTROLS（需先经 [`decode`] 取出 payload）。
pub fn decode_hil_actuator_controls(payload: &[u8]) -> Option<HilActuatorControls> {
    if payload.len() < 81 {
        return None;
    }
    let mut controls = [0f32; 16];
    for i in 0..16 {
        controls[i] = f32::from_le_bytes(payload[8 + i * 4..8 + i * 4 + 4].try_into().unwrap());
    }
    Some(HilActuatorControls {
        time_usec: u64::from_le_bytes(payload[0..8].try_into().unwrap()),
        controls,
        mode: payload[72],
        flags: u64::from_le_bytes(payload[73..81].try_into().unwrap()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::Frame;

    fn frame_from_slice(s: &[u8]) -> Frame {
        Frame::from_bytes(s)
    }

    #[test]
    fn crc_extra_appended_for_heartbeat() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_heartbeat(0, false, 7, &mut out);
        assert_eq!(out[0], MAVLINK_MAGIC);
        assert_eq!(u32::from(out[7]), msg_id::HEARTBEAT);
        // v2 帧长应为 10(头部) + 9(payload) + 2(crc) = 21
        assert_eq!(n, 21);
        let frame = frame_from_slice(&out[..n]);
        let (id, _pl) = decode(&frame).expect("heartbeat should decode with CRC_EXTRA");
        assert_eq!(id, msg_id::HEARTBEAT);
    }

    #[test]
    fn decode_rejects_wrong_crc_extra() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode(msg_id::SYS_STATUS, 0, &[0u8; 31], &mut out);
        out[n - 1] ^= 0xFF; // 篡改 CRC 第二字节
        let frame = frame_from_slice(&out[..n]);
        assert!(decode(&frame).is_none());
    }

    #[test]
    fn heartbeat_uses_standard_fields() {
        let mut out = [0u8; MAX_FRAME_LEN];
        encode_heartbeat(5, true, 0, &mut out);
        let frame = frame_from_slice(&out[..21]);
        let (_id, pl) = decode(&frame).unwrap();
        // type=QUADROTOR(2), autopilot=ARDUPILOT(3), base_mode 含 ARM 位
        assert_eq!(pl[0], enums::MAV_TYPE_QUADROTOR);
        assert_eq!(pl[1], enums::MAV_AUTOPILOT_ARDUPILOTMEGA);
        assert!(pl[2] & enums::MAV_MODE_FLAG_SAFETY_ARMED != 0);
        assert!(pl[2] & enums::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED != 0);
        // custom_mode = 5
        assert_eq!(u16::from_le_bytes([pl[3], pl[4]]), 5);
        // system_status = ACTIVE(4) —— 标准布局位于 payload[7]
        assert_eq!(pl[7], enums::MAV_STATE_ACTIVE);
    }

    #[test]
    fn command_long_roundtrip() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_command_long(
            enums::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 3, &mut out,
        );
        let frame = frame_from_slice(&out[..n]);
        let (id, pl) = decode(&frame).unwrap();
        assert_eq!(id, msg_id::COMMAND_LONG);
        let cmd = decode_command_long(pl).unwrap();
        assert_eq!(cmd.command, enums::MAV_CMD_COMPONENT_ARM_DISARM);
        assert_eq!(cmd.params[0], 1.0);
        assert_eq!(cmd.confirmation, 0);
    }

    #[test]
    fn param_value_roundtrip() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let id = *b"Thrust\0\0\0\0\0\0\0\0\0\0"; // 16 字节
        let n = encode_param_value(&id, 0.75, enums::MAV_PARAM_TYPE_REAL32, 10, 2, 0, &mut out);
        let frame = frame_from_slice(&out[..n]);
        let (_id, pl) = decode(&frame).unwrap();
        let pv = decode_param_value(pl).unwrap();
        assert_eq!(pv.id, id);
        assert!((pv.value - 0.75).abs() < 1e-6);
        assert_eq!(pv.param_type, enums::MAV_PARAM_TYPE_REAL32);
        assert_eq!(pv.param_count, 10);
        assert_eq!(pv.param_index, 2);
    }

    #[test]
    fn local_pos_sys_id_override() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_local_pos_from_raw(7, 0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0, &mut out);
        assert_eq!(out[5], 7); // v2 头部位置 [5] = sys_id，覆盖生效
        let frame = frame_from_slice(&out[..n]);
        assert!(decode(&frame).is_some());
    }

    #[test]
    fn attitude_uses_euler_layout() {
        // 标准 ATTITUDE：28B = time_boot_ms i32 + roll/pitch/yaw f32 + 3 rates f32。
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_attitude_raw(1234, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 0, &mut out);
        assert_eq!(n, 10 + 28 + 2);
        let frame = frame_from_slice(&out[..n]);
        let (_id, pl) = decode(&frame).expect("ATTITUDE should decode (CRC_EXTRA ok)");
        assert_eq!(pl.len(), 28);
        assert_eq!(i32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]), 1234);
        // roll ~ 0.1 rad
        assert!((f32::from_le_bytes([pl[4], pl[5], pl[6], pl[7]]) - 0.1).abs() < 1e-3);
        // yaw ~ 0.3 rad
        assert!((f32::from_le_bytes([pl[12], pl[13], pl[14], pl[15]]) - 0.3).abs() < 1e-3);
        // yawspeed = 3.0
        assert!((f32::from_le_bytes([pl[24], pl[25], pl[26], pl[27]]) - 3.0).abs() < 1e-3);
    }

    #[test]
    fn vfr_hud_standard_layout() {
        // 标准 VFR_HUD：20B = airspeed f32, groundspeed f32, heading i16(cdeg),
        // throttle u16(%), alt f32, climb f32。
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_vfr_hud_raw(5.0, 2865, 42, 50.0, -1.0, 0, &mut out);
        assert_eq!(n, 10 + 20 + 2);
        let frame = frame_from_slice(&out[..n]);
        let (_id, pl) = decode(&frame).expect("VFR_HUD should decode (CRC_EXTRA ok)");
        assert_eq!(pl.len(), 20);
        assert_eq!(f32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]), 0.0); // airspeed
        // groundspeed = 5.0
        assert!((f32::from_le_bytes([pl[4], pl[5], pl[6], pl[7]]) - 5.0).abs() < 1e-3);
        // heading cdeg = 2865
        assert_eq!(i16::from_le_bytes([pl[8], pl[9]]), 2865);
        // throttle = 42 (%)
        assert_eq!(u16::from_le_bytes([pl[10], pl[11]]), 42);
        // alt = 50.0
        assert!((f32::from_le_bytes([pl[12], pl[13], pl[14], pl[15]]) - 50.0).abs() < 1e-3);
        // climb = -1.0
        assert!((f32::from_le_bytes([pl[16], pl[17], pl[18], pl[19]]) - (-1.0)).abs() < 1e-3);
    }

    #[test]
    fn mission_item_int_roundtrip() {
        let item = MissionItem {
            target_system: 1,
            target_component: 1,
            seq: 3,
            command: enums::MAV_CMD_NAV_TAKEOFF,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 312345678,
            y: 121234567,
            z: 12.5,
            frame: 3,
            current: 0,
            autocontinue: 1,
            mission_type: 0,
        };
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_mission_item_int(&item, &mut out);
        let frame = frame_from_slice(&out[..n]);
        let (id, pl) = decode(&frame).unwrap();
        assert_eq!(id, msg_id::MISSION_ITEM_INT);
        let got = decode_mission_item_int(pl).unwrap();
        assert_eq!(got, item);
    }

    #[test]
    fn hil_sensor_roundtrip_via_raw() {
        // 手工组一帧 HIL_SENSOR 载荷（标准 62B），验证 decode_hil_sensor 布局。
        let mut p = [0u8; 62];
        p[0..8].copy_from_slice(&1_000_000u64.to_le_bytes()); // time_usec
        p[8..12].copy_from_slice(&9.81f32.to_le_bytes()); // xacc
        p[16..20].copy_from_slice(&(-0.1f32).to_le_bytes()); // zacc
        p[56..58].copy_from_slice(&25i16.to_le_bytes()); // temperature
        p[58..62].copy_from_slice(&0xFFu32.to_le_bytes()); // fields_updated
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode(msg_id::HIL_SENSOR, 0, &p, &mut out);
        let frame = frame_from_slice(&out[..n]);
        let (id, pl) = decode(&frame).expect("HIL_SENSOR should decode (CRC_EXTRA=108)");
        assert_eq!(id, msg_id::HIL_SENSOR);
        let (t, xacc, _y, _z, _gx, _gy, _gz, _mx, _my, _mz, _ap, _dp, _pa, temp, fu) = decode_hil_sensor(pl).unwrap();
        assert_eq!(t, 1_000_000);
        assert!((xacc - 9.81).abs() < 1e-6);
        assert_eq!(temp, 25);
        assert_eq!(fu, 0xFF);
    }
}
