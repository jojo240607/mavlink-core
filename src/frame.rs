//! MAVLink v2 帧层（与具体消息无关的地基）。
//!
//! 实现标准 MAVLink v2 帧格式 + CRC16/X25 + **CRC_EXTRA**（地面站识别飞控的硬门槛）。
//! 全部固定大小、无堆、无依赖，可在嵌入式端与 host 侧同时编译。
//!
//! 与 v1 的差异：
//! - magic = `0xFD`（v1 为 `0xFE`）；
//! - 头部 6 字节扩为 **9 字节**：在 `len,seq,sys,comp` 之后 `msgid` 由 1 字节扩为 **3 字节小端**，
//!   并新增 `incompat_flags(1) + compat_flags(1)`（本实现均置 0，不用签名）；
//! - 头部布局：`len(1) incompat(1) compat(1) seq(1) sys(1) comp(1) msgid(3 LE)`；
//! - payload ≤ 255，CRC16 + CRC_EXTRA（与 v1 同算法，仅头部长度不同）。
//!
//! CRC_EXTRA 取自标准 common.xml 生成常量（见 [`CRC_EXTRA`]）。

/// MAVLink v2 帧起始符。
pub const MAVLINK_MAGIC: u8 = 0xFD;

/// 单帧最大负载 + 头部开销的硬上限（MAVLink v2 最大 279，这里留余量）。
pub const MAX_FRAME_LEN: usize = 280;

/// 一段已封装的链路帧（含 MAVLink 报文 + 链路层开销）。
/// 用固定大小数组 + 长度字段，避免堆分配。
#[derive(Clone, Copy)]
pub struct Frame {
    pub data: [u8; MAX_FRAME_LEN],
    pub len: usize,
}

impl Frame {
    /// 由原始字节构造（长度 clamp 到上限）。
    pub fn from_bytes(buf: &[u8]) -> Self {
        let n = buf.len().min(MAX_FRAME_LEN);
        let mut data = [0u8; MAX_FRAME_LEN];
        data[..n].copy_from_slice(&buf[..n]);
        Frame { data, len: n }
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for Frame {
    fn default() -> Self {
        Frame { data: [0u8; MAX_FRAME_LEN], len: 0 }
    }
}

/// 系统/组件 ID（飞控侧固定）。
pub const SYS_ID: u8 = 1;
pub const COMP_ID: u8 = 1; // MAV_COMP_ID_AUTOPILOT1

/// 消息 ID 常量（与标准 MAVLink 一致）。
pub mod msg_id {
    pub const HEARTBEAT: u32 = 0;
    pub const SYS_STATUS: u32 = 1;
    pub const GPS_RAW_INT: u32 = 24;
    pub const RC_CHANNELS: u32 = 65;
    pub const FENCE_STATUS: u32 = 162;
    pub const PARAM_REQUEST_LIST: u32 = 21;
    pub const PARAM_VALUE: u32 = 22;
    pub const PARAM_SET: u32 = 23;
    pub const PARAM_REQUEST_READ: u32 = 20;
    pub const ATTITUDE: u32 = 30;
    pub const GLOBAL_POSITION_INT: u32 = 33;
    pub const LOCAL_POSITION_NED: u32 = 32;
    pub const VFR_HUD: u32 = 74;
    pub const COMMAND_LONG: u32 = 76;
    pub const COMMAND_ACK: u32 = 77;
    pub const AUTOPILOT_VERSION: u32 = 300;
    // 航点（MISSION）系列
    pub const MISSION_REQUEST_LIST: u32 = 43;
    pub const MISSION_COUNT: u32 = 44;
    pub const MISSION_CLEAR_ALL: u32 = 45;
    pub const MISSION_ACK: u32 = 47;
    pub const MISSION_REQUEST: u32 = 40;
    pub const MISSION_ITEM_INT: u32 = 73;
    // RC 通道覆盖（地面站手动操控）
    pub const RC_CHANNELS_OVERRIDE: u32 = 70;
    // 围栏（FENCE）系列
    pub const FENCE_POINT: u32 = 160;
    pub const FENCE_FETCH_POINT: u32 = 161;
    // 数据流速率控制
    pub const REQUEST_DATA_STREAM: u32 = 66;
    pub const DATA_STREAM: u32 = 67;
    // HIL（硬件在环）
    pub const HIL_SENSOR: u32 = 107;
    pub const HIL_GPS: u32 = 113;
    pub const HIL_STATE_QUATERNION: u32 = 115;
    pub const HIL_RC_INPUTS_RAW: u32 = 92;
    // HIL 设定点 / 执行器回传（标准 common.xml）
    pub const SET_POSITION_TARGET_LOCAL_NED: u32 = 84;
    pub const HIL_ACTUATOR_CONTROLS: u32 = 93;
}

/// 标准 MAVLink common.xml 的 CRC_EXTRA 值（按 msg_id 索引；无则为 0）。
/// 这些值由 mavgen 从 common.xml 字段定义 + 类型生成，是地面站校验帧合法性的必备字节。
/// 来源：标准 `common.xml`（v2.0 方言）。
pub const CRC_EXTRA: [u8; 301] = {
    let mut t = [0u8; 301];
    t[msg_id::HEARTBEAT as usize] = 50;
    t[msg_id::SYS_STATUS as usize] = 124;
    t[24] = 24; // GPS_RAW_INT(24) 标准 common.xml CRC_EXTRA（地面站解析 GPS 需要）
    t[65] = 118; // RC_CHANNELS(65) 标准 common.xml CRC_EXTRA
    t[162] = 189; // FENCE_STATUS(162) 标准 common.xml CRC_EXTRA
    t[msg_id::PARAM_REQUEST_LIST as usize] = 159;
    t[msg_id::PARAM_VALUE as usize] = 220;
    t[msg_id::PARAM_SET as usize] = 168;
    t[msg_id::PARAM_REQUEST_READ as usize] = 214;
    t[msg_id::AUTOPILOT_VERSION as usize] = 178;
    t[msg_id::ATTITUDE as usize] = 39;
    t[msg_id::GLOBAL_POSITION_INT as usize] = 104; // 标准 common.xml CRC_EXTRA
    t[msg_id::LOCAL_POSITION_NED as usize] = 185; // 标准 common.xml CRC_EXTRA (v2.0)
    t[msg_id::VFR_HUD as usize] = 20; // 标准 common.xml CRC_EXTRA
    t[msg_id::COMMAND_LONG as usize] = 152;
    t[msg_id::COMMAND_ACK as usize] = 143; // 标准 common.xml CRC_EXTRA
    t[msg_id::MISSION_REQUEST_LIST as usize] = 132;
    t[msg_id::MISSION_COUNT as usize] = 221;
    t[msg_id::MISSION_CLEAR_ALL as usize] = 232;
    t[msg_id::MISSION_ACK as usize] = 153;
    t[msg_id::MISSION_REQUEST as usize] = 230;
    t[msg_id::MISSION_ITEM_INT as usize] = 38;
    t[msg_id::RC_CHANNELS_OVERRIDE as usize] = 124;
    t[msg_id::FENCE_POINT as usize] = 78;
    t[msg_id::FENCE_FETCH_POINT as usize] = 68;
    t[msg_id::REQUEST_DATA_STREAM as usize] = 148; // 标准 common.xml
    t[msg_id::DATA_STREAM as usize] = 21;          // 标准 common.xml
    t[msg_id::HIL_SENSOR as usize] = 108;          // 标准 common.xml：HIL_SENSOR(107) CRC_EXTRA=108
    t[msg_id::HIL_GPS as usize] = 55;              // 标准 common.xml：HIL_GPS(113) CRC_EXTRA=55
    t[msg_id::HIL_STATE_QUATERNION as usize] = 4;  // 标准 common.xml
    t[msg_id::HIL_RC_INPUTS_RAW as usize] = 54;    // 标准 common.xml
    t[msg_id::SET_POSITION_TARGET_LOCAL_NED as usize] = 143; // 标准 common.xml：SET_POSITION_TARGET_LOCAL_NED(84) CRC_EXTRA=143
    t[msg_id::HIL_ACTUATOR_CONTROLS as usize] = 47;          // 标准 common.xml：HIL_ACTUATOR_CONTROLS(93) CRC_EXTRA=47
    t
};

/// CRC16/MCRF4XX（标准 MAVLink v2 帧校验多项式，与 pymavlink/fastcrc 一致）。
/// 参数 `crc` 为初始值（标准用 0xFFFF），逐字节累积后返回。
pub fn crc16_x25(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        crc ^= b as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

/// 组一帧 MAVLink v2 报文（含 magic/9 字节头部/payload/crc + CRC_EXTRA）。
/// `seq` 由调用方维护（跨帧递增）。`payload` 长度必须 ≤ 255。
/// 头部布局：`len(1) incompat(1) compat(1) seq(1) sys(1) comp(1) msgid(3 LE)`。
/// 与标准地面站字节级兼容：`crc = CRC16_X25(CRC16_X25(0xFFFF, header+payload), CRC_EXTRA[msgid])`。
// ★★§5.220【悬垂输出缓冲守卫 ✓】输出缓冲若落在**当前 SP 之下**，就指向"已返回帧的死区" ✗
//   —— 那是 §5.219 抓到的真实故障：写它会踩掉该区域的**返回地址** ⇒ 之后 `pop {pc}` 野跳转 ✓
//   判据 ✓：地址落在【调用方声明的栈竞技场 `[GUARD_LO, GUARD_HI)`】内、且 `< SP` ⇒ 必是死区 ✓
//   （只按 `< SP` 判会**误伤静态/`.bss` 缓冲** ✗ —— 它们天生在栈下方 ✓；故需显式竞技场范围 ✓）
//   行为 ✓：**拒绝写入**（返回 0 ✓）—— 帧缺失远好于踩碎返回地址 ✓；并记录粘性证据 ✓。
#[cfg(target_arch = "arm")]
#[used]
pub static mut GUARD_LO: usize = 0;
/// 栈竞技场上界（不含）；`LO == HI == 0` ⇒ 关闭守卫 ✓（host 侧无需设置 ✓）
#[cfg(target_arch = "arm")]
#[used]
pub static mut GUARD_HI: usize = 0;
/// 粘性证据：被拒绝的写入次数 / 最近一次的缓冲地址 / 最近一次的**调用者返回地址** ✓
#[cfg(target_arch = "arm")]
pub static BAD_BUF_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(target_arch = "arm")]
pub static BAD_BUF_ADDR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(target_arch = "arm")]
pub static BAD_BUF_LR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

pub fn encode(msgid: u32, seq: u8, payload: &[u8], out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    // ★§5.220：入口处读 SP/LR（此处 LR 仍是**调用者返回地址** ✓，与非叶子处不同 ✓）
    #[cfg(target_arch = "arm")]
    {
        let sp: usize;
        let lr: usize;
        unsafe {
            core::arch::asm!(
                "mov {0}, sp",
                "mov {1}, lr",
                out(reg) sp,
                out(reg) lr,
                options(nomem, nostack, preserves_flags)
            );
        }
        let base = out.as_ptr() as usize;
        let (lo, hi) = unsafe { (GUARD_LO, GUARD_HI) };
        if lo != hi && base >= lo && base < hi && base < sp {
            use core::sync::atomic::Ordering;
            BAD_BUF_COUNT.fetch_add(1, Ordering::Relaxed);
            BAD_BUF_ADDR.store(base as u32, Ordering::Relaxed);
            BAD_BUF_LR.store(lr as u32, Ordering::Relaxed);
            return 0;
        }
    }
    let plen = payload.len().min(255);
    // 直接写入 out，避免额外 280 字节中转缓冲（栈敏感场景）。
    out[0] = MAVLINK_MAGIC;
    out[1] = plen as u8;
    out[2] = 0; // incompat_flags（无签名）
    out[3] = 0; // compat_flags
    out[4] = seq;
    out[5] = SYS_ID;
    out[6] = COMP_ID;
    // msgid 以小端写入 3 字节（v2 扩展消息 ID，支持 ≥256 的标准消息如 AUTOPILOT_VERSION=300）。
    out[7] = msgid as u8;
    out[8] = (msgid >> 8) as u8;
    out[9] = (msgid >> 16) as u8;
    out[10..10 + plen].copy_from_slice(&payload[..plen]);
    // 标准 MAVLink v2 CRC：先对 9 字节头部+payload 算 CRC16，再异或 CRC_EXTRA 字节。
    let mut crc = crc16_x25(0xFFFF, &out[1..10 + plen]);
    crc = crc16_x25(crc, &[CRC_EXTRA[msgid as usize]]);
    out[10 + plen] = (crc & 0xFF) as u8;
    out[10 + plen + 1] = (crc >> 8) as u8;
    10 + plen + 2
}

/// 从一帧 `Frame` 解析出 (msgid, payload_slice)；非 MAVLink v2 帧或 CRC（含 CRC_EXTRA）不通过返回 None。
/// msgid 返回 u32（v2 三字节 ID 空间，可支持 ≥256 的标准消息如 AUTOPILOT_VERSION=300）。
pub fn decode(frame: &Frame) -> Option<(u32, &[u8])> {
    let d = frame.as_slice();
    if d.len() < 12 || d[0] != MAVLINK_MAGIC {
        return None;
    }
    let plen = d[1] as usize;
    if d.len() < 10 + plen + 2 {
        return None;
    }
    // v2：msgid 为 3 字节小端（位于头部 [7..10]）。
    let msgid = d[7] as u32 | (d[8] as u32) << 8 | (d[9] as u32) << 16;
    if (msgid as usize) >= CRC_EXTRA.len() {
        return None;
    } // 超出发射端已知 CRC_EXTRA 表范围
    let payload = &d[10..10 + plen];
    // 校验 CRC（含 CRC_EXTRA），与 encode 同算法（v2 头部 9 字节）。
    let mut crc = crc16_x25(0xFFFF, &d[1..10 + plen]);
    crc = crc16_x25(crc, &[CRC_EXTRA[msgid as usize]]);
    let got = ((d[10 + plen + 1] as u16) << 8) | (d[10 + plen] as u16);
    if crc != got {
        return None;
    }
    Some((msgid, payload))
}

// ── 小端字段读写（MAVLink 载荷均为 IEEE754/整数小端） ─────────────

/// 把 f32 以小端写入 buf 的 offset 处（MAVLink 用 IEEE754 小端）。
pub fn put_f32(buf: &mut [u8], off: usize, v: f32) {
    let b = v.to_le_bytes();
    buf[off..off + 4].copy_from_slice(&b);
}
pub fn put_i32(buf: &mut [u8], off: usize, v: i32) {
    let b = v.to_le_bytes();
    buf[off..off + 4].copy_from_slice(&b);
}
pub fn put_i16(buf: &mut [u8], off: usize, v: i16) {
    let b = v.to_le_bytes();
    buf[off..off + 2].copy_from_slice(&b);
}
pub fn put_u16(buf: &mut [u8], off: usize, v: u16) {
    let b = v.to_le_bytes();
    buf[off..off + 2].copy_from_slice(&b);
}

/// 标准 MAVLink 枚举常量（与 common.xml 对齐，供 QGC 正确识别）。
pub mod enums {
    /// MAV_TYPE：飞行器类型（HEARTBEAT.type）。
    pub const MAV_TYPE_QUADROTOR: u8 = 2;
    /// MAV_AUTOPILOT：自驾仪类型（HEARTBEAT.autopilot）。
    /// 选 ARDUPILOTMEGA(3) 使地面站（QGC/groundctrl）按 ArduCopter 自定义模式码解析模式名（STABILIZE/ALT_HOLD/LOITER/RTL/LAND…）。
    pub const MAV_AUTOPILOT_DEV: u8 = 13;
    pub const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;
    /// MAV_FRAME：坐标系（SET_POSITION_TARGET_LOCAL_NED.coordinate_frame 等）。
    pub const MAV_FRAME_LOCAL_NED: u8 = 1;
    /// MAV_MODE_FLAG 位（HEARTBEAT.base_mode）。注意 CUSTOM_MODE_ENABLED 是 bit7 = 0x80（标准值，不是 0x01）。
    pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: u8 = 0x80;
    pub const MAV_MODE_FLAG_TEST_ENABLED: u8 = 0x02;
    pub const MAV_MODE_FLAG_AUTO_ENABLED: u8 = 0x10;
    pub const MAV_MODE_FLAG_GUIDED_ENABLED: u8 = 0x08;
    pub const MAV_MODE_FLAG_STABILIZE_ENABLED: u8 = 0x04;
    pub const MAV_MODE_FLAG_HIL_ENABLED: u8 = 0x20;
    pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;
    /// MAV_STATE（HEARTBEAT.system_status）。
    pub const MAV_STATE_UNINIT: u8 = 0;
    pub const MAV_STATE_BOOT: u8 = 1;
    pub const MAV_STATE_CALIBRATING: u8 = 2;
    pub const MAV_STATE_STANDBY: u8 = 3;
    pub const MAV_STATE_ACTIVE: u8 = 4;
    pub const MAV_STATE_CRITICAL: u8 = 5;
    pub const MAV_STATE_EMERGENCY: u8 = 6;
    pub const MAV_STATE_POWEROFF: u8 = 7;
    /// MAV_CMD（COMMAND_LONG.command）子集。
    pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;
    pub const MAV_CMD_NAV_LAND: u16 = 21;
    pub const MAV_CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;
    pub const MAV_CMD_COMPONENT_ARM_DISARM: u16 = 400;
    pub const MAV_CMD_DO_SET_MODE: u16 = 176;
    pub const MAV_CMD_MISSION_START: u16 = 300;
    pub const MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: u16 = 520;
    /// MAV_CMD_SET_MESSAGE_INTERVAL：地面站设置单条消息的发送间隔（微秒）。
    pub const MAV_CMD_SET_MESSAGE_INTERVAL: u16 = 203;
    /// ArduCopter 自定义模式码（custom_mode 字段），地面站据此显示模式名（STABILIZE/ALT_HOLD/...）。
    pub const COPTER_MODE_STABILIZE: u16 = 0;
    pub const COPTER_MODE_ALT_HOLD: u16 = 2;
    pub const COPTER_MODE_LOITER: u16 = 5;
    pub const COPTER_MODE_RTL: u16 = 6;
    pub const COPTER_MODE_LAND: u16 = 9;
    pub const COPTER_MODE_GUIDED: u16 = 4;
    /// MAV_PARAM_TYPE（PARAM_VALUE/PARAM_SET.param_type）。
    pub const MAV_PARAM_TYPE_REAL32: u8 = 9;
    /// MAV_RESULT（COMMAND_ACK.result）。
    pub const MAV_RESULT_ACCEPTED: u8 = 0;
    pub const MAV_RESULT_TEMPORARILY_REJECTED: u8 = 1;
    pub const MAV_RESULT_DENIED: u8 = 2;
    pub const MAV_RESULT_UNSUPPORTED: u8 = 3;
    pub const MAV_RESULT_FAILED: u8 = 4;
    /// MAV_PROTOCOL_CAPABILITY 位（AUTOPILOT_VERSION.capabilities）。
    pub const MAV_PROTOCOL_CAPABILITY_MAVLINK2: u64 = 1 << 23;
    pub const MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT: u64 = 1 << 5;
}
