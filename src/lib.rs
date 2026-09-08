//! MAVLink v2 共用编解码核心（跨工程单一事实来源）。
//!
//! 从 `flyctrl-core::comm::mavlink` 抽取并解耦 `VehicleState` 后独立成 crate，
//! 供四类消费方共用：
//! - **飞控固件核心**（flyctrl-core，no_std MCU）——经 re-export 使用；
//! - **仿真器**（fly-simulater）——SIL/HIL 链路与遥测桥直接依赖；
//! - **板载 App**（joc-app-rust）——经 flyctrl-core 间接使用；
//! - **地面站**（groundctrl）——使用 [`common`] 的官方 crate 兼容层。
//!
//! 分层：
//! - [`frame`]：MAVLink v2 帧层（magic/头部/CRC16+CRC_EXTRA/msg_id 常量/字段读写），
//!   与任何具体消息无关，是两层 API 的共同地基。
//! - [`codec`]：具体消息编解码（函数式 API：`encode_*` / `decode_*`），
//!   直接写固定缓冲、零堆，适合 MCU 与仿真器。
//! - [`common`]：地面站兼容层（`MavHeader` + `MavMessage` 枚举 + `*_DATA` 结构体 +
//!   `write_v2_msg` / `read_v2_msg`），命名与官方 `mavlink` crate 对齐，
//!   使 groundctrl 迁移只需改 import 路径。

#![no_std]

pub mod codec;
#[cfg(feature = "common")]
pub mod common;
pub mod frame;

pub use frame::{encode, decode, CRC_EXTRA, MAX_FRAME_LEN, msg_id, enums, Frame};
pub use codec::*;
