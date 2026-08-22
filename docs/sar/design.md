---
Status: active
Last-reviewed: 2026-08-23
Authority: SAR 设计权威入口
Answers: SAR 模块是什么、和谁交互、设计文档怎么导航
---

# SAR 设计

SAR 模块负责合成孔径雷达的回波仿真、完整孔径 raw IQ 消费、距离压缩、聚焦成像、图像质量摘要、
Recording/Replay 和运行期配置。对外提供稳定 `SarSession` 门面；算法部件、truth oracle、聚焦中间态和
证据矩阵保持 internal。

SAR 的心智模型是**信号处理链**：LFM 波形 → raw history 构造 → 距离压缩 → 聚焦成像（RDA/BP）→
图像质量评估。两条 raw history 来源（内部 echo 生成 / 外部 raw IQ）在统一成像入口汇合，聚焦算法
不关心数据来源。

## 文档导航

- 模块边界、非目标、dt_sec 反直觉差异、环境几何契约、设计变更规则 → [boundaries.md](boundaries.md)
- 数据流图、Public API 边界、时序、生命周期与状态所有权 → [data-flow.md](data-flow.md)
- 算法登记表（LFM/RDA/MoCo/BP/Omega-K/Multilook/calibration）、每算法的实现边界与反直觉点、
  刻意不实现的算法（CSA/PGA/二阶 MoCo） → [algorithms.md](algorithms.md)

跨模块公共规则（public API 边界、条件五域配置所有权、三层输出模型、会话配置直接赋值、运行期配置提交策略、
证据优先开发模式等）以 `docs/common/contract.md` / `session_contract.md` 为准；SAR 本身为四域（无 orientation）。
