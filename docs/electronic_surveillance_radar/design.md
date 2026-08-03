---
Status: active
Last-reviewed: 2026-08-03
Authority: ESR 设计权威入口
Answers: ESR 模块是什么、和谁交互、设计文档怎么导航
---

# ESR 设计

ESR 模块模拟电子侦察接收机对辐射源的观测和估计。它的核心不是直接输出 truth emitter，而是把输入辐射源
场景、平台姿态、接收机配置和电磁环境转化为两个去真值化输出通道：

- observation output：设备观测记录。
- emitter output：系统估计的辐射源假设。

对外提供稳定 `EsrSession` 门面和四域配置；runtime patch 经 resolver 校验后立即提交，不提供 session 层
回滚。truth identity、预计算受扰结论和 pipeline internal context 都不进入公共输出合同。

## 心智模型：拦截流水线

ESR 的心智模型是**拦截流水线**：宽带前端 → 调谐频率-角度单元 → 观测提取 → 分选/假设。

1. **宽带前端**：固定 receive beam、预选器和设备损耗聚合所有进入前端的功率，独立于当前调谐通道，
   负责最大线性输入、同平台泄漏和强带外 blocking 边界。
2. **调谐频率-角度单元**：把调谐通道内的到达活动投影到固定接收时间单元，再按角单元和重叠频带归并。
3. **观测提取**：候选的 signal/interference 功率、有效驻留和脉冲截获机会驱动 post-channel
   SINR/intercept probability 与检测采样；测量噪声只能在 detection 成功后施加。
4. **分选/假设**：preprocess、cluster、deinterleave、associator 只消费实际生成的 observation；
   center frequency、bandwidth、PRI、bearing 等只能由观测统计得出，不复制 truth。

自然环境（大气、杂波）与 RF 发射事实分开输入。RF 发射帧（`RfEmissionFrame`）是意图中立的统一入口：
AR、ECM 或其它 RF 发射在接收链中没有"目标/干扰"角色差异，只依据波形、时频占用、方向和功率决定其
可观测性、可分辨性或干扰。

## 文档导航

- 模块边界、非目标、scan_rate_hz / dt_sec 反直觉差异、扫描窗口与坐标系语义、专项序列验证边界、设计变更规则 → [boundaries.md](boundaries.md)
- 数据流图、Public API 边界、时序、生命周期与状态所有权 → [data-flow.md](data-flow.md)
- 算法登记表（环境采样/扫描窗口/拦截门控/拦截检测链/预处理/聚类/假设关联）、每算法的实现边界与
  反直觉点、刻意不实现的死字段（`spectrum_occupancy_ratio`，cross-ref ESR-OQ-1）→ [algorithms.md](algorithms.md)

跨模块公共规则（public API 边界、四域配置、三层输出模型、runtime patch 立即提交策略、证据优先开发
模式等）见 `docs/common/contract.md`。
