---
Status: active
Last-reviewed: 2026-08-05
Authority: navigation 模块级边界、非目标与设计变更规则
Answers: navigation 有哪些模块级禁令、单位契约、变更规则
---

# Navigation 模块边界

本文承载 navigation 的模块级边界、非目标和变更规则。算法级边界与反直觉点见
[algorithms.md](algorithms.md)。

## 模块定位边界

1. navigation 是**独立中立算法面**：不引用 `flight_dynamic` 的任何类型，包括
   `flight_dynamic::guidance::Waypoint`（弧度制）与 `GreatCircleTrackGeometry`。
2. 输出 `RoutePoint` 为中性类型（度制 `LlaPositionDegM` + 速度 + 到达半径）；
   **与执行模块之间的单位转换属业务层适配职责**，不入库。
3. 区域数据（多边形/圆形，经纬高）由外部解析后传入，规划器不承担解析职责。
4. 航段执行（直线航段、转弯、到达判定）由执行侧承担；规划器只输出航点序列，
   相邻航点间隐含直连航段。

## 单位契约（冻结）

- 经纬度：**度制**（对齐 `oneq::coordinate` 域，`LlaPositionDegM`）。
- 高度/间距/半径/速度：米 / 米每秒。
- 扫描航向：度，0 = 扫描线沿正东，逆时针增大。
- 盘旋方向：逆时针（CCW），自圆心正北起取点。

## 非目标

1. 不做 8 字 / S 型等高级规划模式（首期仅区域覆盖，执行侧已有 `ManeuverExecutor`）。
2. 不引入外部身份通道、编队概念或任务语义（属 example 业务层）。
3. 不做凹多边形/自交多边形的完整拓扑处理（首期按简单多边形处理，
   顶点恰好落在扫描线上的退化情形产生零长段，近共线条带产生退化短段航路，
   属已知限制）。
4. 不提供 Session/Cycle 会话形态，不与传感器周期语义对齐。

## 设计变更规则

1. 任何输出单位或语义变化（如改为弧度制、改变扫描线起始偏移）必须同步本文档集
   与 `docs/review/Behavior.md` §3 冻结项。
2. 新增规划模式（8 字/S 型）必须先冻结模式语义与执行侧适配契约。
3. 若引入对 `flight_dynamic` 或新第三方库的依赖，必须重走证据矩阵并修订
   `docs/review/Behavior.md` §3.1 冻结决策。
