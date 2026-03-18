# 输出管理文档目录

输出管理模块负责把内部生命周期轨迹快照整理为稳定的对外输出视图，供事件广播、记录回放和外部接口按需消费。

## 文件说明

| 文件 | 说明 |
|------|------|
| `output-architecture.md` | **核心文档**：输出管理模块的定位、边界、候选方案对比与推荐落地路径 |

## 建议阅读顺序

1. 先看 `output-architecture.md` — 了解为何不能直接暴露对象池、以及是否应复用 `DecisionInputFrame`。
2. 再看 `../signal/signal-data-contracts.md` — 核对 `DecisionTrackSnapshot` 与对象池封装边界。
3. 最后看 `../decision/decision-architecture.md` — 理解 `DecisionInputFrame` 当前承载的决策语义。

落代码时，优先对照以下入口：

- `include/1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h`
- `include/1q/airborne_radar/common/DecisionTrackSnapshot.h`
- `include/1q/airborne_radar/common/DecisionInputFrame.h`
- `src/airborne_radar/core/controller/RadarController.cpp`
- `include/1q/airborne_radar/core/event/RadarEvents.h`
