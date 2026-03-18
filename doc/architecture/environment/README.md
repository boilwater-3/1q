# Environment Docs Index

Environment 层负责产出传播、杂波和干扰等环境事实，供信号层和决策层按各自职责消费。

## Files

| File | Description |
|------|-------------|
| `environment-architecture.md` | **核心文档**：环境层职责、一期完成基线、二期开发指标、干扰建模边界与跨层数据流 |

## Recommended Reading Order

1. `environment-architecture.md`
2. `../decision/decision-architecture.md`
3. `../signal/signal-architecture.md`

落代码时，优先对照以下入口：

- `include/1q/airborne_radar/environment/IEnvironmentService.h`
- `include/1q/airborne_radar/environment/EnvironmentService.h`
- `src/airborne_radar/environment/EnvironmentService.cpp`
- `include/1q/airborne_radar/common/DecisionInputFrame.h`
