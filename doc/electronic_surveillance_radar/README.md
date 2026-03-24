# Electronic Surveillance Radar Docs

电子侦察雷达模块文档目录。该模块定位为真实电子侦察仿真组件，输入为平台状态、场景辐射源真值和环境快照，输出为接收机观测、辐射源假设、威胁评估和可选定位结果。

## Files

| File | Description |
|------|-------------|
| `electronic_surveillance_radar.md` | 需求基线：真实电子侦察模块的能力边界、首版范围与工程约束 |
| `electronic_surveillance_radar_architecture.md` | 架构设计：复用 `airborne_radar` 的骨架、ESR 模块拆分、边界与首批实现顺序 |
| `ElecReconProcess.h` | 旧参考类接口，设计质量很差，仅用于提取局部算法线索 |
| `ElecReconProcess.cpp` | 旧参考类实现，仅用于提取可复用算法胚子，不作为新模块结构模板 |

## Recommended Reading Order

1. `electronic_surveillance_radar.md`
2. `electronic_surveillance_radar_architecture.md`
3. `ElecReconProcess.cpp` 中被设计文档点名的局部算法段

落代码时，优先对照以下现有模块入口：

- `include/1q/airborne_radar/core/session/RadarSession.h`
- `include/1q/airborne_radar/core/controller/RadarController.h`
- `include/1q/airborne_radar/environment/IEnvironmentService.h`
- `include/1q/airborne_radar/signal/pipeline/ISignalPipeline.h`
- `src/airborne_radar/core/output/IDataOutputManager.h`
- `src/airborne_radar/signal/pipeline/SignalComponentFactory.h`

## Notes

- `airborne_radar` 可复用的是调度、装配、环境冻结和输出分层设计。
- `airborne_radar` 不可直接复用的是主动雷达专属的数据契约，例如 `TargetFeature -> detection -> association -> track` 这条主链。
- `ElecReconProcess.*` 仅可提供少量数值算法线索，不能作为对象设计参考。
