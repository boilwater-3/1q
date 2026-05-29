# JSBSim 初始条件合同

> 日期：2026-05-30
> 范围：阶段 3，定义飞行器初态注入的接口合同与已知限制。

## 1. 当前支持的初态输入

### 1.1 位置

| 输入字段 | 参考系 | JSBSim 映射 | 约束 |
|----------|--------|-------------|------|
| `position_lla_deg_m` | WGS84 LLA | `SetLatitudeDegIC`, `SetLongitudeDegIC`, `SetAltitudeASLFtIC` | `position_frame == kLla` |
| `position_ecef_m` | ECEF XYZ | 先转 LLA 再映射 | `position_frame == kEcef`，依赖 `TryEcefToLla` |

两种位置输入最终都映射到 JSBSim `FGInitialCondition` 的 LLA 接口。ECEF 模式通过项目坐标转换层（`coordinate::TryEcefToLla`）转为 LLA。

**已知限制：**
- `SetAltitudeASLFtIC` 设置的是海拔高度（above sea level），不是地心高度。
- ECEF->LLA 转换精度取决于项目 `TryEcefToLla` 实现。

### 1.2 速度

| 输入字段 | 参考系 | JSBSim 映射 | 适用条件 |
|----------|--------|-------------|----------|
| `velocity_mps.{x,y,z}_mps` | Body UVW | `SetUBodyFpsIC`, `SetVBodyFpsIC`, `SetWBodyFpsIC` | `initial_velocity_frame == kBody`（默认） |
| `velocity_mps.{x,y,z}_mps` | ECEF -> NED | `SetVNorthFpsIC`, `SetVEastFpsIC`, `SetVDownFpsIC` | `initial_velocity_frame == kEcef`，需要位置信息 |

Body UVW 模式：`x` 为机头方向前向速度，`y` 为机身右侧，`z` 为机身下方。

ECEF 模式：速度先在 LLA 位置处转为 NED（`TryEcefToNedVelocity`），然后注入 JSBSim 的 NED 速度 IC。如果 ECEF->NED 转换失败，fallback 到 Body UVW。

**已知限制：**
- ECEF 速度注入依赖同时有效的 LLA 位置信息（用于 ECEF->NED 转换的参考点）。
- 当前不支持直接设置 `VtrueFpsIC`、`MachIC`、`SetVCasIC`。
- 当前不支持直接设置 alpha/beta 作为初态参数。

### 1.3 姿态

| 输入字段 | 单位 | JSBSim 映射 |
|----------|------|-------------|
| `attitude_deg.roll_deg` | deg | `SetPhiRadIC`（转为 rad） |
| `attitude_deg.pitch_deg` | deg | `SetThetaRadIC`（转为 rad） |
| `attitude_deg.yaw_deg` | deg | `SetPsiRadIC`（转为 rad） |

**已知限制：**
- `EulerAnglesDeg` 字段命名顺序为 `yaw_deg`、`pitch_deg`、`roll_deg`，与注入顺序 `Phi, Theta, Psi` 对应关系明确，但字段顺序（yaw 在前）与 ZYX 旋转约定一致，与 FG 体系不同。
- 不支持直接设置四元数初态。

### 1.4 发动机与起落架

发动机和起落架不在 `ExternalKinematics` 中，而是在 `JsbsimAdapter` 构造函数中硬编码处理：

| 步骤 | JSBSim 调用 | 触发条件 |
|------|------------|----------|
| 启动所有发动机 | `InitRunning(-1)` | 始终执行（在 `RunIC()` 之后） |
| 收起起落架 | `gear/gear-cmd-norm = 0` | 初始高度 > 10m |

**已知限制：**
- 所有发动机强制启动，不区分多发机型的部分启动场景。
- 起落架收起阈值 10m 是硬编码，不可配置。
- 不设置油门初始值；`InitRunning` 启动发动机但不设置推力等级，trim 或 C++ AP 后续需要显式设 throttle。
- 不设置混合比（mixture），对活塞发动机（c310、c172x、L410、B17）可能需要额外初始化。

## 2. 初始化流程步骤（当前实现）

```
1. 创建 FGFDMExec
2. 设置 debug level (silent_mode -> 0)
3. 设置 dt
4. LoadAircraft() -- 加载 XML 模型
   ├── SetRootDir, SetAircraftPath, SetEnginePath, SetSystemsPath
   └── LoadModel(aircraft_model, true)
5. ConfigureIntegrators() -- 设置 integrator/gravity 模式
   └── 额外设置 guidance/roll-angle-limit 和 roll-rate-limit（如果存在）
6. ApplyInitialConditions() -- 设置位置、速度、姿态
7. RunIC() -- 执行初态计算
8. InitRunning(-1) -- 启动所有发动机
9. RetractLandingGear() -- 空中初态时收起落架
10. DoTrim(0) -- 如果 config.do_trim == true
    └── 成功：完成
    └── 失败（异常）：
        a. 重新 ApplyInitialConditions()
        b. 重新 RunIC()
        c. 重新 InitRunning(-1)
        d. 重新 RetractLandingGear()
        e. ResetControlStateAfterTrimFailure() -- 清零所有 FCS 状态
```

## 3. JSBSim 原生 IC 能力（当前未使用）

JSBSim `FGInitialCondition` 支持但当前 `ApplyInitialConditions` 未使用的能力：

| 能力 | JSBSim API | 适用场景 |
|------|-----------|----------|
| 真空速 | `SetVtrueFpsIC` | 按 TAS 而非 body UVW 指定速度 |
| 校准空速 | `SetVCasIC` | 按 IAS/KCAS 指定速度 |
| 马赫数 | `SetMachIC` | 跨音速/超音速机型 |
| 攻角 | `SetAlphaDegIC` | 精确指定初始 alpha |
| 侧滑角 | `SetBetaDegIC` | 精确指定初始 beta |
| 航迹角 | `SetFlightPathAngleDegIC` | 指定初始爬升/下降角 |
| 爬升率 | `SetClimbRateFpsIC` | 指定初始 ROC |
| 发动机运行 | 通过 XML `<running>` 或 `InitRunning` | 在 IC 阶段控制发动机状态 |

**对 6 个重点机型的影响：**

| 机型 | 当前速度设置方式 | 问题 |
|------|----------------|------|
| f22 | Body UVW x=200 m/s | 不考虑 alpha 分量；初始 alpha 与给定的 UVW 可能不一致 |
| Concorde | Body UVW x=150 m/s | 同上；高速飞行器 alpha 对 UVW 分量影响大 |
| B17 | Body UVW x=80 m/s | 活塞发动机可能需要 mixture 初始化 |
| C130 | Body UVW x=90 m/s | 涡桨发动机可能需要特殊初始化 |
| L410 | Body UVW x=90 m/s | 双发涡桨，同 C130 |
| c310 | Body UVW x=65 m/s | 双发活塞，需要 mixture/priming |

## 4. Trim 行为合同

### 4.1 当前 trim 策略

- Trim 模式固定为 `0`（`DoTrim(0)`），即纵向配平（longitudinal trim）。
- Trim 失败不阻止构造；捕获异常后重置 FCS 状态并继续。
- Trim 失败恢复后，控制面、integrator 和 trim command 全部清零。

### 4.2 Trim 结果可观察状态

| 属性 | 类型 | 说明 |
|------|------|------|
| `TrimAttempted()` | bool | `config.do_trim == true` |
| `TrimSucceeded()` | bool | `DoTrim(0)` 未抛异常 |

### 4.3 Trim 已知问题

- `DoTrim(0)` 是纵向配平，不配平横向和偏航；FBW 机型（f16、f22）的 FBW integrator 初态在 trim 后未明确清零。
- trim 失败恢复重置所有 FCS 属性为 0，但 FBW integrator 属性名可能不在重置列表中（当前列表只含 `fcs/pitch-rate-integrator`、`fcs/roll-rate-integrator`、`fcs/yaw-rate-integrator`）。
- f22 的 `fcs/roll-rate-integrator` 在 reset 列表中，但 f22 的实际 roll integrator 可能在 XML chain 中使用不同 property 名。

## 5. 判据层级定义

| 层级 | 名称 | 判据 | 意义 |
|------|------|------|------|
| L0 | 可加载 | `LoadModel()` 返回 true | XML 语法正确，模型数据完整 |
| L1 | 可初始化 | `RunIC()` 返回 true | 初态在模型包线内，运动学一致 |
| L2 | 可短时运行 | 10 秒自由飞行不崩溃、无 NaN、高度 > 0 | 基本动态稳定性 |
| L3 | 可配平 | `DoTrim(0)` 不抛异常，trim 后状态合理 | 纵向平衡 |
| L4 | 可控完成机动 | 在 AP 控制下完成指定机动任务 | 完整控制链可用 |

**应用规则：**
- L0 失败 → 机型不可用，不应进入测试集。
- L1 失败 → 初态超出包线或运动学不一致，需要调整初态参数。
- L2 失败但 L1 通过 → 动态不稳定或 trim 引入异常，需要诊断。
- L3 失败但 L2 通过 → 可在无 trim 模式下使用，需要 C++ AP 补偿。
- L4 失败但 L3 通过 → 控制链接口不匹配或机动参数不当，需要 profile 调整。

## 6. 6 个重点机型推荐测试初态

| 机型 | 推荐高度 (m) | 推荐速度 (m/s) | 推荐速度参考 | 备注 |
|------|-------------|---------------|-------------|------|
| f22 | 3000 | 200 | Body UVW | 无 reset00.xml；trim 失败后仍可 L2 |
| Concorde | 5000 | 150 | Body UVW | 有 reset00.xml；Orbit 任务需更高速度 |
| B17 | 1000 | 80 | Body UVW | 活塞发动机；需确认 mixture |
| C130 | 1000 | 90 | Body UVW | 涡桨；重型运输机 |
| L410 | 1000 | 90 | Body UVW | 双发涡桨通勤机 |
| c310 | 500 | 65 | Body UVW | 双发活塞；有 native AP |

## 7. 后续改进方向

1. **扩展 IC 接口**：支持 `alpha`、`gamma`、`vtrue` 直接设置，避免 body UVW 的 alpha 歧义。
2. **发动机初始化合同**：支持按机型配置发动机启动数量、油门初值、混合比初值。
3. **起落架收起阈值**：可配置化，而非硬编码 10m。
4. **Trim 模式可配**：支持不同 trim 模式（longitudinal=0, full=1 等）。
5. **Trim 后 FCS 状态导出**：将 trim 后的关键 FCS 状态（elevator/aileron/rudder position、throttle）导出为可观测诊断信息。
