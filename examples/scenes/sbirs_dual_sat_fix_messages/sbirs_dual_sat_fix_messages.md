# 场景预期表：sbirs_dual_sat_fix_messages（消息机制地面站版）

## 与 sbirs_dual_sat_fix 的差异

| 项 | sbirs_dual_sat_fix | 本场景 |
| --- | --- | --- |
| SBIRS → 地面站 | 写 `AppSceneState.sbirs_ground_station_inbox` | 发 `on_sbirs_frame_submitted`（`events.h` 展平载荷） |
| 地面站组件 | `FusionComponent`（读共享黑板） | `GroundStationFusionComponent`（成员 inbox + 订阅） |
| 周期前准备 | 清空黑板 | `fusion->BeginCycle(world, cycle)` |

几何、验收开关、预期指标与 `sbirs_dual_sat_fix` 相同；本场景验证**消息投递路径**与黑板路径等价。

事件在 `examples/core/events.h`：零库类型，卫星侧展平、地面站重建库类型后再喂融合。融合按本周期收件箱里的**全部卫星**处理；双星交会只是精度评估叠加（库 API 仍吃两颗星）。

## 消息流（每周期）

```
main: app_scene.cycle = N
main: fusion->BeginCycle(world, N)     // 连接信号 + 清空 inbox
World::Step:
  各卫星 Sbirs (kMessage) → on_sbirs_frame_submitted → fusion.sbirs_inbox_
  ground_station GroundStationFusion → 重建 + 适配 + FusionEngine::Update
                                     → 可选：从 N 星里按评估源通道挑两颗做 PE
```

## 机载源扩展（AR/RIR 等）

机载传感器可在 Step 末尾发布展平样本（不要塞 `fusion::DetectionRecord`）：

```cpp
DetectionBatchSubmittedEvent batch;
batch.cycle = scene.cycle;
batch.source_id = 1U;  // 调用方自己的源通道
FusionDetectionSample sample;
sample.key = ...;
sample.source_id = batch.source_id;
sample.has_bearing = true;
sample.bearing_az_deg = ...;
batch.records.push_back(sample);
world.signals().on_detection_batch_submitted(batch);
```

## 构建与运行

```bash
source scripts/activate_1q_git_bash.sh
scripts/1q.sh build VisualStudio.15.0-amd64-release --target sbirs_dual_sat_fix_messages
./build/VisualStudio.15.0-amd64/Release/bin/sbirs_dual_sat_fix_messages.exe
```

输出：`examples/log/sbirs_dual_sat_fix_messages/`（需四验收开关 ON，同 dual_sat_fix）。
