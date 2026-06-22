# SAR L2 轨迹误差与一阶运动补偿审批报告

## 1. 审批结论

L2 连续扰动轨迹与一阶运动补偿在当前平台完成内部能力审批。

- 固定种子 L2 连续扰动轨迹：通过。
- 零扰动严格退化为 L1：通过。
- 相对显式参考点的一阶包络与相位运动补偿：通过。
- public Session 启用 L2 或运动补偿：不批准，继续后置。
- L3、二阶运动补偿、自聚焦和 Auto：继续后置。

## 2. L2 轨迹证据

- 三轴高斯速度扰动使用固定种子生成。
- 位置由前一脉冲速度按 `1/PRF` 积分，保证轨迹连续。
- 相同种子逐点一致，不同种子产生不同轨迹。
- 全零扰动显式返回 L1 轨迹和零误差诊断。
- 诊断覆盖最大/RMS 位置误差和最大/RMS 速度误差。

## 3. 一阶运动补偿证据

确定性单点场景、最大参考斜距误差约 `1.300414 m`：

| 指标 | 未补偿 | 一阶补偿后 | 验收 |
|---|---:|---:|---:|
| 单位能量形状 NRMS | `1.312794` | `0.249779` | 补偿后 `< 0.3` 且低于未补偿 |
| 相干相关系数 | `0.138285` | `0.968805` | 补偿后 `> 0.95` 且高于未补偿 |

零轨迹误差输入经过一阶补偿后 raw pulse history 保持逐样本一致。

## 4. 性能证据

当前平台 Debug：

- `1024x1024` raw pulse history 一阶运动补偿核心处理约 `0.052741 s`。
- 该性能结果只批准内部补偿内核，不改变 public Session 尺寸或默认路径。

## 5. 冻结边界

- public Session 继续使用 L1 理想轨迹，不新增 L2 public 配置。
- public Session 继续固定 RDA + linear RCMC。
- 一阶运动补偿使用单个显式参考点和 linear 包络插值。
- 不实现空间变化参考面、多参考点补偿或二阶残余误差校正。
- 不实现 L3、姿态误差、天线指向误差、IMU/GPS 误差、导航滤波、自聚焦或 Auto。
- Phase 1 `1024x1024` public Session 上限和 GBP `128x128` 上限不变。

## 6. 验证入口

- 默认与 Eigen 3.3.9 下的 L2/运动补偿/GBP/RDA/Session 聚焦过滤测试。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_performance --output-on-failure`
- `ctest --test-dir build/llvm-ninja-debug-eigen339-local -L sar_cxx11_compat --output-on-failure`
- `git diff --check`

## 7. 下一阶段建议

下一步应先讨论选择：

1. 将 L2 与一阶运动补偿受控接入 public Session。
2. 扩展多参考点/空间变化一阶补偿。
3. 转入 L3 轨迹与二阶残余误差校正。

以上方向均需单独工程契约和审批。
