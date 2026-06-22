# SAR 确定性噪声与分布式杂波 SNR/SCR 二维矩阵审批报告

## 1. 审批结论

阶段 55 联合 SNR/SCR 二维参考矩阵已完成当前平台审批。

- 噪声与杂波分别以纯目标 raw-history 能量缩放，requested 与 realized SNR/SCR 一致。
- 噪声 seed 与杂波 seed 的影响彼此隔离，固定全部参数时各分量和最终输入可重复。
- 噪声与杂波交换注入顺序后，最终输入逐样本在 `1e-15` 数值容差内一致。
- M1 完整 `3x3` SNR/SCR 矩阵和 M4 三档哨兵矩阵均通过。
- 所有联合输入下 BP 与 GBP 复图继续逐样本一致。

本阶段不批准生产噪声、生产杂波、质量阈值、警告、结构化拒绝或 Auto。

## 2. 实现范围

- 在 `tests/support/sar_reference_scene.h` 增加联合配置、联合 diagnostics 和组合 helper。
- 联合 helper 只复用既有确定性复高斯噪声与分布式杂波 helper。
- clean、noise-only、clutter-only 和 joint 输入均在 raw pulse history 层形成。
- 未修改生产算法、public API、Session、replay 或默认成像路径。

## 3. 首批矩阵

### M1 完整矩阵

- 中心单点目标。
- `3x3` sparse 杂波网格。
- SNR：无噪声、`20 dB`、`0 dB`。
- SCR：无杂波、`20 dB`、`0 dB`。
- 两组独立 `(noise_seed, clutter_seed)`。

### M4 哨兵矩阵

- 二维分离多目标。
- `5x5` dense 杂波网格。
- 档位：clean、`20 dB SNR + 20 dB SCR`、`0 dB SNR + 0 dB SCR`。
- 两组独立 seed 对。

## 4. 验证结果

- Windows Debug 编译：passed。
- 默认环境 `SarReferenceSnrScrMatrixTest.*`：`2/2 passed`。
- 默认环境全部 `SarReference*Test.*`：`13/13 passed`。
- Conan Eigen 3.3.9 `SarReferenceSnrScrMatrixTest.*`：`2/2 passed`。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：`1/1 passed`。
- 默认 Windows Debug 完整 CTest：`25/25 passed`。

## 5. 冻结边界

- 当前结果只批准测试侧固定网格、固定 seed、小场景联合参考矩阵。
- 不声明真实 clutter-to-noise ratio、绝对功率或辐射定标语义。
- 不扩展相关杂波、随机位置分布、海杂波、运动目标或真实斑点模型。
- 不据此冻结联合质量阈值、算法选择阈值或 Auto。

## 6. 下一阶段建议

进入联合 SNR/SCR 矩阵后续决策门，审计首批矩阵证据并决定是否需要扩展中间档位、
相关杂波研究或联合输入质量诊断。后续范围继续限定于 `1.1.4.4 SAR雷达组件`。
