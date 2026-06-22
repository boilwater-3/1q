# SAR 自聚焦相位误差真值基础验收报告

## 1. 验收范围

本阶段只验收低阶残余相位误差剖面、常量/线性不可观测分量去除与真值校正诊断，
不批准 PGA、图像修改、熵优化或 Session/public 接入。

## 2. 实现结果

- 新增内部 `SarAutofocusPhaseTruth` 无状态诊断模块。
- 生成 `[-1,1]` 归一化孔径坐标和低阶多项式相位误差。
- 使用离散最小二乘拟合并去除常量/线性不可观测分量。
- 输出可观测残余、严格反向校正剖面、RMS/峰值、均值和线性投影。
- 非有限系数和不足样本被拒绝并清空输出。

## 3. 测试证据

- 默认 `SarAutofocusPhaseTruthTest.*`：`4/4` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `SarAutofocusPhaseTruthTest.*`：`4/4` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

## 4. 审批结论

自聚焦相位误差真值基础完成当前平台审批。PGA 估计、强散射点选择、unwrap、迭代停止
和生产图像校正仍缺少独立真值，继续后置。
