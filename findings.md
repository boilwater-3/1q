# 发现与决策 — 统一大气模型

## 现状分析

### AtmosphericPropagationInputs 14 字段
enable_physics, frequency_hz, path_length_m, radar_altitude_m, target_altitude_m,
elevation_deg, pressure_hpa, temperature_k, relative_humidity, k_factor,
day_of_year, solar_flux_f107a, solar_flux_f107, geomagnetic_ap

### 三处手工填充代码段

1. **PropagationModel.cpp L153-167** — AR 全局传播，使用 scene_state 的默认频率/路径
2. **DetectionExecution.cpp L52-77** — AR 逐目标，使用实际频率/距离/高度
3. **EsrEnvironmentService.cpp L105-173** — ESR，硬编码默认值（10GHz/10km/1km/5°）

### GTD7 近似实现 (AtmospherePhysics.cpp L86-120)
- 三段线性温度 + 指数密度衰减
- 包含季节、太阳、地磁因子（k_factor, day_of_year, solar_flux, geomagnetic_ap）
- 精度低于 ISA 1976 的 9 段断点

### ISA 1976 温度断点
```
海平面→11km: -6.5 °C/km (对流层)
11km→20km: 等温 216.65K
20km→32km: +1.0 °C/km
32km→47km: +2.8 °C/km
47km→51km: 等温 270.65K
51km→71km: -2.8 °C/km
71km→86km: -2.0 °C/km
```

### ISA 1976 气压公式
- 梯度层: P = P_b * (T_b / (T_b + L*ΔH))^(g0*M/(R*L))
- 等温层: P = P_b * exp(-g0*M*ΔH/(R*T_b))

### 物理常数 (SI)
- g0 = 9.80665 m/s²
- M = 0.0289644 kg/mol
- R = 8.31447 J/(mol·K)
- R_specific = R/M = 287.05287 J/(kg·K)
- γ = 1.4 (空气比热比)
- r_earth = 6356766.0 m (ISA 标准地球半径)

### CMakeLists.txt 现状
- 15 个源文件在 COMMON_SOURCES 列表中
- 公共头文件安装规则在 PUBLIC_HEADERS_FOUNDATION 列表中
- 需要新增 StandardAtmosphere.cpp + atmosphere_state.h + atmosphere_provider.h

## 技术决策

| 决策 | 理由 |
|------|------|
| BuildPropagationInputs 内部调用 ISA | 统一温度/气压/密度来源，消除重复计算 |
| 保留 GTD7 函数签名 | ABI 兼容，内部委托到 ISA |
| pressure_hpa 作为便利字段 | 消费者大量使用 hPa，避免 /100 转换 |
