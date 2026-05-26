# JSBSim 脚本执飞机动模式调研

> 基于 `third_party/jsbsim` tag `v1.1.13` 分析。
> 脚本文件位于 `scripts/`，系统定义位于 `systems/`，机载系统位于 `aircraft/*/Systems/`。

---

## 一、航路点飞行

**核心系统**: `systems/GNCUtilities.xml` + `systems/Autopilot.xml`

**属性链**:

```
guidance/target_wp_latitude_rad    →  航路点纬度
guidance/target_wp_longitude_rad   →  航路点经度
guidance/wp-heading-rad            →  输出：到航路点的方位角
guidance/wp-distance               →  输出：到航路点的距离（ft）
guidance/heading-selector-switch   →  航向源选择（0=航路点方位 / 1=用户指定）
guidance/specified-heading-rad     →  用户指定航向
ap/active-waypoint                 →  当前航路点序号
```

**典型实现** (`scripts/c3105.xml`): 5 个航路点的闭环航线，通过事件链逐点切换：

```xml
<!-- 设定航路点 -->
<set name="guidance/target_wp_latitude_rad" value="0.517238"/>
<set name="guidance/target_wp_longitude_rad" value="-1.662727"/>
<set name="ap/heading-setpoint-select" value="1"/>
<set name="ap/active-waypoint" value="1"/>

<!-- 切换到下一个航路点 -->
<condition>
  guidance/wp-distance lt 700
  ap/active-waypoint eq 1
</condition>
<set name="guidance/target_wp_latitude_rad" value="0.517533"/>
<set name="guidance/target_wp_longitude_rad" value="-1.663076"/>
<set name="ap/active-waypoint" value="2"/>
```

**涉及的脚本**: `c3105.xml`, `c3104.xml`, `c1723.xml`, `c172_elevation_test.xml`

---

## 二、固定点/盘旋飞行

**核心系统**: `systems/Autopilot.xml`

实现为 Heading Hold 通道，逻辑链：

```
guidance/angle-to-heading-rad
  → pure_gain (限幅 ±30°)
  → roll-attitude-selector (0=机翼水平 / 1=滚转角保持)
  → summer (与 attitude/phi-rad 求差)
  → PID(30, 0.1, 0.1)
  → roll-rate 限幅 → PID(2.5, 0, 0) → lag_filter → aileron 指令
```

**模式切换**:

| `ap/roll-attitude-mode` | 行为 |
|---|---|
| 0 | 机翼水平（wing leveler） |
| 1 | 滚转角保持（按航向误差计算目标滚转角） |

**涉及的脚本**: `c1723.xml`, `c172_cruise_8K.xml`, `c172_elevation_test.xml`, `x153.xml`

---

## 三、朝向变更

### a) 通过航路点方位（自动计算）

```xml
<set name="guidance/target_wp_latitude_rad" value="..."/>
<set name="guidance/target_wp_longitude_rad" value="..."/>
<set name="ap/heading-setpoint-select" value="1"/>
```

### b) 直接指定航向

```xml
<set name="guidance/heading-selector-switch" value="1"/>
<set name="guidance/specified-heading-rad" value="1.75"/>
<set name="ap/heading_setpoint" value="200"/>
<set name="ap/heading_hold" value="1"/>
```

**涉及的脚本**: `c3104.xml`, `c172_cruise_8K.xml`, `c172_elevation_test.xml`

---

## 四、垂直机动（爬升/下降/高度捕获）

### 自动油门调，高度保持

```xml
<set name="ap/altitude_setpoint" value="1000.0"/>
<set name="ap/altitude_hold" value="1"/>
```

### 俯仰姿态控制

```xml
<set name="ap/pitch-hold" value="1"/>
<set name="ap/pitch-target-deg" value="-5.0"/>
```

### 直接升降舵指令

```xml
<set name="fcs/elevator-cmd-norm" action="FG_EXP" value="-0.5" tc="2.0"/>
```

**涉及的脚本**: `x153.xml`, `b171.xml`, `c1723.xml`

---

## 五、横滚/翻滚机动

### 自动滚转通道（`systems/afcs.xml`）

```
roll-err-ctrl → roll-rate-ctrl → roll-cmd-sum → aileron-pos-rad
```

### AP 滚转模式

```xml
<set name="ap/roll-attitude-mode" value="1"/>
<set name="ap/autopilot-roll-on" value="1"/>
```

### 直接副翼指令

```xml
<set name="fcs/aileron-cmd-norm" value="0.22" action="FG_RAMP" tc="20.0"/>
```

**涉及的脚本**: `ah1s_flight_test.xml`, `x151.xml`, `c1723.xml`

---

## 六、直升机特有机动

| 模式 | 属性 | 脚本 |
|---|---|---|
| AFCS 接通 | `ap/afcs/roll-channel-active-norm` | `sim_primer.xml` |
| AFCS 偏航通道 | `ap/afcs/yaw-channel-active-norm` | `ah1s_flight_test.xml` |
| AFCS 俯仰通道 | `ap/afcs/pitch-channel-active-norm` | `sim_primer.xml` |
| 拉平接地 | 手动组合 `fcs/elevator/rudder/aileron-cmd-norm` | `ah1s_flight_test.xml` |

**涉及的脚本**: `ah1s_flight_test.xml`, `sim_primer.xml`

---

## 七、地面操作

| 操作 | 属性 | 脚本 |
|---|---|---|
| 前轮转向 | `fcs/steer-cmd-norm` | `c1724.xml`, `B737_Runway.xml` |
| 差动刹车 | `fcs/{left,right}-brake-cmd-norm` | `b737_runway_new.xml` |
| 弹射起飞 | `systems/catapult.xml` | — |
| 尾钩/拦阻 | `systems/hook.xml` | — |

---

## 八、特殊情况

| 情况 | 属性 | 脚本 |
|---|---|---|
| 单发失效 → 顺桨 | `fcs/feather-cmd-norm` | `c3105.xml` |
| 侧风 | `atmosphere/wind-east-fps`, `wind-down-fps`, `aero/beta-rad` | `c172_cross_wind.xml` |
| 减速板 | `fcs/speedbrake-cmd-norm` → `speedbrake-pos-norm` | `f104/Systems/speedbrakes.xml` |
| 发动机启动 | `propulsion/starter_cmd`, `magneto_cmd` | 多个 |
| 自动油门 | `ap/throttle-cmd-norm` | `p51d/Systems/autothrottle.xml` |
| 襟翼 | `fcs/flap-cmd-norm` | 多个 |
| 制导/轨道 | `guidance/executive/*` (MET/远地点/偏心率/重力转弯) | `J2460.xml`, `J2461.xml` |
| 空中加油 | `systems/refuel.xml` | — |

---

## 九、未发现（需在 flight_dynamic 的 C++ 层实现）

| 机动 | 实现思路 |
|---|---|
| **8 字巡逻 / 矩形跑道航线** | C++ 中设定 4 个航路点构成回路，到达后自动切换下一航路点 |
| **蛇形机动** | C++ 中周期性交替改变 `guidance/specified-heading-rad`（如 ±20°） |
| **S 转弯** | 航向偏差量随时间递减的正弦波 |
| **横滚筋斗 / 桶滚** | 直接设定 `fcs/aileron-cmd-norm` 保持 T 秒后回中，配合俯仰 |
| **跃升 / 俯冲** | 设定俯仰目标角度或直接升降舵指令 |
| **ILS 进近 / 复飞** | 航路点 + 高度保持组合事件驱动 |
| **空速保持** | 油门 PID 闭环（`fcs/throttle-cmd-norm`） |
| **编队飞行** | 多实例 C++ 编排 |
