/**
 * @file AtmospherePhysics.h
 * @brief 定义共享的大气传播物理近似模型与 REOS 对齐函数入口。
 */

#ifndef COMMON_ATMOSPHERE_ATMOSPHERE_PHYSICS_H_
#define COMMON_ATMOSPHERE_ATMOSPHERE_PHYSICS_H_

namespace oneq {
namespace common {
namespace atmosphere {

constexpr float kDefaultEffectiveEarthRadiusFactor = 4.0f / 3.0f;

/**
 * @brief GTD7 近似输出。
 */
struct Gtd7Profile {
  double temperature_k{288.15};  /**< 近似温度（单位：K） */
  double density_kg_m3{1.225};   /**< 近似中性大气密度（单位：kg/m^3） */
};

/**
 * @brief 大气传播物理模型输入。
 */
struct AtmosphericPropagationInputs {
  bool enable_physics{false};        /**< 是否启用物理项 */
  float frequency_hz{10.0e9f};       /**< 雷达频率（单位：Hz） */
  float path_length_m{10.0e3f};      /**< 传播路径长度（单位：m） */
  float radar_altitude_m{1.0e3f};    /**< 雷达高度（单位：m） */
  float target_altitude_m{1.0e3f};   /**< 目标高度（单位：m） */
  float elevation_deg{5.0f};         /**< 传播仰角（单位：deg） */
  float pressure_hpa{1013.25f};      /**< 大气压（单位：hPa） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float relative_humidity{0.5f};     /**< 相对湿度 [0, 1] */
  float k_factor{
      kDefaultEffectiveEarthRadiusFactor}; /**< 地球有效半径因子 */
  int day_of_year{172};              /**< 年积日 */
  float solar_flux_f107a{150.0f};    /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};     /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};        /**< 地磁活动指数 */
};

/**
 * @brief 大气传播物理模型输出。
 */
struct AtmosphericPropagationResult {
  float blake_loss_db{0.0f};            /**< Blake 传播损耗（单位：dB） */
  float refractivity_index{1.0003f};    /**< 折射率 n */
  float refractivity_index_h{1.0003f};  /**< 高度修正折射率 n(h) */
  float neutral_density_kg_m3{1.225f};  /**< 中性大气密度（单位：kg/m^3） */
  float total_physics_loss_db{0.0f};    /**< 聚合物理附加损耗（单位：dB） */
};

/**
 * @brief REOS 对齐入口：Blake 大气损耗（单精度）。
 * @param[in] h_a_m 雷达高度（单位：m）。
 * @param[in] f_hz 雷达频率（单位：Hz）。
 * @param[in] theta_deg 传播仰角（单位：deg）。
 * @param[in] r_m 传播距离（单位：m）。
 * @param[in] k 有效地球半径因子。
 * @return Blake 双程大气损耗（单位：dB）。
 */
float blake_atmos_loss_r4_1(float h_a_m, float f_hz, float theta_deg, float r_m, float k);

/**
 * @brief REOS 对齐入口：Blake 大气损耗（双精度）。
 * @param[in] h_a_m 雷达高度（单位：m）。
 * @param[in] f_hz 雷达频率（单位：Hz）。
 * @param[in] theta_deg 传播仰角（单位：deg）。
 * @param[in] r_m 传播距离（单位：m）。
 * @param[in] k 有效地球半径因子。
 * @return Blake 双程大气损耗（单位：dB）。
 */
double blake_atmos_loss_r8_1(double h_a_m, double f_hz, double theta_deg, double r_m, double k);

/**
 * @brief REOS 对齐入口：折射率 n（单精度）。
 * @param[in] tc_celsius 摄氏温度（单位：°C）。
 * @param[in] tk_kelvin 开氏温度（单位：K）。
 * @param[in] pd_hpa 水汽分压（单位：hPa）。
 * @param[in] p_hpa 总气压（单位：hPa）。
 * @param[in] h_rel 相对湿度 [0, 1]。
 * @param[in] water_or_ice 介质标志（水/冰），沿用 REOS 约定。
 * @return 大气折射率 n。
 */
float refractivity_index_n_r4(float tc_celsius, float tk_kelvin, float pd_hpa, float p_hpa,
                              float h_rel, int water_or_ice);

/**
 * @brief REOS 对齐入口：折射率 n（双精度）。
 * @param[in] tc_celsius 摄氏温度（单位：°C）。
 * @param[in] tk_kelvin 开氏温度（单位：K）。
 * @param[in] pd_hpa 水汽分压（单位：hPa）。
 * @param[in] p_hpa 总气压（单位：hPa）。
 * @param[in] h_rel 相对湿度 [0, 1]。
 * @param[in] water_or_ice 介质标志（水/冰），沿用 REOS 约定。
 * @return 大气折射率 n。
 */
double refractivity_index_n_r8(double tc_celsius, double tk_kelvin, double pd_hpa, double p_hpa,
                               double h_rel, int water_or_ice);

/**
 * @brief REOS 对齐入口：高度修正折射率 n(h)（单精度）。
 * @param[in] n0_index 参考点折射率 n0。
 * @param[in] h_m 目标高度（单位：m）。
 * @param[in] h0_m 参考点高度（单位：m）。
 * @return 高度修正后的折射率 n(h)。
 */
float refractivity_index_nh_r4(float n0_index, float h_m, float h0_m);

/**
 * @brief REOS 对齐入口：MSISE00 GTD7 近似。
 * @param[in] day_of_year 年积日。
 * @param[in] sec 当日秒数（UT，单位：s）。
 * @param[in] alt_m 高度（单位：m）。
 * @param[in] glat_deg 地理纬度（单位：deg）。
 * @param[in] glong_deg 地理经度（单位：deg）。
 * @param[in] stl 地方太阳时（单位：h）。
 * @param[in] f107a 平滑太阳流量指数。
 * @param[in] f107 当日太阳流量指数。
 * @param[in] ap 地磁活动指数。
 * @param[in] mass 物种质量标志（MSISE 约定，48=总密度）。
 * @return GTD7 近似的温度与中性密度。
 */
Gtd7Profile GTD7(int day_of_year, double sec, double alt_m, double glat_deg, double glong_deg,
                 double stl, double f107a, double f107, double ap, int mass);

/**
 * @brief 计算传播路径的物理附加损耗。
 * @param[in] inputs 传播物理模型输入。
 * @return 传播物理模型输出（Blake 损耗、折射率、密度、聚合损耗）。
 */
AtmosphericPropagationResult EvaluateAtmosphericPropagation(
    const AtmosphericPropagationInputs& inputs);

/**
 * @brief 大气观测 + 空间天气组合的引用式输入（用于 BuildPropagationInputs）。
 */
struct AtmosphericObservationRef {
  float pressure_hpa{1013.25f};      /**< 大气压（单位：hPa） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float relative_humidity{0.5f};     /**< 相对湿度 [0, 1] */
  float k_factor{kDefaultEffectiveEarthRadiusFactor}; /**< 地球有效半径因子 */
  int day_of_year{172};              /**< 年积日 */
  float solar_flux_f107a{150.0f};    /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};     /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};        /**< 地磁活动指数 */
};

/**
 * @brief 从大气状态和传感器参数构建 AtmosphericPropagationInputs。
 *
 * 消除各模块手工填充字段的重复代码。
 * @param[in] frequency_hz 频率（单位：Hz）。
 * @param[in] path_length_m 路径长度（单位：m）。
 * @param[in] radar_altitude_m 雷达高度（单位：m）。
 * @param[in] target_altitude_m 目标高度（单位：m）。
 * @param[in] elevation_deg 仰角（单位：deg）。
 * @param[in] observation 大气观测 + 空间天气组合输入。
 * @return 构造好的 AtmosphericPropagationInputs。
 */
AtmosphericPropagationInputs BuildPropagationInputs(
    float frequency_hz, float path_length_m, float radar_altitude_m, float target_altitude_m,
    float elevation_deg, const AtmosphericObservationRef& observation);

/**
 * @brief 逐目标大气物理附加损耗（dB）的标量胶水。
 *
 * 封装 BuildPropagationInputs + EvaluateAtmosphericPropagation；调用方负责
 * enable 门控与 k_factor / 观测字段填充。has_look_angles=false 时仰角按 0 deg。
 */
float ComputeTargetAtmosphericPhysicsLossDb(float frequency_hz, float slant_range_m,
                                            float radar_altitude_m, float target_altitude_m,
                                            float elevation_deg, bool has_look_angles,
                                            const AtmosphericObservationRef& observation);

}  // namespace atmosphere
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ATMOSPHERE_ATMOSPHERE_PHYSICS_H_
