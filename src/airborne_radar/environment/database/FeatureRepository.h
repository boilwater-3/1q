/**
 * @file FeatureRepository.h
 * @brief 定义目标特征仓储的具体实现。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_DATABASE_FEATURE_REPOSITORY_H_
#define AIRBORNE_RADAR_ENVIRONMENT_DATABASE_FEATURE_REPOSITORY_H_

#include <string>
#include <vector>

#include "airborne_radar/environment/database/IFeatureRepository.h"

namespace airborne_radar { namespace environment { namespace database {

/**
 * @brief RawFeatureRow 表示外部数据库拉取后的原始记录。
 */
struct RawFeatureRow {
  std::string target_type; /**< 目标类型标签，例如 `HIGH_THREAT_FIGHTER`。 */
  float speed{0.0f}; /**< 速度特征，单位为 `m/s`。 */
  float rcs{0.0f}; /**< 雷达散射截面积特征，单位为 `m^2`。 */
  bool jamming_expected{false}; /**< 是否预期存在干扰态势。 */
  float sigma_speed{120.0f}; /**< 速度维度归一化尺度 `sigma`。 */
  float sigma_rcs{1.2f}; /**< RCS 维度归一化尺度 `sigma`。 */
  float prior{1.0f}; /**< 该类目标的先验权重。 */
};

/**
 * @brief FeatureRecord 表示加载后的内存仓储记录。
 */
struct FeatureRecord {
  FeatureRecord() = default;

  /**
   * @brief 构造内存仓储记录。
   * @param type 目标类型标签。
   * @param proto 原型特征向量。
   * @param ss 速度维度归一化尺度。
   * @param sr RCS 维度归一化尺度。
   * @param p 先验权重。
   */
  FeatureRecord(std::string type, FeatureVector proto,
                float ss, float sr, float p)
      : target_type(std::move(type)),
        prototype(std::move(proto)),
        sigma_speed(ss),
        sigma_rcs(sr),
        prior(p) {}

  std::string target_type; /**< 目标类型标签。 */
  FeatureVector prototype{}; /**< 原型特征向量。 */
  float sigma_speed{120.0f}; /**< 速度维度归一化尺度。 */
  float sigma_rcs{1.2f}; /**< RCS 维度归一化尺度。 */
  float prior{1.0f}; /**< 先验权重。 */
};

/**
 * @brief 提供特征仓储的内存查询能力。
 */
class FeatureRepository final : public IFeatureRepository {
 public:
  /**
   * @brief 构造函数。
   * @note 初始化内存仓储为默认特征库，保证在未连接外部数据源时也可查询。
   */
  FeatureRepository();

  /**
   * @brief 记录外部数据源连接串。
   * @param connection_string 外部数据库连接串。
   * @return 连接串非空时返回 `true`。
   * @note 当前仅保存连接参数，不执行真实连通性校验。
   */
  bool ConnectDataSource(const std::string &connection_string) override;

  /**
   * @brief 从外部数据源拉取并刷新内存仓储。
   * @return 拉取和转换均成功时返回 `true`；失败时保持现有 `records_` 不变。
   */
  bool ReloadFromDataSource() override;

  /**
   * @brief 在内存仓储中查询输入特征的最优匹配。
   * @param input 输入特征向量。
   * @param[out] result 输出最优匹配类型、归一化概率和距离。
   * @return 查询成功时返回 `true`；当仓储为空或得分无效时返回 `false`。
   */
  bool QueryBestMatch(const FeatureVector &input,
                      MatchResult &result) const override;

 private:
  /**
   * @brief 从外部数据源抓取原始特征数据。
   * @param[out] rows 输出抓取到的原始记录集合。
   * @return 抓取成功时返回 `true`。
   */
  bool FetchRawRowsFromDataSource(std::vector<RawFeatureRow> &rows) const;

  /**
   * @brief 将原始行转换为内存仓储记录。
   * @param rows 外部数据源返回的原始记录。
   * @return 可直接用于查询的内存记录集合。
   * @note 会对 `sigma`/`prior` 等字段做最小值保护，避免异常参数破坏匹配稳定性。
   */
  static std::vector<FeatureRecord> TransformRowsToRecords(
      const std::vector<RawFeatureRow> &rows);

  /**
   * @brief 在未连接外部数据源时加载默认特征库。
   * @return 内置默认记录集合。
   */
  static std::vector<FeatureRecord> BuildDefaultRecords();

  /**
   * @brief 计算输入与仓储记录的归一化距离。
   * @param input 输入特征向量。
   * @param record 仓储中的单条原型记录。
   * @return 数值越小表示越相似。
   * @note 已知维度（`speed`/`rcs`/`jamming`）按固定权重计算；额外维度按差值累加。
   */
  static float ComputeDistance(const FeatureVector &input,
                               const FeatureRecord &record);

  /**
   * @brief 根据距离计算匹配分值。
   * @param distance 特征空间距离。
   * @return 分值范围 `(0, 1]`，距离越小分值越高。
   */
  static float DistanceToScore(float distance);

  std::string connection_string_;
  std::vector<FeatureRecord> records_;
};

} } } // namespace airborne_radar::environment::database

#endif // AIRBORNE_RADAR_ENVIRONMENT_DATABASE_FEATURE_REPOSITORY_H_
