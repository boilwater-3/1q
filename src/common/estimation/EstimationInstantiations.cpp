/**
 * @file EstimationInstantiations.cpp
 * @brief 生产维度（6×3 / 6×2）estimation 模板显式实例化 TU。
 *
 * 将 Kalman/EKF/UKF/IMM 中 Eigen LLT 等重模板的机器码收敛到本文件，
 * 其它 TU 通过头文件末尾 extern template 声明避免重复实例化。
 * 单测若使用其它维度（如 4×2）仍在测试 TU 内本地实例化。
 */

#include "common/estimation/EkfFilter.h"
#include "common/estimation/ImmFilter.h"
#include "common/estimation/KalmanPredictor.h"
#include "common/estimation/KalmanUpdater.h"
#include "common/estimation/UnscentedPredictor.h"
#include "common/estimation/UnscentedUpdater.h"

namespace oneq {
namespace common {
namespace estimation {

template class KalmanPredictor<6, 3>;
template class KalmanPredictor<6, 2>;
template class KalmanUpdater<6, 3>;
template class KalmanUpdater<6, 2>;

template class EkfPredictor<6, 3>;
template class EkfPredictor<6, 2>;
template class EkfUpdater<6, 3>;
template class EkfUpdater<6, 2>;

template class UnscentedPredictor<6, 3>;
template class UnscentedPredictor<6, 2>;
template class UnscentedUpdater<6, 3>;
template class UnscentedUpdater<6, 2>;

template class ImmFilter<6, 3>;
template class ImmFilter<6, 2>;

}  // namespace estimation
}  // namespace common
}  // namespace oneq
