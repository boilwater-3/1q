// Copyright 2026. All Rights Reserved.

/**
 * @file IEventBus.h
 * @brief 定义事件总线抽象接口。
 */

#ifndef AIRBORNE_RADAR_CORE_EVENT_I_EVENT_BUS_H_
#define AIRBORNE_RADAR_CORE_EVENT_I_EVENT_BUS_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <typeindex>

namespace airborne_radar { namespace core { namespace event {

/**
 * @brief 表示订阅的可撤销句柄。
 */
struct EventToken {
	EventToken() = default;
	EventToken(std::type_index t, std::size_t i) : type(t), id(i) {}

	std::type_index type{typeid(void)}; /**< 已订阅事件的类型标识。 */
	std::size_t id{0}; /**< 订阅记录的内部唯一编号。 */
};

/**
 * @brief 为跨层消息分发提供统一抽象。
 */
class IEventBus {
public:
	using EventPayload = std::shared_ptr<const void>;

	virtual ~IEventBus() = default;

	/**
	 * @brief 订阅指定事件类型。
	 * @tparam Event 事件类型。
	 * @param handler 事件回调。
	 * @return 订阅句柄。
	 */
	template <typename Event>
	EventToken Subscribe(const std::function<void(const Event &)> &handler) {
		return SubscribeImpl(
				typeid(Event),
				[handler](const EventPayload &payload) {
					handler(*std::static_pointer_cast<const Event>(payload));
				});
	}

	/**
	 * @brief 发布指定事件。
	 * @tparam Event 事件类型。
	 * @param event 事件实例。
	 */
	template <typename Event>
	void Publish(const Event &event) {
		PublishImpl(typeid(Event), std::make_shared<Event>(event));
	}

	/**
	 * @brief 将事件放入下一周期队列。
	 * @tparam Event 事件类型。
	 * @param event 事件实例。
	 */
	template <typename Event>
	void Enqueue(const Event &event) {
		EnqueueImpl(typeid(Event), std::make_shared<Event>(event));
	}

	/**
	 * @brief 取消订阅。
	 * @param token 订阅句柄。
	 */
	virtual void Unsubscribe(const EventToken &token) = 0;

	/**
	 * @brief 清空所有订阅。
	 */
	virtual void Clear() = 0;

	/**
	 * @brief 周期开始时切换当前消费队列。
	 * @note 对即时分发总线可保持默认空实现。
	 */
	virtual void BeginCycle() {}

	/**
	 * @brief 消费当前周期队列中的事件。
	 * @note 对即时分发总线可保持默认空实现。
	 */
	virtual void DispatchCurrentCycle() {}

	/**
	 * @brief 周期结束时补充统计或状态收尾。
	 * @note 对即时分发总线可保持默认空实现。
	 */
	virtual void EndCycle() {}

protected:
	/**
	 * @brief 类型擦除后的订阅实现。
	 * @param type 事件类型标识。
	 * @param handler 类型擦除后的回调。
	 * @return 订阅句柄。
	 */
	virtual EventToken SubscribeImpl(
			std::type_index type,
			std::function<void(const EventPayload &)> handler) = 0;

	/**
	 * @brief 类型擦除后的发布实现。
	 * @param type 事件类型标识。
	 * @param payload 事件载荷指针。
	 */
	virtual void PublishImpl(std::type_index type, EventPayload payload) = 0;

	/**
	 * @brief 类型擦除后的入队实现。
	 * @param type 事件类型标识。
	 * @param payload 事件载荷指针。
	 * @note 默认行为退化为即时发布，以保持旧实现兼容。
	 */
	virtual void EnqueueImpl(std::type_index type, EventPayload payload) {
		PublishImpl(type, std::move(payload));
	}
};

} } } // namespace airborne_radar::core::event

#endif // AIRBORNE_RADAR_CORE_EVENT_I_EVENT_BUS_H_
