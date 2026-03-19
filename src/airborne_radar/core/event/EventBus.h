/**
 * @file EventBus.h
 * @brief 定义基于 eventpp 的事件总线实现。
 */

#ifndef AIRBORNE_RADAR_CORE_EVENT_EVENT_BUS_H_
#define AIRBORNE_RADAR_CORE_EVENT_EVENT_BUS_H_

#include <cstddef>
#include <typeindex>
#include <unordered_map>
#include <utility>

#include <eventpp/eventdispatcher.h>

#include "airborne_radar/core/event/IEventBus.h"

namespace airborne_radar { namespace core { namespace event {

/**
 * @brief 使用 eventpp 实现即时事件分发。
 */
class EventBus final : public IEventBus {
public:
	/**
	 * @brief 取消订阅。
	 * @param token 订阅句柄。
	 */
	void Unsubscribe(const EventToken &token) override;

	/**
	 * @brief 清空所有订阅。
	 */
	void Clear() override;

protected:
	/**
	 * @brief 类型擦除后的订阅实现。
	 * @param type 事件类型标识。
	 * @param handler 类型擦除后的回调。
	 * @return 订阅句柄。
	 */
	EventToken SubscribeImpl(
			std::type_index type,
			std::function<void(const EventPayload &)> handler) override;

	/**
	 * @brief 类型擦除后的发布实现。
	 * @param type 事件类型标识。
	 * @param payload 事件载荷指针。
	 */
	void PublishImpl(std::type_index type, EventPayload payload) override;

private:
	using Dispatcher =
			eventpp::EventDispatcher<std::type_index, void(const EventPayload &)>;
	using Handle = Dispatcher::Handle;

	/**
	 * @brief 保存单个订阅者的事件类型与回调句柄。
	 */
	struct ListenerRecord {
		ListenerRecord() = default;
		ListenerRecord(std::type_index t, Handle h)
				: type(t), handle(std::move(h)) {}

		std::type_index type{typeid(void)};
		Handle handle{};
	};

	Dispatcher dispatcher_;
	std::unordered_map<std::size_t, ListenerRecord> listeners_;
	std::size_t next_id_{0};
};

} } } // namespace airborne_radar::core::event

#endif // AIRBORNE_RADAR_CORE_EVENT_EVENT_BUS_H_
