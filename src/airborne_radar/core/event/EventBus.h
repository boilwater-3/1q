// Copyright 2026. All Rights Reserved.
//
// Description: 基于 eventpp 的事件总线实现。

#ifndef AIRBORNE_RADAR_SRC_CORE_EVENT_EVENT_BUS_H_
#define AIRBORNE_RADAR_SRC_CORE_EVENT_EVENT_BUS_H_

#include <cstddef>
#include <typeindex>
#include <unordered_map>

#include <eventpp/eventdispatcher.h>

#include "1q/airborne_radar/core/event/IEventBus.h"

namespace airborne_radar::core::event {

/// @brief EventBus 使用 eventpp 实现事件分发。
class EventBus final : public IEventBus {
public:
	/// @brief 取消订阅。
	/// @param token 订阅句柄。
	void Unsubscribe(const EventToken &token) override;

	/// @brief 清空所有订阅。
	void Clear() override;

protected:
	/// @brief 订阅实现（类型擦除）。
	EventToken SubscribeImpl(
			std::type_index type,
			std::function<void(const EventPayload &)> handler) override;

	/// @brief 发布实现（类型擦除）。
	void PublishImpl(std::type_index type, EventPayload payload) override;

private:
	using Dispatcher =
			eventpp::EventDispatcher<std::type_index, void(const EventPayload &)>;
	using Handle = Dispatcher::Handle;

	struct ListenerRecord {
		std::type_index type{typeid(void)};
		Handle handle{};
	};

	Dispatcher dispatcher_;
	std::unordered_map<std::size_t, ListenerRecord> listeners_;
	std::size_t next_id_{0};
};

} // namespace airborne_radar::core::event

#endif // AIRBORNE_RADAR_SRC_CORE_EVENT_EVENT_BUS_H_
