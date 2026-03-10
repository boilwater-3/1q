// Copyright 2026. All Rights Reserved.
//
// Description: 基于 eventpp::EventQueue 的周期事件总线实现。

#ifndef AIRBORNE_RADAR_SRC_CORE_EVENT_CYCLE_EVENT_BUS_H_
#define AIRBORNE_RADAR_SRC_CORE_EVENT_CYCLE_EVENT_BUS_H_

#include <array>
#include <cstddef>
#include <typeindex>
#include <unordered_map>

#include <eventpp/eventqueue.h>

#include "1q/airborne_radar/core/event/IEventBus.h"

namespace airborne_radar { namespace core { namespace event {

/// @brief CycleEventBus 在周期边界消费事件，发布事件默认进入下一周期。
class CycleEventBus final : public IEventBus {
public:
	/// @brief 取消订阅。
	/// @param token 订阅句柄。
	void Unsubscribe(const EventToken &token) override;

	/// @brief 清空所有订阅。
	void Clear() override;

	/// @brief 周期开始：切换当前消费队列。
	void BeginCycle() override;

	/// @brief 处理当前周期队列中的事件。
	void DispatchCurrentCycle() override;

	/// @brief 周期结束：当前实现无额外行为，保留扩展点。
	void EndCycle() override;

protected:
	/// @brief 订阅实现（类型擦除）。
	EventToken SubscribeImpl(
			std::type_index type,
			std::function<void(const EventPayload &)> handler) override;

	/// @brief 发布实现（类型擦除）。
	/// 在周期总线中，发布也进入“下一周期队列”。
	void PublishImpl(std::type_index type, EventPayload payload) override;

	/// @brief 入队实现（类型擦除）。
	void EnqueueImpl(std::type_index type, EventPayload payload) override;

private:
	using Queue = eventpp::EventQueue<std::type_index, void(const EventPayload &)>;
	using Handle = Queue::Handle;

	struct ListenerRecord {
		ListenerRecord() = default;
		ListenerRecord(std::type_index t, Handle q0, Handle q1) 
		    : type(t), queue_zero_handle(q0), queue_one_handle(q1) {}

		std::type_index type{typeid(void)};
		Handle queue_zero_handle{};
		Handle queue_one_handle{};
	};

	std::size_t CurrentQueueIndex() const { return current_queue_index_; }
	std::size_t NextQueueIndex() const { return next_queue_index_; }

	std::array<Queue, 2> queues_;
	std::unordered_map<std::size_t, ListenerRecord> listeners_;
	std::size_t next_id_{0};
	std::size_t current_queue_index_{0};
	std::size_t next_queue_index_{1};
};

} } } // namespace airborne_radar::core::event

#endif // AIRBORNE_RADAR_SRC_CORE_EVENT_CYCLE_EVENT_BUS_H_
