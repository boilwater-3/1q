// Copyright 2026. All Rights Reserved.
//
// Description: CycleEventBus 的实现。

#include "1q/airborne_radar/core/event/CycleEventBus.h"

#include <utility>

namespace airborne_radar { namespace core { namespace event {

EventToken CycleEventBus::SubscribeImpl(
			std::type_index type,
			std::function<void(const EventPayload &)> handler) {
	const auto handle_zero = queues_[0].appendListener(type, handler);
	const auto handle_one = queues_[1].appendListener(type, handler);
	const std::size_t id = ++next_id_;
	listeners_.emplace(id, ListenerRecord{type, handle_zero, handle_one});
	return EventToken{type, id};
}

void CycleEventBus::PublishImpl(std::type_index type, EventPayload payload) {
	queues_[NextQueueIndex()].enqueue(type, std::move(payload));
}

void CycleEventBus::EnqueueImpl(std::type_index type, EventPayload payload) {
	queues_[NextQueueIndex()].enqueue(type, std::move(payload));
}

void CycleEventBus::Unsubscribe(const EventToken &token) {
	const auto it = listeners_.find(token.id);
	if (it == listeners_.end()) {
		return;
	}

	queues_[0].removeListener(it->second.type, it->second.queue_zero_handle);
	queues_[1].removeListener(it->second.type, it->second.queue_one_handle);
	listeners_.erase(it);
}

void CycleEventBus::Clear() {
	for (const auto &entry : listeners_) {
		queues_[0].removeListener(entry.second.type, entry.second.queue_zero_handle);
		queues_[1].removeListener(entry.second.type, entry.second.queue_one_handle);
	}
	listeners_.clear();
}

void CycleEventBus::BeginCycle() {
	std::swap(current_queue_index_, next_queue_index_);
}

void CycleEventBus::DispatchCurrentCycle() {
	queues_[CurrentQueueIndex()].process();
}

void CycleEventBus::EndCycle() {
	// 预留扩展点：可在此记录周期处理统计。
}

} } } // namespace airborne_radar::core::event
