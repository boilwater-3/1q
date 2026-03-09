// Copyright 2026. All Rights Reserved.
//
// Description: EventBus 的实现。

#include "airborne_radar/core/event/EventBus.h"

namespace airborne_radar::core::event {

EventToken EventBus::SubscribeImpl(std::type_index type,
											std::function<void(const EventPayload &)> handler) {
	const auto handle = dispatcher_.appendListener(type, std::move(handler));
	const std::size_t id = ++next_id_;
	listeners_.emplace(id, ListenerRecord{type, handle});
	return EventToken{type, id};
}

void EventBus::PublishImpl(std::type_index type, EventPayload payload) {
	dispatcher_.dispatch(type, payload);
}

void EventBus::Unsubscribe(const EventToken &token) {
	const auto it = listeners_.find(token.id);
	if (it == listeners_.end()) {
		return;
	}

	dispatcher_.removeListener(it->second.type, it->second.handle);
	listeners_.erase(it);
}

void EventBus::Clear() {
	for (const auto &entry : listeners_) {
		dispatcher_.removeListener(entry.second.type, entry.second.handle);
	}
	listeners_.clear();
}

} // namespace airborne_radar::core::event
