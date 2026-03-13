// Copyright 2026. All Rights Reserved.
//
// Description: RadarController 的核心调度实现。

#include "1q/airborne_radar/core/controller/RadarController.h"

#include <algorithm>

#include <Eigen/Core>
#include <spdlog/spdlog.h>

#include "1q/airborne_radar/core/context/DecisionContext.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/event/IEventBus.h"
#include "1q/airborne_radar/core/event/RadarEvents.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalProcessor.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"

namespace airborne_radar {
namespace core {
namespace controller {

RadarController::RadarController(
		core::context::IRadarContext &radar_context,
		signal::pipeline::ISignalPipeline &signal_pipeline,
		decision::pipeline::ITacticalProcessor &decision_pipeline,
		environment::IEnvironmentService &environment_service)
		: radar_context_(radar_context), signal_pipeline_(signal_pipeline),
			decision_pipeline_(decision_pipeline),
			environment_service_(environment_service) {}

RadarController::RadarController(
		core::context::IRadarContext &radar_context,
		signal::pipeline::ISignalPipeline &signal_pipeline,
		decision::pipeline::ITacticalProcessor &decision_pipeline,
		environment::IEnvironmentService &environment_service,
		core::event::IEventBus &event_bus)
		: radar_context_(radar_context), signal_pipeline_(signal_pipeline),
			decision_pipeline_(decision_pipeline),
			environment_service_(environment_service), event_bus_(&event_bus) {}

void RadarController::RunOnce() {
	if (event_bus_ != nullptr) {
		// 周期开始：先处理上一周期沉淀到当前队列的事件。
		event_bus_->BeginCycle();
		event_bus_->DispatchCurrentCycle();
	}

	const common::TargetFeatureList input_features =
			radar_context_.GetTargetFeatures();
	std::size_t association_seed_count = 0;
	if (track_lifecycle_manager_ != nullptr) {
		const std::vector<signal::tracking::AssociationTrackSeed> seeds =
				track_lifecycle_manager_->BuildAssociationSeeds();
		association_seed_count = seeds.size();
		signal_pipeline_.SetAssociationSeeds(seeds);
	} else {
		signal_pipeline_.ResetAssociationSeedModeToFallbackHistory();
	}
	const common::TargetFeatureList updated_features =
			signal_pipeline_.RunCycle(input_features, environment_service_);
	const signal::pipeline::AssociationQualityMetrics association_metrics =
			signal_pipeline_.GetLastAssociationQualityMetrics();
	common::TargetFeatureList decision_features = updated_features;
	std::size_t measurement_count = 0;

	if (track_lifecycle_manager_ != nullptr) {
		const std::vector<signal::tracking::TrackMeasurement> measurements =
				signal_pipeline_.GetLastTrackMeasurements();
		measurement_count = measurements.size();
		signal::tracking::CycleContext cycle;
		cycle.cycle_index = cycle_index_;
		cycle.batch_id = batch_id_;
		track_lifecycle_manager_->Update(cycle, measurements);
		decision_features = track_lifecycle_manager_->BuildFeatureSnapshot();
	}

	core::context::DecisionContext context(decision_features);
	decision_pipeline_.ProcessTactics(context);
	ExecuteCommands(context);

	const bool jamming_detected = std::any_of(
			decision_features.begin(), decision_features.end(),
			[](const common::TargetFeature &feature) {
				return feature.check_jamming_detected;
			});

	if (event_bus_ != nullptr) {
		core::event::TracksUpdatedEvent tracks_event;
		tracks_event.state = decision_features;
		event_bus_->Enqueue(tracks_event);

		if (jamming_detected) {
			core::event::JammingAlertEvent jamming_event;
			jamming_event.detected = true;
			event_bus_->Enqueue(jamming_event);
		}

		core::event::CommandsSubmittedEvent commands_event;
		commands_event.command_count = context.decision_commands.size();
		event_bus_->Enqueue(commands_event);

		core::event::RadarCycleCompletedEvent event;
		event.command_count = context.decision_commands.size();
		event.jamming_detected = jamming_detected;
		event_bus_->Enqueue(event);

		// 周期结束：本周期发布的事件将留待下一周期处理。
		event_bus_->EndCycle();
	}

	spdlog::debug(
			"[RadarController] cycle summary: cycle_index={} batch_id={} input_targets={} association_seeds={} measurements={} decision_features={} commands={} lifecycle_enabled={} jamming_detected={} assoc_priors={} assoc_detections={} assoc_matches={} assoc_new_tracks={} assoc_missed_tracks={} assoc_match_rate={:.3f} assoc_new_track_rate={:.3f} assoc_missed_rate={:.3f} assoc_mean_cost={:.3f} assoc_p95_cost={:.3f}",
			cycle_index_, batch_id_, input_features.size(), association_seed_count,
			measurement_count, decision_features.size(),
			context.decision_commands.size(),
			track_lifecycle_manager_ != nullptr ? "true" : "false",
			jamming_detected ? "true" : "false",
			association_metrics.prior_track_count,
			association_metrics.detection_count,
			association_metrics.matched_count,
			association_metrics.new_track_count,
			association_metrics.missed_track_count,
			association_metrics.match_rate,
			association_metrics.new_track_rate,
			association_metrics.missed_track_rate,
			association_metrics.mean_match_cost,
			association_metrics.p95_match_cost);

	++cycle_index_;
	++batch_id_;
}

void RadarController::RunCycles(std::size_t cycles) {
	for (std::size_t i = 0; i < cycles; ++i) {
		RunOnce();
	}
}

void RadarController::ExecuteCommands(
		const core::context::DecisionContext &context) {
	for (const auto &cmd : context.decision_commands) {
		radar_context_.SubmitControlCommand(cmd);
	}
}

void RadarController::SetEventBus(core::event::IEventBus *event_bus) {
	event_bus_ = event_bus;
}

void RadarController::SetTrackLifecycleManager(
		signal::tracking::ITrackLifecycleManager *lifecycle_manager) {
	track_lifecycle_manager_ = lifecycle_manager;
	signal_pipeline_.SetInternalAssociationHistoryFallbackEnabled(
			lifecycle_manager == nullptr);
	spdlog::info(
			"[RadarController] track lifecycle manager {}",
			lifecycle_manager != nullptr ? "attached" : "detached");
}

} // namespace controller
} // namespace core
} // namespace airborne_radar
