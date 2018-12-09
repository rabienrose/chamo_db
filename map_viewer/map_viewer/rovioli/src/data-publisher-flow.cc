#include "rovioli/data-publisher-flow.h"

#include <maplab-common/file-logger.h>

DEFINE_double(
    map_publish_interval_s, 2.0,
    "Interval of publishing the visual-inertial map to ROS [seconds].");

DEFINE_bool(
    publish_only_on_keyframes, false,
    "Publish frames only on keyframes instead of the IMU measurements. This "
    "means a lower frequency.");

DEFINE_bool(
    publish_debug_markers, false,
    "Publish debug sphere markers for T_M_I, T_G_I and localization frames.");

DEFINE_string(
    export_estimated_poses_to_csv, "",
    "If not empty, the map builder will export the estimated poses to a CSV "
    "file.");

DEFINE_bool(
    rovioli_visualize_map, true,
    "Set to false to disable map visualization. Note: map building needs to be "
    "active for the visualization.");

DECLARE_bool(rovioli_run_map_builder);

namespace rovioli {

DataPublisherFlow::DataPublisherFlow()
    : map_publisher_timeout_(
          common::TimeoutCounter(
              FLAGS_map_publish_interval_s * kSecondsToNanoSeconds)) {
}

void DataPublisherFlow::registerPublishers() {
}

void DataPublisherFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  registerPublishers();
  static constexpr char kSubscriberNodeName[] = "DataPublisherFlow";

  if (FLAGS_rovioli_run_map_builder && FLAGS_rovioli_visualize_map) {
    flow->registerSubscriber<message_flow_topics::RAW_VIMAP>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const VIMapWithMutex::ConstPtr& map_with_mutex) {
          if (map_publisher_timeout_.reached()) {
            std::lock_guard<std::mutex> lock(map_with_mutex->mutex);
            visualizeMap(map_with_mutex->vi_map);
            map_publisher_timeout_.reset();
          }
        });
  }

  // Publish localization results.
  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [&](const vio::LocalizationResult::ConstPtr& localization) {
        CHECK(localization != nullptr);
        localizationCallback(localization->T_G_I_lc_pnp.getPosition());
      });

  flow->registerSubscriber<message_flow_topics::ROVIO_ESTIMATES>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [&](const RovioEstimate::ConstPtr& state) {
        CHECK(state != nullptr);
        if (!FLAGS_publish_only_on_keyframes) {
          stateCallback(
              state->timestamp_s * kSecondsToNanoSeconds, state->vinode,
              state->has_T_G_M, state->T_G_M);
        }
      });

  flow->registerSubscriber<message_flow_topics::VIO_UPDATES>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const vio::VioUpdate::ConstPtr& vio_update) {
        CHECK(vio_update != nullptr);
        if (FLAGS_publish_only_on_keyframes) {
          bool has_T_G_M =
              (vio_update->localization_state ==
                   vio::LocalizationState::kLocalized ||
               vio_update->localization_state ==
                   vio::LocalizationState::kMapTracking);

          stateCallback(
              vio_update->timestamp_ns, vio_update->vinode, has_T_G_M,
              vio_update->T_G_M);
        }
      });
}

void DataPublisherFlow::visualizeMap(const vi_map::VIMap& vi_map) const {

}

void DataPublisherFlow::stateCallback(
    int64_t timestamp_ns, const vio::ViNodeState& vinode, const bool has_T_G_M,
    const aslam::Transformation& T_G_M) {
}

void DataPublisherFlow::stateDebugCallback(
    const vio::ViNodeState& vinode, const bool has_T_G_M,
    const aslam::Transformation& T_G_M) {

}

void DataPublisherFlow::localizationCallback(
    const Eigen::Vector3d& p_G_I_lc_pnp) {

}

}  //  namespace rovioli
