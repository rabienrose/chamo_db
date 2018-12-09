#ifndef ROVIOLI_DATA_PUBLISHER_FLOW_H_
#define ROVIOLI_DATA_PUBLISHER_FLOW_H_

#include <memory>
#include <string>

#include <maplab-common/conversions.h>
#include <maplab-common/timeout-counter.h>
#include <message-flow/message-flow.h>

#include "rovioli/flow-topics.h"

namespace rovioli {

class DataPublisherFlow {
 public:

  DataPublisherFlow();

  void attachToMessageFlow(message_flow::MessageFlow* flow);
  void visualizeMap(const vi_map::VIMap& vi_map) const;

 private:
  void registerPublishers();
  void stateCallback(
      int64_t timestamp_ns, const vio::ViNodeState& vinode,
      const bool has_T_G_M, const aslam::Transformation& T_G_M);
  void stateDebugCallback(
      const vio::ViNodeState& vinode, const bool has_T_G_M,
      const aslam::Transformation& T_G_M);
  void localizationCallback(const Eigen::Vector3d& p_G_I_lc_pnp);

  common::TimeoutCounter map_publisher_timeout_;

};

}  //  namespace rovioli

#endif  // ROVIOLI_DATA_PUBLISHER_FLOW_H_
