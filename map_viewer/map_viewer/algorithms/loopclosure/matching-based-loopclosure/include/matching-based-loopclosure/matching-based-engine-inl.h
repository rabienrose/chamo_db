#ifndef MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_INL_H_
#define MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_INL_H_

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <aslam/common/reader-writer-lock.h>
#include <vi-map/unique-id.h>

//#include "matching-based-loopclosure/matching-based-engine.h"

namespace std {
template <>
struct hash<vi_map::FrameKeyPointToStructureMatch> {
  std::size_t operator()(
      const vi_map::FrameKeyPointToStructureMatch& value) const {
    const std::size_t h0(
        std::hash<vi_map::KeypointIdentifier>()(value.keypoint_id_query));
    const std::size_t h1(
        std::hash<vi_map::VisualFrameIdentifier>()(value.keyframe_id_result));
    const std::size_t h2(
        std::hash<vi_map::LandmarkId>()(value.landmark_result));
    return h0 ^ h1 ^ h2;
  }
};

template <>
struct hash<std::pair<vi_map::KeypointIdentifier, vi_map::LandmarkId>> {
  std::size_t operator()(
      const std::pair<vi_map::KeypointIdentifier, vi_map::LandmarkId>& value)
      const {
    const std::size_t h1(std::hash<vi_map::KeypointIdentifier>()(value.first));
    const std::size_t h2(std::hash<vi_map::LandmarkId>()(value.second));
    return h1 ^ h2;
  }
};
}  // namespace std

namespace matching_based_loopclosure {

}  // namespace matching_based_loopclosure

#endif  // MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_INL_H_
