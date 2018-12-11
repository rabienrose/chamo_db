#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <descriptor-projection/descriptor-projection.h>
#include <loopclosure-common/types.h>
#include <maplab-common/conversions.h>
#include <maplab-common/parallel-process.h>
#include <nabo/nabo.h>
#include <vi-map/loop-constraint.h>

#include "matching-based-loopclosure/detector-settings.h"
#include "matching-based-loopclosure/helpers.h"
#include "matching-based-loopclosure/inverted-index-interface.h"
#include "matching-based-loopclosure/inverted-multi-index-interface.h"
#include "matching-based-loopclosure/kd-tree-index-interface.h"
#include "matching-based-loopclosure/matching-based-engine.h"
#include "matching-based-loopclosure/scoring.h"
DECLARE_bool(summary_time);

namespace matching_based_loopclosure {
    
    
    template <typename IdType>
    void MatchingBasedLoopDetector::doCovisibilityFiltering(
                                                            const loop_closure::IdToMatches<IdType>& id_to_matches_map,
                                                            const bool make_matches_unique,
                                                            loop_closure::FrameToMatches* frame_matches_ptr,
                                                            std::mutex* frame_matches_mutex) const {
        // WARNING: Do not clear frame matches. It is intended that new matches can
        // be added to already existing matches. The mutex passed to the function
        // can be nullptr, in which case locking is disabled.
        CHECK_NOTNULL(frame_matches_ptr);
        loop_closure::FrameToMatches& frame_matches = *frame_matches_ptr;
        
        typedef int ComponentId;
        constexpr ComponentId kInvalidComponentId = -1;
        typedef std::unordered_map<loop_closure::Match, ComponentId>
        MatchesToComponents;
        typedef std::unordered_map<ComponentId,
        std::unordered_set<loop_closure::Match>>
        Components;
        typedef std::unordered_map<vi_map::LandmarkId,
        std::vector<loop_closure::Match>>
        LandmarkMatches;
        
        const size_t num_matches_to_filter =
        loop_closure::getNumberOfMatches(id_to_matches_map);
        if (num_matches_to_filter == 0u) {
            return;
        }
        
        MatchesToComponents matches_to_components;
        LandmarkMatches landmark_matches;
        // To avoid rehashing, we reserve at least twice the number of elements.
        matches_to_components.reserve(num_matches_to_filter * 2u);
        // Reserving the number of matches is still conservative because the number
        // of matched landmarks is smaller than the number of matches.
        landmark_matches.reserve(num_matches_to_filter);
        
        for (const typename loop_closure::IdToMatches<IdType>::value_type&
             id_matches_pair : id_to_matches_map) {
            for (const loop_closure::Match& match : id_matches_pair.second) {
                landmark_matches[match.landmark_result].emplace_back(match);
                matches_to_components.emplace(match, kInvalidComponentId);
            }
        }
        
        IdToScoreMap<IdType> id_to_score_map;
        computeRelevantIdsForFiltering(id_to_matches_map, &id_to_score_map);
        
        ComponentId count_component_index = 0;
        size_t max_component_size = 0u;
        ComponentId max_component_id = kInvalidComponentId;
        Components components;
        for (const MatchesToComponents::value_type& match_to_component :
             matches_to_components) {
            if (match_to_component.second != kInvalidComponentId)
                continue;
            ComponentId component_id = count_component_index++;
            
            // Find the largest set of keyframes connected by landmark covisibility.
            std::queue<loop_closure::Match> exploration_queue;
            exploration_queue.push(match_to_component.first);
            while (!exploration_queue.empty()) {
                const loop_closure::Match& exploration_match = exploration_queue.front();
                
                if (skipMatch(id_to_score_map, exploration_match)) {
                    exploration_queue.pop();
                    continue;
                }
                
                const MatchesToComponents::iterator exploration_match_and_component =
                matches_to_components.find(exploration_match);
                CHECK(exploration_match_and_component != matches_to_components.end());
                
                if (exploration_match_and_component->second == kInvalidComponentId) {
                    // Not part of a connected component.
                    exploration_match_and_component->second = component_id;
                    components[component_id].insert(exploration_match);
                    // Mark all observations (which are matches) from this ID (keyframe or
                    // vertex) as visited.
                    const typename loop_closure::IdToMatches<IdType>::const_iterator
                    id_and_matches =
                    getIteratorForMatch(id_to_matches_map, exploration_match);
                    CHECK(id_and_matches != id_to_matches_map.cend());
                    const std::vector<loop_closure::Match>& id_matches =
                    id_and_matches->second;
                    for (const loop_closure::Match& id_match : id_matches) {
                        matches_to_components[id_match] = component_id;
                        components[component_id].insert(id_match);
                        
                        // Put all observations of this landmark on the stack.
                        const std::vector<loop_closure::Match>& lm_matches =
                        landmark_matches[id_match.landmark_result];
                        for (const loop_closure::Match& lm_match : lm_matches) {
                            if (matches_to_components[lm_match] == kInvalidComponentId) {
                                exploration_queue.push(lm_match);
                            }
                        }
                    }
                    
                    if (components[component_id].size() > max_component_size) {
                        max_component_size = components[component_id].size();
                        max_component_id = component_id;
                    }
                }
                exploration_queue.pop();
            }
        }
        
        // Only store the structure matches if there is a relevant amount of them.
        if (max_component_size > settings_.min_verify_matches_num) {
            const std::unordered_set<loop_closure::Match>& matches_max_component =
            components[max_component_id];
            typedef std::pair<loop_closure::KeypointId, vi_map::LandmarkId>
            KeypointLandmarkPair;
            std::unordered_set<KeypointLandmarkPair> used_matches;
            if (make_matches_unique) {
                // Conservative reserve to avoid rehashing.
                used_matches.reserve(2u * matches_max_component.size());
            }
            auto lock = (frame_matches_mutex == nullptr)
            ? std::unique_lock<std::mutex>()
            : std::unique_lock<std::mutex>(*frame_matches_mutex);
            for (const loop_closure::Match& structure_match : matches_max_component) {
                if (make_matches_unique) {
                    // clang-format off
                    const bool is_match_unique = used_matches.emplace(
                                                                      structure_match.keypoint_id_query,
                                                                      structure_match.landmark_result).second;
                    // clang-format on
                    if (!is_match_unique) {
                        // Skip duplicate (keypoint to landmark) structure matches.
                        continue;
                    }
                }
                frame_matches[structure_match.keypoint_id_query.frame_id].push_back(
                                                                                    structure_match);
            }
        }
    }
    
    template <>
    typename loop_closure::FrameToMatches::const_iterator
    MatchingBasedLoopDetector::getIteratorForMatch(
                                                   const loop_closure::FrameToMatches& frame_to_matches,
                                                   const loop_closure::Match& match) const {
        return frame_to_matches.find(match.keyframe_id_result);
    }
    
    template <>
    typename loop_closure::VertexToMatches::const_iterator
    MatchingBasedLoopDetector::getIteratorForMatch(
                                                   const loop_closure::VertexToMatches& vertex_to_matches,
                                                   const loop_closure::Match& match) const {
        return vertex_to_matches.find(match.keyframe_id_result.vertex_id);
    }
    
    template <>
    bool MatchingBasedLoopDetector::skipMatch(
                                              const IdToScoreMap<loop_closure::KeyframeId>& frame_to_score_map,
                                              const loop_closure::Match& match) const {
        const typename IdToScoreMap<loop_closure::KeyframeId>::const_iterator iter =
        frame_to_score_map.find(match.keyframe_id_result);
        return iter == frame_to_score_map.cend();
    }
    
    template <>
    bool MatchingBasedLoopDetector::skipMatch(
                                              const IdToScoreMap<loop_closure::VertexId>& /* vertex_to_score_map */,
                                              const loop_closure::Match& /* match */) const {
        // We do not skip vertices because we want to consider all keyframes that
        // passed the keyframe covisibility filtering step.
        return false;
    }
    
    template <>
    void MatchingBasedLoopDetector::computeRelevantIdsForFiltering(
                                                                   const loop_closure::FrameToMatches& frame_to_matches,
                                                                   IdToScoreMap<loop_closure::KeyframeId>* frame_to_score_map) const {
        CHECK_NOTNULL(frame_to_score_map)->clear();
        // Score each keyframe, then take the part which is in the
        // top fraction and allow only matches to landmarks which are associated with
        // these keyframes.
        
        scoring::ScoreList<loop_closure::KeyframeId> score_list;
        timing::Timer timer_scoring("Loop Closure: scoring for covisibility filter");
        CHECK(compute_keyframe_scores_);
        compute_keyframe_scores_(
                                 frame_to_matches, keyframe_id_to_num_descriptors_,
                                 static_cast<size_t>(NumDescriptors()), &score_list);
        timer_scoring.Stop();
        
        // We want to take matches from the best n score keyframes, but make sure
        // that we evaluate at minimum a given number.
        constexpr size_t kNumMinimumScoreIdsToEvaluate = 4u;
        size_t num_score_ids_to_evaluate = std::max<size_t>(
                                                            static_cast<size_t>(score_list.size() * settings_.fraction_best_scores),
                                                            kNumMinimumScoreIdsToEvaluate);
        // Ensure staying in bounds.
        num_score_ids_to_evaluate =
        std::min<size_t>(num_score_ids_to_evaluate, score_list.size());
        std::nth_element(
                         score_list.begin(), score_list.begin() + num_score_ids_to_evaluate,
                         score_list.end(),
                         [](const scoring::Score<loop_closure::KeyframeId>& lhs,
                            const scoring::Score<loop_closure::KeyframeId>& rhs) -> bool {
                             return lhs.second > rhs.second;
                         });
        frame_to_score_map->insert(
                                   score_list.begin(), score_list.begin() + num_score_ids_to_evaluate);
    }
    
    template <>
    void MatchingBasedLoopDetector::computeRelevantIdsForFiltering(
                                                                   const loop_closure::VertexToMatches& /* vertex_to_matches */,
                                                                   IdToScoreMap<loop_closure::VertexId>* /* vertex_to_score_map */) const {
        // We do not have to score vertices to filter unlikely matches because this
        // is done already at keyframe level.
    }

MatchingBasedLoopDetector::MatchingBasedLoopDetector(
    const MatchingBasedEngineSettings& settings)
    : settings_(settings), descriptor_index_(0) {
  setKeyframeScoringFunction();
  setDetectorEngine();

  VLOG(1) << "Initializing loop-detector:"
          << "\n\tengine: " << settings_.detector_engine_type_string
          << "\n\tscoring: " << settings_.scoring_function_type_string
          << "\n\tquantizer: " << settings_.projected_quantizer_filename
          << "\n\tproj matrix: " << settings_.projection_matrix_filename;
}

void MatchingBasedLoopDetector::ProjectDescriptors(
    const aslam::VisualFrame::DescriptorsT& descriptors,
    Eigen::MatrixXf* projected_descriptors) const {
  index_interface_->ProjectDescriptors(descriptors, projected_descriptors);
}

void MatchingBasedLoopDetector::ProjectDescriptors(
    const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
    Eigen::MatrixXf* projected_descriptors) const {
  index_interface_->ProjectDescriptors(descriptors, projected_descriptors);
}

void MatchingBasedLoopDetector::Find(
    const loop_closure::ProjectedImagePtrList& projected_image_ptr_list,
    const bool parallelize_if_possible,
    loop_closure::FrameToMatches* frame_matches_ptr) const {
  CHECK_NOTNULL(frame_matches_ptr)->clear();
  if (projected_image_ptr_list.empty()) {
    // Nothing to search if the query is empty.
    return;
  }
  CHECK(doProjectedImagesBelongToSameVertex(projected_image_ptr_list));

  timing::Timer timer_find("Loop Closure: Find projected images of vertex.");
  aslam::ScopedReadLock lock(&read_write_mutex);

  const int num_neighbors_to_search = getNumNeighborsToSearch();
  const size_t num_query_frames = projected_image_ptr_list.size();
  // Vertex to landmark covisibility filtering only makes sense, if more than
  // one camera is associated with the query vertex.
  const bool use_vertex_covis_filter = num_query_frames > 1u;
  const bool parallelize = parallelize_if_possible && num_query_frames > 1u;

  loop_closure::FrameToMatches temporary_frame_matches;
  std::mutex covis_frame_matches_mutex;
  std::mutex* covis_frame_matches_mutex_ptr =
      parallelize ? &covis_frame_matches_mutex : nullptr;

  std::function<void(const std::vector<size_t>&)> query_helper = [&](
      const std::vector<size_t>& range) {
    for (const size_t job_index : range) {
      const loop_closure::ProjectedImage& projected_image_query =
          *projected_image_ptr_list[job_index];
      const size_t num_descriptors_in_query_image = static_cast<size_t>(
          projected_image_query.projected_descriptors.cols());
      CHECK_EQ(
          num_descriptors_in_query_image,
          static_cast<size_t>(projected_image_query.measurements.cols()));
      Eigen::MatrixXi indices;
      indices.resize(num_neighbors_to_search, num_descriptors_in_query_image);
      Eigen::MatrixXf distances;
      distances.resize(num_neighbors_to_search, num_descriptors_in_query_image);
      timing::Timer timer_get_nn("Loop Closure: Get neighbors");
      // If this loop is running in multiple threads and the inverted
      // multi index is utilized as NN search structure, ensure that the nearest
      // neighbor back-end (libnabo) is not multi-threaded. Otherwise,
      // performance could decrease drastically.
      
        std::clock_t time1=std::clock();
  
        
      index_interface_->GetNNearestNeighborsForFeatures(
          projected_image_query.projected_descriptors, num_neighbors_to_search,
          &indices, &distances);
      if(FLAGS_summary_time){
                std::cout <<"chamo,"<<"loc raw match," << 1000 *(std::clock() - time1) / (float)CLOCKS_PER_SEC <<","<<projected_image_query.timestamp_nanoseconds/1000.0/1000.0/1000.0-1540000000<< std::endl;
            }
      timer_get_nn.Stop();

      KeyframeToMatchesMap keyframe_to_matches_map;
      for (int keypoint_idx = 0; keypoint_idx < indices.cols();
           ++keypoint_idx) {
        for (int nn_search_idx = 0; nn_search_idx < indices.rows();
             ++nn_search_idx) {
          const int nn_match_descriptor_idx =
              indices(nn_search_idx, keypoint_idx);
          const float nn_match_distance =
              distances(nn_search_idx, keypoint_idx);
          if (nn_match_descriptor_idx == -1 ||
              nn_match_distance == std::numeric_limits<float>::infinity()) {
            break;  // No more results for this feature.
          }
//           if(sqrt(nn_match_distance)>5){
//               std::cout<<"global lc dis: "<<sqrt(nn_match_distance)<<std::endl;
//           }
          
          loop_closure::Match structure_match;
          if (!getMatchForDescriptorIndex(
                  nn_match_descriptor_idx, projected_image_query, keypoint_idx,
                  &structure_match)) {
            continue;
          }

          keyframe_to_matches_map[structure_match.keyframe_id_result].push_back(
              structure_match);
        }
      }
      // We don't want to enforce unique matches yet in case of additional
      // vertex-landmark covisibility filtering. The reason for this is that
      // removing non-unique matches can split covisibility clusters.
      
        std::clock_t time2=std::clock();
  
  
      doCovisibilityFiltering(
          keyframe_to_matches_map, !use_vertex_covis_filter,
          &temporary_frame_matches, covis_frame_matches_mutex_ptr);
      if(FLAGS_summary_time){
        std::cout <<"chamo,"<<"loc covisi filter," << 1000 *(std::clock() - time2) / (float)CLOCKS_PER_SEC <<","<<projected_image_query.timestamp_nanoseconds/1000.0/1000.0/1000.0-1540000000<< std::endl;
    }
    }
  };
  if (parallelize) {
    static const size_t kNumHardwareThreads = common::getNumHardwareThreads();
    const size_t num_threads =
        std::min<size_t>(num_query_frames, kNumHardwareThreads);
    common::ParallelProcess(
        static_cast<int>(num_query_frames), query_helper, parallelize,
        num_threads);
  } else {
    std::vector<size_t> proj_img_indices(num_query_frames);
    // Fill in indices: {0, 1, 2, ...}.
    std::iota(proj_img_indices.begin(), proj_img_indices.end(), 0u);
    // Avoid spawning a thread by directly calling the function.
    query_helper(proj_img_indices);
  }

  if (use_vertex_covis_filter) {
    // Convert keyframe matches to vertex matches.
    const size_t num_frame_matches =
        loop_closure::getNumberOfMatches(temporary_frame_matches);
    VertexToMatchesMap vertex_to_matches_map;
    // Conservative reserve to avoid rehashing.
    vertex_to_matches_map.reserve(num_frame_matches);
    for (const loop_closure::FrameToMatches::value_type& id_frame_matches_pair :
         temporary_frame_matches) {
      for (const loop_closure::Match& match : id_frame_matches_pair.second) {
        vertex_to_matches_map[match.keyframe_id_result.vertex_id].push_back(
            match);
      }
    }
    doCovisibilityFiltering(
        vertex_to_matches_map, use_vertex_covis_filter, frame_matches_ptr);
  } else {
    frame_matches_ptr->swap(temporary_frame_matches);
  }
  CHECK_LE(frame_matches_ptr->size(), projected_image_ptr_list.size())
      << "There cannot be more query frames than projected images.";
}

bool MatchingBasedLoopDetector::getMatchForDescriptorIndex(
    int nn_match_descriptor_index,
    const loop_closure::ProjectedImage& projected_image_query,
    int keypoint_index_query, loop_closure::Match* structure_match_ptr) const {
  CHECK_GE(nn_match_descriptor_index, 0);
  CHECK_NOTNULL(structure_match_ptr);
  loop_closure::Match& structure_match = *structure_match_ptr;

  const DescriptorIndexToKeypointIdMap::const_iterator iter_keypoint_id_result =
      descriptor_index_to_keypoint_id_.find(nn_match_descriptor_index);
  CHECK(iter_keypoint_id_result != descriptor_index_to_keypoint_id_.cend());
  const loop_closure::KeypointId& keypoint_id_result =
      iter_keypoint_id_result->second;
  CHECK(keypoint_id_result.isValid());

  const Database::const_iterator iter_image_result =
      database_.find(keypoint_id_result.frame_id);
  CHECK(iter_image_result != database_.cend());
  const loop_closure::ProjectedImage& projected_image_result =
      *iter_image_result->second;

  // Skip matches to images which are too close in time.
  if (std::abs(
          projected_image_query.timestamp_nanoseconds -
          projected_image_result.timestamp_nanoseconds) <
          settings_.min_image_time_seconds * kSecondsToNanoSeconds &&
      projected_image_query.dataset_id == projected_image_result.dataset_id) {
    return false;
  }

  structure_match.keypoint_id_query.frame_id =
      projected_image_query.keyframe_id;
  structure_match.keypoint_id_query.keypoint_index =
      static_cast<size_t>(keypoint_index_query);
  structure_match.keyframe_id_result = keypoint_id_result.frame_id;

  if (!projected_image_result.landmarks.empty()) {
    CHECK_LT(
        keypoint_id_result.keypoint_index,
        projected_image_result.landmarks.size());
    structure_match.landmark_result =
        projected_image_result.landmarks[keypoint_id_result.keypoint_index];
    CHECK(structure_match.isValid());
  }
  return true;
}

void MatchingBasedLoopDetector::Insert(
    const loop_closure::ProjectedImage::Ptr& projected_image_ptr) {
  CHECK(projected_image_ptr != nullptr);
  const loop_closure::ProjectedImage& projected_image = *projected_image_ptr;

  aslam::ScopedWriteLock lock(&read_write_mutex);
  CHECK(projected_image.keyframe_id.isValid());
  CHECK_EQ(
      projected_image.projected_descriptors.cols(),
      static_cast<int>(projected_image.landmarks.size()));
  for (int keypoint_idx = 0;
       keypoint_idx < projected_image.projected_descriptors.cols();
       ++keypoint_idx) {
    CHECK(
        descriptor_index_to_keypoint_id_
            .emplace(
                std::piecewise_construct,
                std::forward_as_tuple(descriptor_index_),
                std::forward_as_tuple(
                    projected_image.keyframe_id, keypoint_idx))
            .second);
    ++descriptor_index_;
  }
  CHECK(index_interface_ != nullptr);
  index_interface_->AddDescriptors(projected_image.projected_descriptors);

  const loop_closure::KeyframeId& frame_id = projected_image.keyframe_id;
  CHECK(frame_id.isValid());
  CHECK(
      keyframe_id_to_num_descriptors_
          .emplace(frame_id, projected_image.projected_descriptors.cols())
          .second);
  // Remove the descriptors before adding the projected image to the database.
  std::shared_ptr<loop_closure::ProjectedImage> projected_copy(
      new loop_closure::ProjectedImage(projected_image));
  projected_copy->projected_descriptors.resize(Eigen::NoChange, 0);
  CHECK(database_.emplace(projected_image.keyframe_id, projected_copy).second)
      << "Duplicate projected image in database.";
}

void MatchingBasedLoopDetector::Clear() {
  aslam::ScopedWriteLock lock(&read_write_mutex);
  database_.clear();
  descriptor_index_to_keypoint_id_.clear();
  index_interface_->Clear();
  descriptor_index_ = 0;
}

void MatchingBasedLoopDetector::setKeyframeScoringFunction() {
  typedef MatchingBasedEngineSettings::KeyframeScoringFunctionType
      ScoringFunctionType;

  switch (settings_.keyframe_scoring_function_type) {
    case ScoringFunctionType::kAccumulation: {
      compute_keyframe_scores_ =
          &scoring::computeAccumulationScore<loop_closure::KeyframeId>;
      break;
    }
    case ScoringFunctionType::kProbabilistic: {
      compute_keyframe_scores_ =
          &scoring::computeProbabilisticScore<loop_closure::KeyframeId>;
      break;
    }
    default: {
      LOG(FATAL) << "Invalid selection ("
                 << settings_.scoring_function_type_string
                 << ") for scoring function.";
      break;
    }
  }
}

void MatchingBasedLoopDetector::setDetectorEngine() {
  typedef MatchingBasedEngineSettings::DetectorEngineType DetectorEngineType;

  switch (settings_.detector_engine_type) {
    case DetectorEngineType::kMatchingLDKdTree: {
      index_interface_.reset(
          new loop_closure::KDTreeIndexInterface(
              settings_.projection_matrix_filename));
      break;
    }
    case DetectorEngineType::kMatchingLDInvertedIndex: {
      index_interface_.reset(
          new loop_closure::InvertedIndexInterface(
              settings_.projected_quantizer_filename,
              settings_.num_closest_words_for_nn_search));
      break;
    }
    case DetectorEngineType::kMatchingLDInvertedMultiIndex: {
      index_interface_.reset(
          new loop_closure::InvertedMultiIndexInterface(
              settings_.projected_quantizer_filename,
              settings_.num_closest_words_for_nn_search));
      break;
    }
    case DetectorEngineType::kMatchingLDInvertedMultiIndexProductQuantization: {
      index_interface_.reset(
          new loop_closure::InvertedMultiProductQuantizationIndexInterface(
              settings_.projected_quantizer_filename,
              settings_.num_closest_words_for_nn_search));
      break;
    }
    default: {
      LOG(FATAL) << "Invalid selection ("
                 << settings_.detector_engine_type_string
                 << ") for detector engine.";
      break;
    }
  }
}

int MatchingBasedLoopDetector::getNumNeighborsToSearch() const {
  // Determine the best number of neighbors for a given database size.

  int num_neighbors_to_search = settings_.num_nearest_neighbors;
  if (num_neighbors_to_search == -1) {
    int num_descriptors_in_db = index_interface_->GetNumDescriptorsInIndex();
    if (num_descriptors_in_db < 1e4) {
      num_neighbors_to_search = 1;
    } else if (num_descriptors_in_db < 1e5) {
      num_neighbors_to_search = 2;
    } else if (num_descriptors_in_db < 1e6) {
      num_neighbors_to_search = 3;
    } else if (num_descriptors_in_db < 1e7) {
      num_neighbors_to_search = 6;
    } else {
      num_neighbors_to_search = 8;
    }
  }
  CHECK_GT(num_neighbors_to_search, 0);
  return num_neighbors_to_search;
}

void MatchingBasedLoopDetector::serialize(
    proto::MatchingBasedLoopDetector* proto_matching_based_loop_detector)
    const {
  CHECK_NOTNULL(proto_matching_based_loop_detector);
  CHECK_EQ(
      settings_.detector_engine_type_string,
      kMatchingLDInvertedMultiIndexString)
      << "Only the inverted multi-index can be serialized at the moment.";

  for (const DescriptorIndexToKeypointIdMap::value_type&
           descriptor_index_keypoint_pair : descriptor_index_to_keypoint_id_) {
    CHECK(descriptor_index_keypoint_pair.second.isValid());

    proto::MatchingBasedLoopDetector_DescriptorIndexToKeypoint*
        proto_descriptor_index_entry = CHECK_NOTNULL(
            proto_matching_based_loop_detector
                ->add_descriptor_index_to_keypoint());

    proto_descriptor_index_entry->set_descriptor_index(
        descriptor_index_keypoint_pair.first);

    descriptor_index_keypoint_pair.second.frame_id.vertex_id.serialize(
        proto_descriptor_index_entry->mutable_vertex_id());

    proto_descriptor_index_entry->set_frame_index(
        descriptor_index_keypoint_pair.second.frame_id.frame_index);

    proto_descriptor_index_entry->set_keypoint_index(
        descriptor_index_keypoint_pair.second.keypoint_index);
  }

  for (const Database::value_type& database_entry : database_) {
    CHECK(database_entry.first.isValid());

    proto::MatchingBasedLoopDetector_DatabaseEntry* proto_database_entry =
        CHECK_NOTNULL(
            proto_matching_based_loop_detector->add_database_entries());

    CHECK(database_entry.first.vertex_id.isValid());
    database_entry.first.vertex_id.serialize(
        proto_database_entry->mutable_vertex_id());

    proto_database_entry->set_frame_index(database_entry.first.frame_index);

    database_entry.second->serialize(
        proto_database_entry->mutable_projected_image());
  }

  proto_matching_based_loop_detector->set_descriptor_index(descriptor_index_);

  std::shared_ptr<loop_closure::InvertedMultiIndexInterface>
      inverted_multi_index_interface =
          std::dynamic_pointer_cast<loop_closure::InvertedMultiIndexInterface>(
              index_interface_);
  CHECK(inverted_multi_index_interface)
      << "Unable to cast the index interface "
      << "to type loop_closure::InvertedMultiIndexInterface. Only the inverted "
      << "multi-index can be serialized at the moment.";

  inverted_multi_index_interface->serialize(
      proto_matching_based_loop_detector
          ->mutable_inverted_multi_index_interface());

  for (const KeyframeIdToNumDescriptorsMap::value_type&
           keyframe_id_descriptors_pair : keyframe_id_to_num_descriptors_) {
    CHECK(keyframe_id_descriptors_pair.first.isValid());

    proto::MatchingBasedLoopDetector_KeyframeIdToNumDescriptors*
        proto_keyframe_id_to_num_descriptors = CHECK_NOTNULL(
            proto_matching_based_loop_detector
                ->add_keyframe_id_to_num_descriptors());
    proto::MatchingBasedLoopDetector_KeyframeIdToNumDescriptors_KeyframeId*
        proto_keyframe_id = CHECK_NOTNULL(
            proto_keyframe_id_to_num_descriptors->mutable_keyframe_id());

    keyframe_id_descriptors_pair.first.vertex_id.serialize(
        proto_keyframe_id->mutable_vertex_id());

    proto_keyframe_id->set_frame_index(
        static_cast<uint32_t>(keyframe_id_descriptors_pair.first.frame_index));

    proto_keyframe_id_to_num_descriptors->set_num_descriptors(
        static_cast<uint32_t>(keyframe_id_descriptors_pair.second));
  }
}

void MatchingBasedLoopDetector::deserialize(
    const proto::MatchingBasedLoopDetector&
        proto_matching_based_loop_detector) {
  for (const ::matching_based_loopclosure::proto::
           MatchingBasedLoopDetector_DescriptorIndexToKeypoint&
               proto_descriptor_index_to_keyframe :
       proto_matching_based_loop_detector.descriptor_index_to_keypoint()) {
    const int descriptor_index =
        proto_descriptor_index_to_keyframe.descriptor_index();

    loop_closure::KeypointId keypoint_id;
    keypoint_id.frame_id.vertex_id.deserialize(
        proto_descriptor_index_to_keyframe.vertex_id());
    keypoint_id.frame_id.frame_index =
        proto_descriptor_index_to_keyframe.frame_index();
    keypoint_id.keypoint_index =
        proto_descriptor_index_to_keyframe.keypoint_index();
    CHECK(keypoint_id.isValid());

    CHECK(
        descriptor_index_to_keypoint_id_.emplace(descriptor_index, keypoint_id)
            .second);
  }

  for (const ::matching_based_loopclosure::proto::
           MatchingBasedLoopDetector_DatabaseEntry& proto_database_entry :
       proto_matching_based_loop_detector.database_entries()) {
    loop_closure::KeyframeId keyframe_id;
    keyframe_id.frame_index = proto_database_entry.frame_index();
    keyframe_id.vertex_id.deserialize(proto_database_entry.vertex_id());
    CHECK(keyframe_id.vertex_id.isValid());
    CHECK(keyframe_id.isValid());

    std::shared_ptr<loop_closure::ProjectedImage> projected_image(
        new loop_closure::ProjectedImage);

    projected_image->deserialize(proto_database_entry.projected_image());

    CHECK(
        database_.insert(std::make_pair(keyframe_id, projected_image)).second);
  }

  descriptor_index_ = proto_matching_based_loop_detector.descriptor_index();

  std::shared_ptr<loop_closure::InvertedMultiIndexInterface>
      inverted_multi_index_interface =
          std::dynamic_pointer_cast<loop_closure::InvertedMultiIndexInterface>(
              index_interface_);
  CHECK(inverted_multi_index_interface);
  inverted_multi_index_interface->deserialize(
      proto_matching_based_loop_detector.inverted_multi_index_interface());

  for (const ::matching_based_loopclosure::proto::
           MatchingBasedLoopDetector_KeyframeIdToNumDescriptors&
               proto_keyframe_id_to_num_descriptors :
       proto_matching_based_loop_detector.keyframe_id_to_num_descriptors()) {
    CHECK(proto_keyframe_id_to_num_descriptors.has_keyframe_id())
        << "The loop detector has been serialized with a different template "
        << "parameter.";
    const ::matching_based_loopclosure::proto::
        MatchingBasedLoopDetector_KeyframeIdToNumDescriptors_KeyframeId&
            proto_keyframe_id =
                proto_keyframe_id_to_num_descriptors.keyframe_id();
    loop_closure::KeyframeId frame_id;
    frame_id.vertex_id.deserialize(proto_keyframe_id.vertex_id());
    frame_id.frame_index = proto_keyframe_id.frame_index();
    CHECK(frame_id.isValid());
    const size_t num_descriptors =
        proto_keyframe_id_to_num_descriptors.num_descriptors();
    CHECK(
        keyframe_id_to_num_descriptors_.emplace(frame_id, num_descriptors)
            .second);
  }
}

}  // namespace matching_based_loopclosure
