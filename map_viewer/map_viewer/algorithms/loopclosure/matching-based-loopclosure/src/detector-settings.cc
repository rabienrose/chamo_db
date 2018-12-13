#include "matching-based-loopclosure/detector-settings.h"

#include <descriptor-projection/flags.h>
#include <glog/logging.h>
#include <loopclosure-common/flags.h>
#include <loopclosure-common/types.h>

DEFINE_string(
              lc_detector_engine,
              matching_based_loopclosure::kMatchingLDInvertedMultiIndexString,
              "Which loop-closure engine to use");
DEFINE_string(
              lc_scoring_function, matching_based_loopclosure::kAccumulationString,
              "Type of scoring function to be used for scoring keyframes.");
DEFINE_double(
              lc_min_image_time_seconds, 10.0,
              "Minimum time between matching images to allow a loop closure.");
DEFINE_uint64(
              lc_min_verify_matches_num, 10u,
              "The minimum number of matches needed to verify geometry.");
DEFINE_double(
              lc_fraction_best_scores, 0.25,
              "Fraction of best scoring "
              "keyframes/vertices that are considered for covisibility filtering.");
DEFINE_int32(
             lc_num_neighbors, -1,
             "Number of neighbors to retrieve for loop-closure. -1 auto.");
DEFINE_int32(
             lc_num_words_for_nn_search, 10,
             "Number of nearest words to retrieve in the inverted index.");

namespace matching_based_loopclosure {
    
    MatchingBasedEngineSettings::MatchingBasedEngineSettings()
    : projection_matrix_filename(FLAGS_lc_projection_matrix_filename),
    projected_quantizer_filename(FLAGS_lc_projected_quantizer_filename),
    num_closest_words_for_nn_search(FLAGS_lc_num_words_for_nn_search),
    min_image_time_seconds(FLAGS_lc_min_image_time_seconds),
    min_verify_matches_num(FLAGS_lc_min_verify_matches_num),
    fraction_best_scores(FLAGS_lc_fraction_best_scores),
    num_nearest_neighbors(FLAGS_lc_num_neighbors) {
        CHECK_GT(num_closest_words_for_nn_search, 0);
        CHECK_GE(min_image_time_seconds, 0.0);
        CHECK_GE(min_verify_matches_num, 0u);
        CHECK_GT(fraction_best_scores, 0.f);
        CHECK_LT(fraction_best_scores, 1.f);
        CHECK_GE(num_nearest_neighbors, -1);
        
        setKeyframeScoringFunctionType(FLAGS_lc_scoring_function);
        setDetectorEngineType(FLAGS_lc_detector_engine);
    }
    
    void MatchingBasedEngineSettings::setKeyframeScoringFunctionType(
                                                                     const std::string& scoring_function_string) {
        scoring_function_type_string = scoring_function_string;
        if (scoring_function_string == kAccumulationString) {
            keyframe_scoring_function_type = KeyframeScoringFunctionType::kAccumulation;
        } else if (scoring_function_string == kProbabilisticString) {
            keyframe_scoring_function_type =
            KeyframeScoringFunctionType::kProbabilistic;
        } else {
            LOG(FATAL) << "Unknown scoring function type: " << scoring_function_string;
        }
    }
    
    void MatchingBasedEngineSettings::setDetectorEngineType(
                                                            const std::string& detector_engine_string) {
        detector_engine_type_string = detector_engine_string;
        if (detector_engine_string == kMatchingLDKdTreeString) {
            detector_engine_type = DetectorEngineType::kMatchingLDKdTree;
        } else if (detector_engine_string == kMatchingLDInvertedIndexString) {
            detector_engine_type = DetectorEngineType::kMatchingLDInvertedIndex;
        } else if (detector_engine_string == kMatchingLDInvertedMultiIndexString) {
            detector_engine_type = DetectorEngineType::kMatchingLDInvertedMultiIndex;
        } else if (
                   detector_engine_string ==
                   kMatchingLDInvertedMultiIndexProductQuantizationString) {
            detector_engine_type =
            DetectorEngineType::kMatchingLDInvertedMultiIndexProductQuantization;
        } else {
            LOG(FATAL) << "Unknown loop detector engine type: "
            << detector_engine_string;
        }
    }
    
    std::string MatchingBasedEngineSettings::getLoopClosureFilePath() {
        return "";
    }
    
}  // namespace matching_based_loopclosure
