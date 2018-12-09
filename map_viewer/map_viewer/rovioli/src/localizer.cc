#include "rovioli/localizer.h"

#include <localization-summary-map/localization-summary-map.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <vio-common/vio-types.h>

DECLARE_bool(summary_time);

namespace rovioli {

Localizer::Localizer(
    const summary_map::LocalizationSummaryMap& localization_summary_map,
    const bool visualize_localization)
    : localization_summary_map_(localization_summary_map) {
  current_localization_mode_ = Localizer::LocalizationMode::kGlobal;

  global_loop_detector_.reset(new loop_detector_node::LoopDetectorNode);

  CHECK(global_loop_detector_ != nullptr);
  if (visualize_localization) {
    global_loop_detector_->instantiateVisualizer();
  }

  LOG(INFO) << "Creating localization database...";
  global_loop_detector_->addLocalizationSummaryMapToDatabase(
      localization_summary_map_);
  LOG(INFO) << "Done.";
  lastest_state=NULL;
  state=-1;
}

Localizer::LocalizationMode Localizer::getCurrentLocalizationMode() const {
  return current_localization_mode_;
}

bool Localizer::localizeNFrame(
    const aslam::VisualNFrame::ConstPtr& nframe,
    vio::LocalizationResult* localization_result){
  CHECK(nframe);
  CHECK_NOTNULL(localization_result);

  bool result = false;
  vi_map::VertexKeyPointToStructureMatchList inlier_structure_matches;
  switch (current_localization_mode_) {
    case Localizer::LocalizationMode::kGlobal:
      result = localizeNFrameGlobal(nframe, &localization_result->T_G_I_lc_pnp, inlier_structure_matches);
      break;
    case Localizer::LocalizationMode::kMapTracking:
      result =
          localizeNFrameMapTracking(nframe, &localization_result->T_G_I_lc_pnp);
      break;
    default:
      LOG(FATAL) << "Unknown localization mode.";
      break;
  }
  
  for (int i=0; i<inlier_structure_matches.size(); i++){
      vi_map::LandmarkId mp_id = inlier_structure_matches[i].landmark_result;
      Eigen::Vector3d posi = localization_summary_map_.getGLandmarkPosition(mp_id);
      localization_result->matched_posi.push_back(posi);
      unsigned int kp_index = inlier_structure_matches[i].keypoint_index_query;
      localization_result->matched_uv.push_back(nframe->getFrame(0).getKeypointMeasurement(kp_index));
  }

  localization_result->timestamp = nframe->getMinTimestampNanoseconds();
  localization_result->nframe_id = nframe->getId();
  localization_result->localization_type = current_localization_mode_;
  
    if(state==-1){
        state=1;
        succ_count=0;
        fail_count=0;
    }else if(state==1){
        if(result){
            if((last_posi-localization_result->T_G_I_lc_pnp.getPosition()).norm()<1){
                if(succ_count>=2){
                    fail_count=0;
                    result=true;
                }else{
                    result=false;
                }
                succ_count++;
            }else{
                if(fail_count>3){
                    succ_count=0;
                }
                fail_count++;
                result=false;
            }
        }else{
            if(fail_count>3){
                succ_count=0;
            }
            fail_count++;
            result=false;
        }
    }
    last_posi=localization_result->T_G_I_lc_pnp.getPosition();
  return result;
}

bool Localizer::localizeNFrameGlobal(
    const aslam::VisualNFrame::ConstPtr& nframe,
    aslam::Transformation* T_G_I_lc_pnp, vi_map::VertexKeyPointToStructureMatchList& inlier_structure_matches) const {
  constexpr bool kSkipUntrackedKeypoints = false;
  unsigned int num_lc_matches;
  aslam::Transformation T_G_I_init;
  bool re_local_match=false; 
  if(false){
        T_G_I_init=lastest_state->T_G_M*lastest_state->vinode.get_T_M_I();
        re_local_match = global_loop_detector_->findPoseNearby(*nframe, localization_summary_map_, &T_G_I_init, &num_lc_matches, &inlier_structure_matches);
        *T_G_I_lc_pnp=T_G_I_init;
  }
  bool re=true;
  if(re_local_match==false){
      std::clock_t time=std::clock();
      re= global_loop_detector_->findNFrameInSummaryMapDatabase(
      *nframe, kSkipUntrackedKeypoints, localization_summary_map_, T_G_I_lc_pnp,
      &num_lc_matches, &inlier_structure_matches);
      if(FLAGS_summary_time){
          std::cout <<"chamo,"<<"match map," << 1000 *(std::clock() - time) / (float)CLOCKS_PER_SEC <<","<<nframe->getFrame(0).getTimestampNanoseconds()/1000.0/1000.0/1000.0-1540000000<< std::endl;
      }
      
  }
  
  return re;
}

bool Localizer::localizeNFrameMapTracking(
    const aslam::VisualNFrame::ConstPtr& /*nframe*/,
    aslam::Transformation* /*T_G_I_lc_pnp*/) const {
  LOG(FATAL) << "Not implemented yet.";
  return false;
}

}  // namespace rovioli
