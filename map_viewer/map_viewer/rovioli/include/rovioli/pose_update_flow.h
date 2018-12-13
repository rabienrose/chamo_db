#ifndef ROVIOLI_POSE_UPDATE_FLOW_
#define ROVIOLI_POSE_UPDATE_FLOW_

#include <atomic>
#include <memory>
#include <vector>

#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include <gflags/gflags.h>

DECLARE_string(datasource_rosbag);
DECLARE_string(lidar_calibration_file); 
namespace rovioli {

class PoseUpdateFlow {
public:
    std::map<double, aslam::Transformation, std::greater<double>> poses;
    std::vector<std::string> split(const std::string& str, const std::string& delim)
    {
        std::vector<std::string> tokens;
        size_t prev = 0, pos = 0;
        do
        {
            pos = str.find(delim, prev);
            if (pos == std::string::npos) pos = str.length();
            std::string token = str.substr(prev, pos-prev);
            if (!token.empty()) tokens.push_back(token);
            prev = pos + delim.length();
        }
        while (pos < str.length() && prev < str.length());
        return tokens;
    }
    explicit PoseUpdateFlow(const aslam::NCamera& camera_system, std::string traj="") {
        const aslam::Transformation t_cb = camera_system.get_T_C_B(0);
        std::ifstream infile(traj);
        std::string line;
        while(true){
            std::getline(infile, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            double time_stamp;
            time_stamp=atof(splited[2].c_str());
            Eigen::Matrix4d pose_eigen=Eigen::Matrix4d::Identity();
            int count_temp=0;
            for(int i=0; i<3; i++){
                for(int j=0; j<4; j++){
                    pose_eigen(i,j)=atof(splited[count_temp+3].c_str());
                    count_temp++;
                }
            }
            
            aslam::Transformation world_pose_c(pose_eigen);
            
            cv::FileStorage fs(FLAGS_lidar_calibration_file, cv::FileStorage::READ);
            if(!fs.isOpened())
            {
                std::cout<<"Invalid calibration filename."<<std::endl;
            }
            
            cv::Mat cam_ext; 
            fs["CameraExtrinsicMat"]>>cam_ext;
            Eigen::Matrix4d camera_lidar;
            for (int i=0; i<4; i++){
                for (int j=0; j<4; j++){
                    camera_lidar(i,j)=cam_ext.at<double>(i,j);        
                }
            }
            aslam::Transformation camera_lidar_aslam(camera_lidar);
            
            Eigen::Matrix4d hori_lidar=Eigen::Matrix4d::Identity();
            hori_lidar.block<3,3>(0,0)=Eigen::AngleAxisd(7 * M_PI / 180, Eigen::Vector3d::UnitY()).matrix();
            aslam::Transformation hori_lidar_aslam(hori_lidar);
            aslam::Transformation world_pose_b=world_pose_c*hori_lidar_aslam*camera_lidar_aslam.inverse()*t_cb;
            poses[time_stamp]=world_pose_b;
        }
    }
    void attachToMessageFlow(message_flow::MessageFlow* flow) {
        CHECK_NOTNULL(flow);
        static constexpr char kSubscriberNodeName[] = "PoseUpdateFlow";

        std::function<void(vio::LocalizationResult::ConstPtr)> publish_result =
            flow->registerPublisher<message_flow_topics::LOCALIZATION_RESULT>();
        std::function<void(const RovioEstimate::ConstPtr&)> publish_est =
            flow->registerPublisher<message_flow_topics::ROVIO_ESTIMATES>();

        flow->registerSubscriber<message_flow_topics::THROTTLED_TRACKED_NFRAMES_AND_IMU>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [publish_result,
         this](const vio::SynchronizedNFrameImu::ConstPtr& nframe_imu) {
            double cur_time= aslam::time::nanoSecondsToSeconds(nframe_imu->nframe->getMinTimestampNanoseconds());
            std::map<double, aslam::Transformation>::iterator max_it= poses.lower_bound(cur_time);
            std::map<double, aslam::Transformation>::iterator min_it=max_it--;
            double min_dis=fabs(min_it->first-cur_time);
            double max_dis=fabs(max_it->first-cur_time);
            std::map<double, aslam::Transformation>::iterator query_it;
            if(min_dis<=max_dis){
                query_it=min_it;
            }else{
                query_it=max_it;
            }

            double query_time= query_it->first;
            aslam::Transformation query_pose =query_it->second;
            if (query_time<cur_time+0.05 &&query_time>cur_time-0.05) {
                vio::LocalizationResult::Ptr loc_result(new vio::LocalizationResult);
                loc_result->localization_type=Localizer::LocalizationMode::kGlobal;
                loc_result->timestamp=nframe_imu->nframe->getMinTimestampNanoseconds();
                loc_result->T_G_I_lc_pnp=query_pose;
                loc_result->nframe_id=nframe_imu->nframe->getId();
                publish_result(loc_result);             
            }
            });
    }

};

}  // namespace rovioli

#endif 
