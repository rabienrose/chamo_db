#import "DetailViewController.h"
#include "aslam/cameras/ncamera.h"
#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>
#import <SceneKit/SceneKit.h>
#include <message-flow/message-flow.h>
#include <localization-summary-map/localization-summary-map.h>
#include "rovioli/rovioli-node.h"
#include "LocAlgo.h"


@interface ViewController : UIViewController<SCNSceneRendererDelegate, DetailViewControllerDelegate>{
    std::unique_ptr<message_flow::MessageFlow> flow;
    LocAlgo* algo;
    SCNNode *meNode;
    SCNNode *cameraNode;
    SCNNode *worldNode;
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map;
    vio::VioUpdate::ConstPtr loc_result;
    std::unique_ptr<rovioli::RovioliNode> loc_node;
    float cam_pitch;
    float cam_yaw;
    float cam_roll;
    Eigen::Vector3d cam_target;
    float cam_distence;
    int cam_type;
    bool show_features;
    bool show_backgounr_pc;
    float temp_cam_pitch;
    float temp_cam_yaw;
    float temp_cam_roll;
    float temp_cam_tar_x;
    float temp_cam_tar_y;
    float cam_tar_x;
    float cam_tar_y;
    float temp_scale;
    float cam_scale;
    DetailViewController *detail_view;
    bool hasNewFrame;
    aslam::NCamera::Ptr camera_system;
}
    @property (nonatomic) dispatch_queue_t sessionQueue;

@end

