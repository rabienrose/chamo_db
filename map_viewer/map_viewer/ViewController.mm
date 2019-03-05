#import "ViewController.h"
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include "gflags/gflags.h"
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <ros/ros.h>
#include <sensors/imu.h>
#include <sensors/sensor-factory.h>
#include <vi-map/vi-map-serialization.h>
#include <math.h>



#include <maplab-common/threading-helpers.h>

DEFINE_string(
              vio_localization_map_folder, "",
              "Path to a localization summary map or a full VI-map used for "
              "localization.");
DEFINE_string(
              ncamera_calibration, "ncamera.yaml",
              "Path to the camera calibration yaml.");
// TODO(schneith): Unify these two noise definitions.
DEFINE_string(
              imu_parameters_rovio, "imu-rovio.yaml",
              "Path to the imu configuration yaml "
              "for ROVIO.");
DEFINE_string(
              imu_parameters_maplab, "imu-maplab.yaml",
              "Path to the imu configuration yaml for MAPLAB.");
DEFINE_string(
              save_map_folder, "", "Save map to folder; if empty nothing is saved.");
DEFINE_bool(
            overwrite_existing_map, true,
            "If set to true, an existing map will be overwritten on save. Otherwise, a "
            "number will be appended to save_map_folder to obtain an available "
            "folder.");
DEFINE_bool(
            optimize_map_to_localization_map, false,
            "Optimize and process the map into a localization map before "
            "saving it.");

DEFINE_bool(use_map, false, "");
DEFINE_bool(do_simulation, false, "");

DECLARE_bool(map_builder_save_image_as_resources);
DECLARE_string(datasource_rosbag);
DECLARE_string(algorithm_config_file);
DECLARE_bool(summary_time);

DECLARE_string(datasource_type);
DECLARE_double(vio_throttler_max_output_frequency_hz);
DECLARE_int32(swe_feature_tracking_detector_orb_pyramid_levels);
DECLARE_string(feature_descriptor_type);
DECLARE_double(lc_min_inlier_ratio);
DECLARE_int32(lc_num_neighbors);
DECLARE_bool(rovio_update_filter_on_imu);
DECLARE_string(lc_projection_matrix_filename);
DECLARE_string(lc_projected_quantizer_filename);
DECLARE_int32(num_hardware_threads);

@interface ViewController ()
@end
#define PI 3.1415
@implementation ViewController
- (int)getCamType{
    return cam_type;
}
- (void)setCamType:(int)type{
    cam_type=type;
}

- (cv::Mat)getNewFrame{
    cv::Mat img;
    if(hasNewFrame){
        if (loc_result){
            hasNewFrame=false;
            img =loc_result->debug_img;
        }
    }
    return img;
}

void loadFlags(){
    FLAGS_do_simulation=false;
    NSBundle* myBundle = [NSBundle mainBundle];
    NSString* mycam_str;
    if(FLAGS_do_simulation){
        mycam_str = [myBundle pathForResource:@"ncamera" ofType:@"yaml"];
    }else{
        mycam_str = [myBundle pathForResource:@"ncamera_ios" ofType:@"yaml"];
    }
    FLAGS_ncamera_calibration=[mycam_str UTF8String];
    NSString* myimu_str = [myBundle pathForResource:@"imu" ofType:@"yaml"];
    FLAGS_imu_parameters_maplab =[myimu_str UTF8String];
    NSString* myimumaplab_str = [myBundle pathForResource:@"imu-sigmas" ofType:@"yaml"];
    FLAGS_imu_parameters_rovio=[myimumaplab_str UTF8String];
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSDate *date = [NSDate date];
    NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
    [formatter setDateFormat:@"MM-dd-HH-mm-ss"];
    NSString *timeString = [formatter stringFromDate:date];
    NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:timeString];
    std::string full_file_name([full_addr UTF8String]);
    FLAGS_save_map_folder="";
    NSString* mybag_str = [myBundle pathForResource:@"data" ofType:@"bag"];
    FLAGS_datasource_rosbag=[mybag_str UTF8String];
    FLAGS_vio_localization_map_folder =[myBundle.resourcePath UTF8String];
    
    NSString* myalgoconfig_str = [myBundle pathForResource:@"rovio_default_config" ofType:@"info"];
    FLAGS_algorithm_config_file=[myalgoconfig_str UTF8String];
    FLAGS_summary_time=false;
    
    if(FLAGS_do_simulation){
        FLAGS_datasource_type="rosbag";
    }else{
        FLAGS_datasource_type="rostopic";
    }
    
    FLAGS_vio_throttler_max_output_frequency_hz=10;
    FLAGS_swe_feature_tracking_detector_orb_pyramid_levels=8;
    FLAGS_feature_descriptor_type="freak";
    FLAGS_lc_min_inlier_ratio=0.1;
    FLAGS_lc_num_neighbors=10;
    FLAGS_rovio_update_filter_on_imu=false;
    FLAGS_lc_projection_matrix_filename=[[myBundle pathForResource:@"projection_matrix_freak" ofType:@"dat"] UTF8String];
    FLAGS_lc_projected_quantizer_filename=[[myBundle pathForResource:@"inverted_multi_index_quantizer_freak" ofType:@"dat"] UTF8String];
    FLAGS_use_map=true;
    FLAGS_num_hardware_threads=0;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    loadFlags();
    flow.reset(message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(common::getNumHardwareThreads()));
    algo =[[LocAlgo alloc] init];
    if(!FLAGS_do_simulation){
        [algo startAlgo:flow];
    }

    if(FLAGS_use_map){
        localization_map.reset(new summary_map::LocalizationSummaryMap);
        if (!localization_map->loadFromFolder(FLAGS_vio_localization_map_folder)) {
            NSLog(@"load map wrong!!");
        }
    }
    
    SCNScene *scene = [SCNScene scene];
    worldNode=scene.rootNode;

    cameraNode = [SCNNode node];
    cameraNode.camera = [SCNCamera camera];
    cameraNode.camera.usesOrthographicProjection=YES;
    cameraNode.camera.orthographicScale=20;
    cameraNode.camera.automaticallyAdjustsZRange=YES;
    [worldNode addChildNode:cameraNode];
    cam_distence=500;
    cameraNode.position = SCNVector3Make(0, 0, cam_distence);

    if(FLAGS_use_map){
        SCNNode *pcNode = [SCNNode node];
        pcNode.geometry=[algo fillPC: localization_map];
        if (pcNode.geometry== nullptr){
            return;
        }
        [worldNode addChildNode:pcNode];
    }
    
    meNode = [SCNNode node];
    meNode.geometry = [SCNPyramid pyramidWithWidth:1 height:2 length:1];
    [worldNode addChildNode:meNode];
    
    SCNNode* girdNode = [SCNNode node];
    girdNode.geometry = [algo buildGird];
    [worldNode addChildNode:girdNode];
    
    SCNView *scnView = (SCNView *)self.view;
    UIPanGestureRecognizer *panGesture = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handlePan:)];
    UIRotationGestureRecognizer *rotGesture = [[UIRotationGestureRecognizer alloc] initWithTarget:self action:@selector(handleRot:)];
    UIPinchGestureRecognizer *pinGesture = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(handlePin:)];
    UITapGestureRecognizer *tapGesture = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleTap:)];
    NSMutableArray *gestureRecognizers = [NSMutableArray array];
    [gestureRecognizers addObject:panGesture];
    [gestureRecognizers addObject:rotGesture];
    [gestureRecognizers addObject:pinGesture];
    [gestureRecognizers addObject:tapGesture];
    scnView.gestureRecognizers = gestureRecognizers;

    static constexpr char kSubscriberNodeName[] = "RenderingFlow";
    flow->registerSubscriber<message_flow_topics::VIO_UPDATES>(kSubscriberNodeName, message_flow::DeliveryOptions(), [self](const vio::VioUpdate::ConstPtr& vio_update){
        loc_result=vio_update;
        hasNewFrame=true;
    });
    
    scnView.scene = scene;
    scnView.allowsCameraControl = YES;
    scnView.showsStatistics = YES;
    scnView.backgroundColor = [UIColor blackColor];
    scnView.delegate = self;
    scnView.playing=YES;
    scnView.preferredFramesPerSecond=15;
    
    camera_system = aslam::NCamera::loadFromYaml(FLAGS_ncamera_calibration);
    vi_map::Imu::UniquePtr maplab_imu_sensor =vi_map::createFromYaml<vi_map::Imu>(FLAGS_imu_parameters_maplab);
    vi_map::ImuSigmas rovio_imu_sigmas;
    rovio_imu_sigmas.loadFromYaml(FLAGS_imu_parameters_rovio);
    loc_node.reset(new rovioli::RovioliNode(camera_system, std::move(maplab_imu_sensor), rovio_imu_sigmas,FLAGS_save_map_folder, localization_map.get(), flow.get()));
    if(FLAGS_do_simulation){
        loc_node->start();
    }

    cam_pitch=90;
    cam_yaw=0;
    cam_roll=0;
    cam_type=2;
    cam_tar_x=0;
    cam_tar_y=0;
    cam_scale=cameraNode.camera.orthographicScale;
    hasNewFrame=false;

    UIStoryboard* storyboard = [UIStoryboard storyboardWithName:@"Main" bundle:nil];
    detail_view = [storyboard instantiateViewControllerWithIdentifier:@"DetailViewController"];
    detail_view.delegate = self;
}

- (void)renderer:(id<SCNSceneRenderer>)renderer updateAtTime:(NSTimeInterval)time{
    
    cameraNode.eulerAngles = SCNVector3Make(0,0,cam_roll/180*PI);
    cameraNode.camera.orthographicScale=cam_scale;
    Eigen::Vector3d my_posi=Eigen::Vector3d::Zero();
    Eigen::Quaterniond r_wc=Eigen::Quaterniond::Identity();
    Eigen::Quaterniond r_wc1=Eigen::Quaterniond::Identity();
    Eigen::Quaterniond r_wv=Eigen::Quaterniond::Identity();
    if(loc_result){
        Eigen::Quaterniond r_wi =loc_result->vinode.get_T_M_I().getEigenQuaternion();
        Eigen::Vector3d local_h(0,0,2);
        Eigen::Quaterniond r_ic = camera_system->get_T_C_B(0).inverse().getEigenQuaternion();
        r_wc =r_wi*r_ic;
        Eigen::Quaterniond r_cv = Eigen::Quaterniond(0, 0, -0.7071068, 0.7071068);
        r_wv=r_wc*r_cv;
        Eigen::Quaterniond r_cc1 = Eigen::Quaterniond(0, 1, 0, 0);
        r_wc1=r_wc*r_cc1;
        meNode.orientation=SCNVector4Make(r_wv.x(), r_wv.y(), r_wv.z(), r_wv.w());
        my_posi =loc_result->vinode.get_T_M_I().getPosition()+r_wc*local_h;
        meNode.position=SCNVector3Make(my_posi(0), my_posi(1), my_posi(2));
        
    }
    
    if(cam_type==2){
        cameraNode.camera.usesOrthographicProjection=YES;
        cameraNode.position=SCNVector3Make(cam_tar_x, cam_tar_y, cam_distence);
    }else if(cam_type==1){
        cameraNode.camera.usesOrthographicProjection=YES;
        cameraNode.position=SCNVector3Make(my_posi(0), my_posi(1), cam_distence);
    }else if(cam_type==0){
        cameraNode.camera.usesOrthographicProjection=NO;
        cameraNode.position=SCNVector3Make(my_posi(0), my_posi(1), my_posi(2));
        cameraNode.orientation=SCNVector4Make(r_wc1.x(), r_wc1.y(), r_wc1.z(), r_wc1.w());
    }
}

- (BOOL)shouldAutorotate
{
    return YES;
}

- (BOOL)prefersStatusBarHidden {
    return YES;
}

- (UIInterfaceOrientationMask)supportedInterfaceOrientations
{
    if ([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPhone) {
        return UIInterfaceOrientationMaskAllButUpsideDown;
    } else {
        return UIInterfaceOrientationMaskAll;
    }
}

- (void) handleTap:(UIGestureRecognizer*)gestureRecognize
{
    [self presentViewController:detail_view animated:NO completion:nil];
}

- (void) handlePan:(UIGestureRecognizer*)gestureRecognize
{
    SCNView *scnView = (SCNView *)self.view;
    UIPanGestureRecognizer* reco = (UIPanGestureRecognizer*)gestureRecognize;
    CGPoint p = [reco translationInView:scnView];
    if(reco.numberOfTouches==2){
    }else if(reco.numberOfTouches==1){
        if(reco.state==1){
            temp_cam_tar_x=p.x;
            temp_cam_tar_y=p.y;
        }else if(reco.state==2){
            float rate=cam_scale/200;
            float rot_x=(p.x-temp_cam_tar_x)*cos(-cam_roll/180*PI)-(p.y-temp_cam_tar_y)*sin(-cam_roll/180*PI);
            float rot_y=(p.x-temp_cam_tar_x)*sin(-cam_roll/180*PI)+(p.y-temp_cam_tar_y)*cos(-cam_roll/180*PI);
            cam_tar_x=cam_tar_x-rot_x*rate;
            cam_tar_y=cam_tar_y+rot_y*rate;
            temp_cam_tar_x=p.x;
            temp_cam_tar_y=p.y;
        }
    }
}

- (void) handleRot:(UIGestureRecognizer*)gestureRecognize
{
    SCNView *scnView = (SCNView *)self.view;
    UIRotationGestureRecognizer* reco = (UIRotationGestureRecognizer*)gestureRecognize;
    if(reco.state==1){
        temp_cam_roll=reco.rotation;
    }else if(reco.state==2){
        cam_roll=cam_roll+(reco.rotation-temp_cam_roll)*180/PI;
        if(cam_yaw>360){
            cam_roll=cam_roll-360;
        }
        if(cam_yaw<0){
            cam_roll=cam_roll+360;
        }
        temp_cam_roll=reco.rotation;
    }
}

- (void) handlePin:(UIGestureRecognizer*)gestureRecognize
{
    SCNView *scnView = (SCNView *)self.view;
    UIPinchGestureRecognizer* reco = (UIPinchGestureRecognizer*)gestureRecognize;
    if(reco.state==1){
        temp_scale=reco.scale;
    }else if(reco.state==2){
        cam_scale=cam_scale*(temp_scale/reco.scale);
        temp_scale=reco.scale;
    }
}

@end
