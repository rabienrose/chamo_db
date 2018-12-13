#import "ViewController.h"
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include "aslam/cameras/ncamera.h"
#include "gflags/gflags.h"
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <ros/ros.h>
#include <sensors/imu.h>
#include <sensors/sensor-factory.h>
#include <vi-map/vi-map-serialization.h>


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


void loadFlags(){
    NSBundle* myBundle = [NSBundle mainBundle];
    NSString* mycam_str = [myBundle pathForResource:@"ncamera" ofType:@"yaml"];
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
    FLAGS_summary_time=true;
    FLAGS_datasource_type="rosbag";
    FLAGS_vio_throttler_max_output_frequency_hz=10;
    FLAGS_swe_feature_tracking_detector_orb_pyramid_levels=8;
    FLAGS_feature_descriptor_type="freak";
    FLAGS_lc_min_inlier_ratio=0.1;
    FLAGS_lc_num_neighbors=10;
    FLAGS_rovio_update_filter_on_imu=false;
    FLAGS_lc_projection_matrix_filename=[[myBundle pathForResource:@"projection_matrix_freak" ofType:@"dat"] UTF8String];
    FLAGS_lc_projected_quantizer_filename=[[myBundle pathForResource:@"inverted_multi_index_quantizer_freak" ofType:@"dat"] UTF8String];
    //FLAGS_num_hardware_threads=1;
}

@interface ViewController ()
@end
#define VERTEX_COUNT 4
@implementation ViewController

- (SCNGeometry*) fillPC: (std::unique_ptr<summary_map::LocalizationSummaryMap>& )localization_map{

    const Eigen::Matrix3Xf& G_observer_positions =localization_map->GLandmarkPosition();
    int size_byte=  G_observer_positions.cols()*G_observer_positions.rows()*4;
    int vertex_count=G_observer_positions.cols();
    NSData *data = [NSData dataWithBytes:G_observer_positions.data() length:size_byte];
    SCNGeometrySource *vertexSource;
    
    vertexSource = [SCNGeometrySource geometrySourceWithData:data
                                                    semantic:SCNGeometrySourceSemanticVertex
                                                 vectorCount:vertex_count
                                             floatComponents:YES
                                         componentsPerVector:3
                                           bytesPerComponent:4
                                                  dataOffset:0
                                                  dataStride:12];
    
    std::vector<int> indice;
    for(int i=0; i<vertex_count; i++){
        indice.push_back(i);
    }
    NSData *ind_data = [NSData dataWithBytes:indice.data() length:4*vertex_count];
    SCNGeometryElement* vertexInd = [SCNGeometryElement geometryElementWithData:ind_data primitiveType:SCNGeometryPrimitiveTypePoint primitiveCount:vertex_count bytesPerIndex:4];
    vertexInd.minimumPointScreenSpaceRadius = 1;
    vertexInd.maximumPointScreenSpaceRadius=1;
    NSMutableArray *vsourArrar = [[NSMutableArray alloc] init];
    [vsourArrar addObject:vertexSource];
    NSMutableArray *isourArrar = [[NSMutableArray alloc] init];
    [isourArrar addObject:vertexInd];
    SCNGeometry* pc_geo =[SCNGeometry geometryWithSources:vsourArrar elements:isourArrar];
    return pc_geo;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    loadFlags();
    flow.reset(message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(common::getNumHardwareThreads()));
    localization_map.reset(new summary_map::LocalizationSummaryMap);
    if (!localization_map->loadFromFolder(FLAGS_vio_localization_map_folder)) {
        NSLog(@"load map wrong!!");
    }
    
    static constexpr char kSubscriberNodeName[] = "RenderingFlow";
    
    SCNScene *scene = [SCNScene scene];

    // create and add a camera to the scene
    SCNNode *cameraNode = [SCNNode node];
    cameraNode.camera = [SCNCamera camera];
    cameraNode.camera.usesOrthographicProjection=YES;
    [scene.rootNode addChildNode:cameraNode];

    // place the camera
    cameraNode.position = SCNVector3Make(0, 0, 15);
    [cameraNode lookAt:SCNVector3Make(0, 0, 0)];

    SCNNode *pcNode = [SCNNode node];
    pcNode.geometry=[self fillPC: localization_map];
    if (pcNode.geometry== nullptr){
        return;
    }
    [scene.rootNode addChildNode:pcNode];

    meNode = [SCNNode node];
    meNode.geometry = [SCNSphere sphereWithRadius:1];
    [scene.rootNode addChildNode:meNode];
    flow->registerSubscriber<message_flow_topics::VIO_UPDATES>(kSubscriberNodeName, message_flow::DeliveryOptions(), [self](const vio::VioUpdate::ConstPtr& vio_update){
        my_posi=vio_update->vinode.get_T_M_I().getPosition();
    });

    SCNView *scnView = (SCNView *)self.view;
    scnView.scene = scene;
    scnView.allowsCameraControl = YES;
    scnView.showsStatistics = YES;
    scnView.backgroundColor = [UIColor blackColor];
    // add a tap gesture recognizer
    UITapGestureRecognizer *tapGesture = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleTap:)];
    NSMutableArray *gestureRecognizers = [NSMutableArray array];
    [gestureRecognizers addObject:tapGesture];
    [gestureRecognizers addObjectsFromArray:scnView.gestureRecognizers];
    scnView.gestureRecognizers = gestureRecognizers;
    
    aslam::NCamera::Ptr camera_system = aslam::NCamera::loadFromYaml(FLAGS_ncamera_calibration);
    vi_map::Imu::UniquePtr maplab_imu_sensor =vi_map::createFromYaml<vi_map::Imu>(FLAGS_imu_parameters_maplab);
    vi_map::ImuSigmas rovio_imu_sigmas;
    rovio_imu_sigmas.loadFromYaml(FLAGS_imu_parameters_rovio);
    loc_node.reset(new rovioli::RovioliNode(camera_system, std::move(maplab_imu_sensor), rovio_imu_sigmas,FLAGS_save_map_folder, localization_map.get(), flow.get()));
    loc_node->start();
    scnView.delegate = self;
    scnView.playing=YES;
    scnView.preferredFramesPerSecond=15;
}

- (void)renderer:(id<SCNSceneRenderer>)renderer updateAtTime:(NSTimeInterval)time{
    meNode.position = SCNVector3Make(my_posi(0), my_posi(1), my_posi(2));
}

- (void) handleTap:(UIGestureRecognizer*)gestureRecognize
{
    // retrieve the SCNView
    SCNView *scnView = (SCNView *)self.view;
    
    // check what nodes are tapped
    CGPoint p = [gestureRecognize locationInView:scnView];
    NSArray *hitResults = [scnView hitTest:p options:nil];
    
    // check that we clicked on at least one object
    if([hitResults count] > 0){
        // retrieved the first clicked object
        SCNHitTestResult *result = [hitResults objectAtIndex:0];
        
        // get its material
        SCNMaterial *material = result.node.geometry.firstMaterial;
        
        // highlight it
        [SCNTransaction begin];
        [SCNTransaction setAnimationDuration:0.5];
        
        // on completion - unhighlight
        [SCNTransaction setCompletionBlock:^{
            [SCNTransaction begin];
            [SCNTransaction setAnimationDuration:0.5];
            
            material.emission.contents = [UIColor blackColor];
            
            [SCNTransaction commit];
        }];
        
        material.emission.contents = [UIColor redColor];
        
        [SCNTransaction commit];
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

@end
