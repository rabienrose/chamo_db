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
#include "rovioli/rovioli-node.h"

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
    NSString* mybag_str = [myBundle pathForResource:@"ios_test_tiny" ofType:@"bag"];
    FLAGS_datasource_rosbag=[mybag_str UTF8String];
    
    NSString* myalgoconfig_str = [myBundle pathForResource:@"rovio_default_config" ofType:@"info"];
    FLAGS_algorithm_config_file=[myalgoconfig_str UTF8String];
    FLAGS_summary_time=true;
}

@interface ViewController ()
@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    loadFlags();
    self.sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    dispatch_async( self.sessionQueue, ^{
        std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map;
        std::unique_ptr<message_flow::MessageFlow> flow(message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(common::getNumHardwareThreads()));
        aslam::NCamera::Ptr camera_system = aslam::NCamera::loadFromYaml(FLAGS_ncamera_calibration);
        vi_map::Imu::UniquePtr maplab_imu_sensor =vi_map::createFromYaml<vi_map::Imu>(FLAGS_imu_parameters_maplab);
        vi_map::ImuSigmas rovio_imu_sigmas;
        rovio_imu_sigmas.loadFromYaml(FLAGS_imu_parameters_rovio);
        rovioli::RovioliNode rovio_localization_node(camera_system, std::move(maplab_imu_sensor), rovio_imu_sigmas,FLAGS_save_map_folder, localization_map.get(), flow.get());
        rovio_localization_node.start();
        sleep(100000);
//        if (!FLAGS_save_map_folder.empty()) {
//            rovio_localization_node.saveMapAndOptionallyOptimize(FLAGS_save_map_folder, FLAGS_overwrite_existing_map,FLAGS_optimize_map_to_localization_map);
//        }
    } );
}

@end
