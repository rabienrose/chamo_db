#import "ViewController.h"
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include "aslam/cameras/ncamera.h"
#include "gflags/gflags.h"

DEFINE_bool(summary_time, false, "")
@interface ViewController ()
@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    NSBundle* myBundle = [NSBundle mainBundle];
    NSString* mycam_str = [myBundle pathForResource:@"ncamera-euroc" ofType:@"yaml"];
    aslam::NCamera::Ptr camera_system = aslam::NCamera::loadFromYaml([mycam_str UTF8String]);
    std::cout<<camera_system->get_T_C_B_Mutable(0)<<std::endl;
  
}

@end
