#include <message-flow/message-flow.h>
#include <deque>
#include <memory>
#include <vector>
#import <QuartzCore/QuartzCore.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>
#include "rovioli/flow-topics.h"
#include <localization-summary-map/localization-summary-map.h>
#import <SceneKit/SceneKit.h>

@interface LocAlgo:NSObject<AVCaptureVideoDataOutputSampleBufferDelegate>{
    CMMotionManager *motionManager;
    dispatch_queue_t sessionQueue;
    NSOperationQueue *quene;
    AVCaptureSession *session;
    AVCaptureDeviceInput *videoDeviceInput;
    AVCaptureDeviceDiscoverySession *videoDeviceDiscoverySession;
    AVCaptureDevice *videoDevice;
    AVCaptureVideoDataOutput *video_output;
    std::vector<std::vector<double> > gyros;
    std::vector<std::vector<double> > acces;

    std::function<void(vio::ImageMeasurement::Ptr)> imageCallback;
    std::function<void(vio::ImuMeasurement::Ptr)> imuCallback;
    
}
- (void) startAlgo:(std::unique_ptr<message_flow::MessageFlow>&) flow;
- (SCNGeometry*) fillPC: (std::unique_ptr<summary_map::LocalizationSummaryMap>& )localization_map;
- (SCNGeometry*) buildGird;
    
@end
