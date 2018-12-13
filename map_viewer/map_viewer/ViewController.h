
#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>
#import <SceneKit/SceneKit.h>
#include <message-flow/message-flow.h>
#include <localization-summary-map/localization-summary-map.h>
#include "rovioli/rovioli-node.h"

@interface ViewController : UIViewController<SCNSceneRendererDelegate>{
    std::unique_ptr<message_flow::MessageFlow> flow;
    SCNNode *meNode;
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map;
    Eigen::Vector3d my_posi;
    std::unique_ptr<rovioli::RovioliNode> loc_node;
    
}
    @property (nonatomic) dispatch_queue_t sessionQueue;

@end

