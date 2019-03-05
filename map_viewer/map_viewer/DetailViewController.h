#include "common_header.h"
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <memory>
#include <vector>

static const float kExposureDurationPower = 5;
static const float kExposureMinimumDuration = 1.0/1000;
@protocol DetailViewControllerDelegate <NSObject>
@required
- (int)getCamType;
- (void)setCamType:(int)type;
- (cv::Mat)getNewFrame;
- (AVCaptureDevice *)getDevice;
@end

@interface DetailViewController : UIViewController{
    dispatch_queue_t sessionQueue;
    bool stop_update;
    float cache_focus_posi;
    float cache_exposure_t;
    float cache_iso;
    NSString* exposure_mode;
    NSString* focus_mode;
    NSUserDefaults *defaults;
}
@property (weak, nonatomic) IBOutlet UISegmentedControl *cam_type_switch;
@property (weak, nonatomic) IBOutlet UIImageView *image_view;
@property (nonatomic, weak) id<DetailViewControllerDelegate> delegate;
@property (weak, nonatomic) IBOutlet UISegmentedControl *expo_switch;
@property (weak, nonatomic) IBOutlet UISlider *expo_dura_slider;
@property (nonatomic, weak) IBOutlet UILabel *exposureDurationNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *exposureDurationValueLabel;
@property (nonatomic, weak) IBOutlet UISlider *ISOSlider;
@property (nonatomic, weak) IBOutlet UILabel *ISONameLabel;
@property (nonatomic, weak) IBOutlet UILabel *ISOValueLabel;
@property (nonatomic, weak) IBOutlet UISegmentedControl *focusModeControl;
@property (nonatomic, weak) IBOutlet UISlider *lensPositionSlider;
@property (nonatomic, weak) IBOutlet UILabel *lensPositionNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *lensPositionValueLabel;
@property (nonatomic) NSArray *focusModes;
@property (nonatomic) NSArray *exposureModes;
@property (nonatomic) AVCaptureDevice *videoDevice;
@end
