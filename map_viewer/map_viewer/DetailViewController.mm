#import "DetailViewController.h"

@implementation DetailViewController

- (void)viewDidLoad
{
	[super viewDidLoad];
    int cam_type=[self.delegate getCamType];
    self.cam_type_switch.selectedSegmentIndex = cam_type;
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    stop_update=true;
}

- (void)viewDidAppear:(BOOL)animated{
    stop_update=false;
    dispatch_async( sessionQueue, ^{
        while(!stop_update){
            cv::Mat img= [self.delegate getNewFrame];
            if(!img.empty()){
                UIImage *ui_image;
                ui_image = [mm_Try UIImageFromCVMat:img];
                dispatch_async( dispatch_get_main_queue(), ^{
                    self.image_view.image=ui_image;
                });
            }
            [NSThread sleepForTimeInterval:0.01];
        }
    });
}
- (void)viewWillDisappear:(BOOL)animated{
    stop_update=true;
}

- (void) loadConfig{
    defaults = [NSUserDefaults standardUserDefaults];
    focus_mode = [defaults objectForKey:@"focus_mode"];
    if (focus_mode==nil){
        focus_mode=@"auto";
    }
    
    exposure_mode = [defaults objectForKey:@"exposure_mode"];
    if (exposure_mode==nil){
        exposure_mode=@"auto";
    }
    
    NSString* focus_posi_ns = [defaults objectForKey:@"focus_posi"];
    if (focus_posi_ns==nil){
        cache_focus_posi=-1;
    }else{
        cache_focus_posi = atof(std::string([focus_posi_ns UTF8String]).c_str());
    }
    
    NSString* exposure_ns = [defaults objectForKey:@"exposure_t"];
    if (exposure_ns==nil){
        cache_exposure_t=-1;
    }else{
        cache_exposure_t = atof(std::string([exposure_ns UTF8String]).c_str());
    }
    
    NSString* iso_ns = [defaults objectForKey:@"iso"];
    if (iso_ns==nil){
        cache_iso=-1;
    }else{
        cache_iso = atof(std::string([iso_ns UTF8String]).c_str());
    }
    
}

- (void)configureManualHUD
{
    // Manual focus controls
    self.focusModes = @[@(AVCaptureFocusModeContinuousAutoFocus), @(AVCaptureFocusModeLocked)];
    
    self.focusModeControl.enabled = ( self.videoDevice != nil );
    self.focusModeControl.selectedSegmentIndex = [self.focusModes indexOfObject:@(self.videoDevice.focusMode)];
    for ( NSNumber *mode in self.focusModes ) {
        [self.focusModeControl setEnabled:[self.videoDevice isFocusModeSupported:(AVCaptureFocusMode)mode.intValue] forSegmentAtIndex:[self.focusModes indexOfObject:mode]];
    }
    
    
    self.lensPositionSlider.minimumValue = 0.0;
    self.lensPositionSlider.maximumValue = 1.0;
    if (cache_focus_posi<0){
        self.lensPositionSlider.value = self.videoDevice.lensPosition;
    }else{
        self.lensPositionSlider.value=cache_focus_posi;
    }
    std::cout<<"lensPosition: "<<self.lensPositionSlider.value<<std::endl;
    self.lensPositionSlider.enabled = ( self.videoDevice && self.videoDevice.focusMode == AVCaptureFocusModeLocked && [self.videoDevice isFocusModeSupported:AVCaptureFocusModeLocked] );
    
    // Manual exposure controls
    self.exposureModes = @[@(AVCaptureExposureModeContinuousAutoExposure), @(AVCaptureExposureModeLocked), @(AVCaptureExposureModeCustom)];
    
    
    self.expo_switch.enabled = ( self.videoDevice != nil );
    self.expo_switch.selectedSegmentIndex = [self.exposureModes indexOfObject:@(self.videoDevice.exposureMode)];
    for ( NSNumber *mode in self.exposureModes ) {
        [self.expo_switch setEnabled:[self.videoDevice isExposureModeSupported:(AVCaptureExposureMode)mode.intValue] forSegmentAtIndex:[self.exposureModes indexOfObject:mode]];
    }
    
    // Use 0-1 as the slider range and do a non-linear mapping from the slider value to the actual device exposure duration
    self.expo_dura_slider.minimumValue = 0;
    self.expo_dura_slider.maximumValue = 1;
    double exposureDurationSeconds;
    if(cache_exposure_t<0){
        exposureDurationSeconds = CMTimeGetSeconds( self.videoDevice.exposureDuration );
    }else{
        exposureDurationSeconds = cache_exposure_t;
    }
    
    double minExposureDurationSeconds = MAX( CMTimeGetSeconds( self.videoDevice.activeFormat.minExposureDuration ), kExposureMinimumDuration );
    double maxExposureDurationSeconds = CMTimeGetSeconds( self.videoDevice.activeFormat.maxExposureDuration );
    // Map from duration to non-linear UI range 0-1
    double p = ( exposureDurationSeconds - minExposureDurationSeconds ) / ( maxExposureDurationSeconds - minExposureDurationSeconds ); // Scale to 0-1
    self.expo_dura_slider.value = pow( p, 1 / kExposureDurationPower ); // Apply inverse power
    self.expo_dura_slider.enabled = ( self.videoDevice && self.videoDevice.exposureMode == AVCaptureExposureModeCustom );
    std::cout<<"exposure time: "<<exposureDurationSeconds<<std::endl;
    self.ISOSlider.minimumValue = self.videoDevice.activeFormat.minISO;
    self.ISOSlider.maximumValue = self.videoDevice.activeFormat.maxISO;
    if(cache_iso<0){
        self.ISOSlider.value = self.videoDevice.ISO;
    }else{
        self.ISOSlider.value=cache_iso;
    }
    
    std::cout<<"iso: "<<self.ISOSlider.value<<std::endl;
    self.ISOSlider.enabled = ( self.videoDevice.exposureMode == AVCaptureExposureModeCustom );
    
}


- (IBAction)expo_mode_change:(id)sender {
    UISegmentedControl *control = sender;
    AVCaptureExposureMode mode = (AVCaptureExposureMode)[self.exposureModes[control.selectedSegmentIndex] intValue];
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        if ( [self.videoDevice isExposureModeSupported:mode] ) {
            self.videoDevice.exposureMode = mode;
            if (mode==AVCaptureExposureModeCustom){
                if(cache_exposure_t>0 && cache_iso>0){
                    [self.videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( cache_exposure_t, 1000*1000*1000 ) ISO:cache_iso completionHandler:nil];
                }else{
                    if(cache_iso>0){
                        [self.videoDevice setExposureModeCustomWithDuration:AVCaptureExposureDurationCurrent ISO:cache_iso completionHandler:nil];
                    }
                    if(cache_exposure_t>0){
                        [self.videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( cache_exposure_t, 1000*1000*1000 ) ISO:AVCaptureISOCurrent completionHandler:nil];
                    }
                }
                exposure_mode=@"custom";
            }else if(mode==AVCaptureExposureModeContinuousAutoExposure){
                exposure_mode=@"auto";
            }else if(mode==AVCaptureExposureModeLocked){
                exposure_mode=@"lock";
            }else{
            }
            [defaults setObject:exposure_mode forKey:@"exposure_mode"];
            [defaults synchronize];
        }
        else {
            NSLog( @"Exposure mode %@ is not supported. Exposure mode is %@.", [self stringFromExposureMode:mode], [self stringFromExposureMode:self.videoDevice.exposureMode] );
            self.expo_switch.selectedSegmentIndex = [self.exposureModes indexOfObject:@(self.videoDevice.exposureMode)];
        }
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}


- (NSString *)stringFromFocusMode:(AVCaptureFocusMode)focusMode
{
    NSString *string = @"INVALID FOCUS MODE";
    
    if ( focusMode == AVCaptureFocusModeLocked ) {
        string = @"Locked";
    }
    else if ( focusMode == AVCaptureFocusModeAutoFocus ) {
        string = @"Auto";
    }
    else if ( focusMode == AVCaptureFocusModeContinuousAutoFocus ) {
        string = @"ContinuousAuto";
    }
    
    return string;
}

- (NSString *)stringFromExposureMode:(AVCaptureExposureMode)exposureMode
{
    NSString *string = @"INVALID EXPOSURE MODE";
    
    if ( exposureMode == AVCaptureExposureModeLocked ) {
        string = @"Locked";
    }
    else if ( exposureMode == AVCaptureExposureModeAutoExpose ) {
        string = @"Auto";
    }
    else if ( exposureMode == AVCaptureExposureModeContinuousAutoExposure ) {
        string = @"ContinuousAuto";
    }
    else if ( exposureMode == AVCaptureExposureModeCustom ) {
        string = @"Custom";
    }
    
    return string;
}


- (IBAction)changeFocusMode:(id)sender
{
    UISegmentedControl *control = sender;
    AVCaptureFocusMode mode = (AVCaptureFocusMode)[self.focusModes[control.selectedSegmentIndex] intValue];
    
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        if ( [self.videoDevice isFocusModeSupported:mode] ) {
            self.videoDevice.focusMode = mode;
            if (mode==AVCaptureFocusModeLocked){
                if(cache_focus_posi>0){
                    [self.videoDevice setFocusModeLockedWithLensPosition:cache_focus_posi completionHandler:nil];
                }
                focus_mode=@"custom";
            }else if(mode==AVCaptureFocusModeContinuousAutoFocus){
                focus_mode=@"auto";
            }else{
            }
            [defaults setObject:focus_mode forKey:@"focus_mode"];
            [defaults synchronize];
        }
        else {
            NSLog( @"Focus mode %@ is not supported. Focus mode is %@.", [self stringFromFocusMode:mode], [self stringFromFocusMode:self.videoDevice.focusMode] );
            self.focusModeControl.selectedSegmentIndex = [self.focusModes indexOfObject:@(self.videoDevice.focusMode)];
        }
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeLensPosition:(id)sender
{
    UISlider *control = sender;
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        [self.videoDevice setFocusModeLockedWithLensPosition:control.value completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
        cache_focus_posi=control.value;
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)sliderTouchEnded:(id)sender
{
    UISlider *slider = (UISlider *)sender;
    if ( slider == self.lensPositionSlider ) {
        std::stringstream ss;
        ss<<cache_focus_posi;
        NSString *temp_ss = [NSString stringWithCString:ss.str().c_str() encoding:[NSString defaultCStringEncoding]];
        [defaults setObject:temp_ss forKey:@"focus_posi"];
        [defaults synchronize];
    }
    else if ( slider == self.expo_dura_slider ) {
        std::stringstream ss;
        ss<<cache_exposure_t;
        NSString *temp_ss = [NSString stringWithCString:ss.str().c_str() encoding:[NSString defaultCStringEncoding]];
        [defaults setObject:temp_ss forKey:@"exposure_t"];
        [defaults synchronize];
    }
    else if ( slider == self.ISOSlider ) {
        std::stringstream ss;
        ss<<cache_iso;
        NSString *temp_ss = [NSString stringWithCString:ss.str().c_str() encoding:[NSString defaultCStringEncoding]];
        [defaults setObject:temp_ss forKey:@"iso"];
        [defaults synchronize];
    }
}

- (IBAction)exit_btn:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}

- (IBAction)can_type_change:(id)sender {
    UISegmentedControl *control = (UISegmentedControl *)sender;
    [self.delegate setCamType:control.selectedSegmentIndex];
}


- (IBAction)changeExposureDuration:(id)sender
{
    UISlider *control = sender;
    NSError *error = nil;
    
    double p = pow( control.value, kExposureDurationPower ); // Apply power function to expand slider's low-end range
    double minDurationSeconds = MAX( CMTimeGetSeconds( self.videoDevice.activeFormat.minExposureDuration ), kExposureMinimumDuration );
    double maxDurationSeconds = CMTimeGetSeconds( self.videoDevice.activeFormat.maxExposureDuration );
    double newDurationSeconds = p * ( maxDurationSeconds - minDurationSeconds ) + minDurationSeconds; // Scale from 0-1 slider range to actual duration
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        [self.videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( newDurationSeconds, 1000*1000*1000 )  ISO:AVCaptureISOCurrent completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
        cache_exposure_t=newDurationSeconds;
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeISO:(id)sender
{
    UISlider *control = sender;
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        [self.videoDevice setExposureModeCustomWithDuration:AVCaptureExposureDurationCurrent ISO:control.value completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
        cache_iso=control.value;
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}



@end
