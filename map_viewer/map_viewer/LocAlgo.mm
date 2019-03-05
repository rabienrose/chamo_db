#include "common_header.h"
#include "LocAlgo.h"
#include <vio-common/vio-types.h>

@implementation LocAlgo
- (void) startAlgo:(std::unique_ptr<message_flow::MessageFlow>&) flow{
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    imuCallback =flow->registerPublisher<message_flow_topics::IMU_MEASUREMENTS>();
    imageCallback =flow->registerPublisher<message_flow_topics::IMAGE_MEASUREMENTS>();
    quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    motionManager = [[CMMotionManager alloc] init];
    if (motionManager.accelerometerAvailable){
        motionManager.accelerometerUpdateInterval =0.01;
        [motionManager
         startAccelerometerUpdatesToQueue:quene
         withHandler:
         ^(CMAccelerometerData *data, NSError *error){
             std::vector<double> imu;
             imu.resize(5);
             imu[0]=-data.acceleration.x*9.8;
             imu[1]=-data.acceleration.y*9.8;
             imu[2]=-data.acceleration.z*9.8;
             imu[3]=data.timestamp;
             imu[4]=0;
             acces.push_back(imu);
         }];
    }
    if (motionManager.gyroAvailable){
        motionManager.gyroUpdateInterval =0.01;
        [motionManager
         startGyroUpdatesToQueue:quene
         withHandler:
         ^(CMGyroData *data, NSError *error){
             std::vector<double> imu;
             imu.resize(5);
             imu[0]=data.rotationRate.x;
             imu[1]=data.rotationRate.y;
             imu[2]=data.rotationRate.z;
             imu[3]=data.timestamp;
             imu[4]=0;
             gyros.push_back(imu);
             [self processIMU_gyro];
         }];
    }
    session = [[AVCaptureSession alloc] init];
    NSError *error = nil;
    [session beginConfiguration];
    session.sessionPreset =AVCaptureSessionPreset640x480;
    videoDevice = [AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInWideAngleCamera mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified];
    AVCaptureDeviceInput *videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
    if ( ! videoDeviceInput ) {
        NSLog( @"Could not create video device input: %@", error );
        [session commitConfiguration];
        return;
    }
    if ( [session canAddInput:videoDeviceInput] ) {
        [session addInput:videoDeviceInput];
        videoDeviceInput = videoDeviceInput;
    }
    else {
        NSLog( @"Could not add video device input to the session" );
        [session commitConfiguration];
        return;
    }
    [videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 10)];
    
    video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    video_output.videoSettings = newSettings;
    [video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([session canAddOutput:video_output]) {
        [session addOutput:video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }
    
    [video_output setSampleBufferDelegate:self queue:sessionQueue];
    
    [session commitConfiguration];
    [session startRunning];
    
}

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

- (void) processIMU_gyro{
    int last_acc_id=-1;
    int last_gyro_id=-1;
    int g_size=gyros.size();
    int a_size=acces.size();
    for(int i=0;i<gyros.size();i++){
        for(int j=0;j<a_size-1;j++){
            if(gyros[i][3]>acces[j][3] && gyros[i][3]<=acces[j+1][3]){
                double x,y,z;
                interDouble(acces[j][0], acces[j+1][0], acces[j][3], acces[j+1][3], x, gyros[i][3]);
                interDouble(acces[j][1], acces[j+1][1], acces[j][3], acces[j+1][3], y, gyros[i][3]);
                interDouble(acces[j][2], acces[j+1][2], acces[j][3], acces[j+1][3], z, gyros[i][3]);
                last_acc_id=j;
                last_gyro_id=i;
                
                vio::ImuMeasurement::Ptr imu_measurement(new vio::ImuMeasurement);
                imu_measurement->timestamp = (int64_t)(gyros[i][3]*1000*1000*1000);
                imu_measurement->imu_data << x, y, z,gyros[i][0], gyros[i][1],gyros[i][2];
                imuCallback(imu_measurement);
                break;
            }
        }
    }
    if(last_acc_id>0){
        if(last_acc_id-1<acces.size()){
            acces.erase(acces.begin(), acces.begin()+last_acc_id);
        }else{
            NSLog(@"test overflow");
        }
    }
    if(last_gyro_id>=0){
        if(last_gyro_id<gyros.size()){
            gyros.erase(gyros.begin(), gyros.begin()+last_gyro_id+1);
        }else{
            NSLog(@"test overflow");
        }
    }
}

- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    CGImageRelease(quartzImage);
    return image;
}
                
- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
    double time_sec = (double)timestamp.value/(double)timestamp.timescale;
    UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
    cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];

    vio::ImageMeasurement::Ptr image_measurement(new vio::ImageMeasurement);
    cvtColor(img_cv, image_measurement->image, cv::COLOR_BGR2GRAY);
    image_measurement->timestamp = (int64_t)(time_sec*1000*1000*1000);
    image_measurement->camera_index = 0;
    imageCallback(image_measurement);
}


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
    
    Eigen::Matrix3Xf colors;
    colors.conservativeResize(colors.rows(), vertex_count);
    for (int i=0; i<colors.cols(); i++){
        Eigen::Vector3f p1(1.0, 1.0, 0.0);
        colors.col(i) = p1;
    }
    NSData *data_color = [NSData dataWithBytes:colors.data() length:size_byte];
    SCNGeometrySource *colorSource;
    colorSource = [SCNGeometrySource geometrySourceWithData:data_color
                                                   semantic:SCNGeometrySourceSemanticColor
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
    [vsourArrar addObject:colorSource];
    NSMutableArray *isourArrar = [[NSMutableArray alloc] init];
    [isourArrar addObject:vertexInd];
    SCNGeometry* pc_geo =[SCNGeometry geometryWithSources:vsourArrar elements:isourArrar];
    return pc_geo;
}

- (SCNGeometry*) buildGird{
    double step=5;
    int count=20;
    float height=2.0;
    Eigen::Matrix3Xf vertice;
    for (int i=-count; i<count; i++){
        Eigen::Vector3f p1(i*step, -step*count, -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p1;
        Eigen::Vector3f p2(i*step, step*count, -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p2;
        
        Eigen::Vector3f p3(-step*count, i*step, -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p3;
        Eigen::Vector3f p4(step*count, i*step , -height);
        vertice.conservativeResize(vertice.rows(), vertice.cols()+1);
        vertice.col(vertice.cols()-1) = p4;
    }
    int vertex_count=vertice.cols();
    int size_byte=vertice.cols()*vertice.rows()*4;
    NSData *data = [NSData dataWithBytes:vertice.data() length:size_byte];
    SCNGeometrySource *vertexSource;
    vertexSource = [SCNGeometrySource geometrySourceWithData:data
                                                    semantic:SCNGeometrySourceSemanticVertex
                                                 vectorCount:vertex_count
                                             floatComponents:YES
                                         componentsPerVector:3
                                           bytesPerComponent:4
                                                  dataOffset:0
                                                  dataStride:12];
    
    Eigen::Matrix3Xf colors;
    colors.conservativeResize(colors.rows(), vertice.cols());
    for (int i=0; i<vertice.cols(); i++){
        Eigen::Vector3f p1(0.1, 0.1, 0.1);
        colors.col(i) = p1;
    }
    NSData *data_color = [NSData dataWithBytes:colors.data() length:size_byte];
    SCNGeometrySource *colorSource;
    colorSource = [SCNGeometrySource geometrySourceWithData:data_color
                                                    semantic:SCNGeometrySourceSemanticColor
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
    SCNGeometryElement* vertexInd = [SCNGeometryElement geometryElementWithData:ind_data primitiveType:SCNGeometryPrimitiveTypeLine primitiveCount:vertex_count bytesPerIndex:4];
    
    NSMutableArray *vsourArrar = [[NSMutableArray alloc] init];
    [vsourArrar addObject:vertexSource];
    [vsourArrar addObject:colorSource];
    NSMutableArray *isourArrar = [[NSMutableArray alloc] init];
    [isourArrar addObject:vertexInd];
    SCNGeometry* gird_geo =[SCNGeometry geometryWithSources:vsourArrar elements:isourArrar];
    return gird_geo;
}





@end
