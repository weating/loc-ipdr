//
//  ViewController.h
//  cig_logger
//
//  Created by Mac on 27/08/2018.
//  Copyright © 2018 Mac. All rights reserved.
//

#import <CoreMotion/CoreMotion.h>
#import <CoreLocation/CoreLocation.h>
#import <AVFoundation/AVFoundation.h>
#import <ARKit/ARKit.h>

// 前向声明iPDRHeadingEstimator类
@class iPDRHeadingEstimator;
// 在ViewController.h中修改interface声明，添加NSURLSessionDelegate
@interface ViewController : UIViewController <AVCaptureVideoDataOutputSampleBufferDelegate, CLLocationManagerDelegate, ARSessionDelegate, NSURLSessionDelegate>
{
    __weak IBOutlet UIImageView *imageView;
    __weak IBOutlet UIButton *button;
    __weak IBOutlet UILabel *runTimeLabel;
    __weak IBOutlet UIButton *afButton;
    __weak IBOutlet UILabel *afLabel;
    __weak IBOutlet UISlider *afSlider;
    __weak IBOutlet UISegmentedControl *segmentedControl;
    __weak IBOutlet UISwitch *accgyroSwitch;
    __weak IBOutlet UISwitch *gpsheadSwitch;
    __weak IBOutlet UISwitch *motionSwitch;
    __weak IBOutlet UISwitch *magnetSwitch;
    
    AVCaptureSession *session;
    AVCaptureDevice *device;
    AVCaptureDeviceInput *input;
    AVCaptureVideoDataOutput *output;
    dispatch_queue_t sbfQueue;
    
    ARSession *arSession;
    ARWorldTrackingConfiguration *arConfiguration;
    
    CALayer *viewLayer;
    AVCaptureVideoPreviewLayer *captureVideoPreviewLayer;
    
    CMMotionManager *motionManager;
    CLLocationManager *locationManager;
    
    CLLocation *locationData;
    CLHeading *headingData;
    
    bool isRecording;
    bool isStarted;
    bool isAf;
    
    NSString *theDate;
    NSDateFormatter *dateFormat;
    
    double FPS;
    double imuFreq;
    
    NSOperationQueue *accelgyroQueue;
    NSOperationQueue *motionQueue;
    NSOperationQueue *magnetQueue;
    
    NSMutableString  *logStringAccel;
    NSMutableString  *logStringGyro;
    NSMutableString  *logStringMotion; //mot
    NSMutableString  *logStringMotARH; //mot
    NSMutableString  *logStringMotMagnFull; //mot
    NSMutableString  *logStringMagnet; //mag
    NSMutableString  *logStringGps; //gps
    NSMutableString  *logStringHeading; //gps
    NSMutableString  *logStringFrameStamps;
    NSMutableString  *logStringArPose; //arkit
    
    AVAssetWriter *assetWriter;
    AVAssetWriterInput *assetWriterInput;
    AVAssetWriterInputPixelBufferAdaptor *assetWriterInputPixelBufferAdaptor;
    unsigned long frameNum;
    
    float lensPosition;
    
    //----------------
    CGFloat fr_height;
    CGFloat fr_width;
    
    NSTimeInterval bootTime;
    
    NSTimer *_timer;
    int iTimer;
    
    int ireduceFps;
    int reduseFpsInNTimes;
    
    double prevFrTs;
    
    // 新增PDR相关变量
    iPDRHeadingEstimator *pdrEstimator;
    NSMutableArray<NSNumber *> *accelerationBuffer;
    NSTimeInterval lastStepTime;
    double totalDistance;
    CGPoint currentPosition;
    NSMutableString *logStringPDR; // 记录PDR结果
    
    // 步长检测相关
    BOOL isWalking;
    NSTimeInterval walkingStartTime;
    int stepCount;
    double averageStepLength;
    
    dispatch_queue_t pdrQueue;
    NSLock *pdrLock;
    
    // iPDR结果缓存
    double cachedHeading;
    double cachedPitch;
    double cachedRoll;
    BOOL iPDRResultsValid;
    
    // 新增视觉定位相关属性
    __weak IBOutlet UISwitch *visualLocalizationSwitch;
    __weak IBOutlet UILabel *localizationStatusLabel;
    
    BOOL isVisualLocalizationEnabled;
    NSTimer *visualLocalizationTimer;
    NSMutableString *logStringVisualLocalization;
        
        // 定位参数 (对应web端的参数)
    NSInteger currentFloor;
    NSInteger destinationFloor;
    NSInteger destinationID;
        
    BOOL lastLocalizationSuccess;
}

- (IBAction)toggleButton:(id)sender;
- (IBAction)toggleAfButton:(id)sender;
- (IBAction)afSliderValueChanged:(id)sender;
- (IBAction)afSliderEndEditing:(id)sender;
- (IBAction)segmentedControlValueChanged:(UISegmentedControl *)sender;
- (IBAction)accgyroSwChanged:(UISwitch *)sender;
- (IBAction)gpsheadSwChanged:(UISwitch *)sender;
- (IBAction)motionSwChanged:(UISwitch *)sender;
- (IBAction)magnetSwChanged:(UISwitch *)sender;
- (IBAction)visualLocalizationSwChanged:(UISwitch *)sender;
- (void)sendVisualizationRequest;
- (void)processVisualizationResponse:(NSDictionary *)response;
- (UIImage *)resizeImage:(UIImage *)image toSize:(CGSize)size;

@end
