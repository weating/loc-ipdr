//
//  ViewController.m
//  cig_logger
//
//  Created by Mac on 27/08/2018.
//  Copyright © 2018 Mac. All rights reserved.
//

#import "ViewController.h"
#import <GLKit/GLKit.h>
#import "ipdr.h"
#import <ImageIO/ImageIO.h>
#if __IPHONE_14_0
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#endif

// Private helpers for COLMAP-AR
@interface ViewController ()
- (void)dumpARFrameForColmap:(ARFrame *)frame;
- (BOOL)savePixelBuffer:(CVPixelBufferRef)pb toPath:(NSString *)path quality:(float)q;
- (NSData *)pixelBufferToJPEGData:(CVPixelBufferRef)pb quality:(float)q;
// === PATCHED: Additional COLMAP-AR helpers ===
- (double)computeSharpnessScoreForPixelBuffer:(CVPixelBufferRef)pb;
- (int)cameraIdForIntrinsicsFx:(double)fx fy:(double)fy cx:(double)cx cy:(double)cy W:(int)W H:(int)H;
- (void)writeColmapKnownModel;
@end

@implementation ViewController
{
    // === COLMAP-AR 2FPS output ===
    CIContext *colmapCI;
    NSMutableString *logStringColmapAR;
    double lastColmapTs;
    double colmapInterval;
    dispatch_queue_t colmapQueue;
}
- (void)viewDidLoad {
    [super viewDidLoad];
    NSLog(@"=== ViewDidLoad Starting ===");
    
    FPS = 30;
    imuFreq = 100;
    
    lensPosition = afSlider.value;
    afLabel.text = [NSString stringWithFormat:@"%5.3f",afSlider.value];
    
    dateFormat = [[NSDateFormatter alloc] init];
    [dateFormat setDateFormat:@"yyyy-MM-dd'T'HH-mm-ss"];
    
    locationData = nil;
    headingData = nil;
    
    arSession = nil;
    arConfiguration = nil;
    
    logStringAccel = [NSMutableString stringWithString: @""];
    logStringGyro = [NSMutableString stringWithString: @""];
    logStringMotion = [NSMutableString stringWithString: @""];
    logStringMotARH = [NSMutableString stringWithString: @""];
    logStringMotMagnFull = [NSMutableString stringWithString: @""];
    logStringMagnet = [NSMutableString stringWithString: @""];
    logStringGps = [NSMutableString stringWithString: @""];
    logStringHeading = [NSMutableString stringWithString: @""];
    logStringFrameStamps = [NSMutableString stringWithString: @""];
    logStringArPose = [NSMutableString stringWithString: @""];
    
    fr_height = MAX(self.view.frame.size.width, self.view.frame.size.height);
    fr_width = MIN(self.view.frame.size.width, self.view.frame.size.height);
    
    isRecording = false;
    isStarted = false;
    
    bootTime =  [[NSDate date] timeIntervalSince1970] - [[NSProcessInfo processInfo] systemUptime];
    
    reduseFpsInNTimes = 1;
    prevFrTs = -1.0;
    
    // 创建线程安全机制
    pdrQueue = dispatch_queue_create("com.yourapp.pdr", DISPATCH_QUEUE_SERIAL);
    pdrLock = [[NSLock alloc] init];
    pdrLock.name = @"PDR Lock";
    
    // 初始化缓存变量
    cachedHeading = 0.0;
    cachedPitch = 0.0;
    cachedRoll = 0.0;
    iPDRResultsValid = NO;
    
    // 初始化PDR系统
    NSLog(@"Initializing PDR system with thread safety...");
    accelerationBuffer = [NSMutableArray arrayWithCapacity:10];
    lastStepTime = 0;
    totalDistance = 0;
    currentPosition = CGPointMake(0, 0);
    logStringPDR = [NSMutableString stringWithString:@""];
    
    isWalking = NO;
    walkingStartTime = 0;
    stepCount = 0;
    averageStepLength = 0.7;
    
    // 在专用队列中创建PDR估计器
    dispatch_async(pdrQueue, ^{
        @try {
            self->pdrEstimator = [[iPDRHeadingEstimator alloc] init];
            NSLog(@"PDR estimator created successfully in dedicated queue");
        }
        @catch (NSException *exception) {
            NSLog(@"Error creating PDR estimator: %@", exception.reason);
            self->pdrEstimator = nil;
        }
    });
    
    NSLog(@"PDR system initialized with thread safety");
    
    //--------
    motionManager = [[CMMotionManager alloc] init];
    motionManager.gyroUpdateInterval = 1./imuFreq;
    motionManager.accelerometerUpdateInterval = 1./imuFreq;
    motionManager.deviceMotionUpdateInterval = 1./imuFreq;
    
    NSLog(@"Motion manager created - Gyro available: %@, Accel available: %@, DeviceMotion available: %@",
          motionManager.gyroAvailable ? @"YES" : @"NO",
          motionManager.accelerometerAvailable ? @"YES" : @"NO",
          motionManager.deviceMotionAvailable ? @"YES" : @"NO");
    
    locationManager = [[CLLocationManager alloc] init];
    locationManager.delegate = self;
    locationManager.distanceFilter = kCLDistanceFilterNone;
    locationManager.desiredAccuracy = kCLLocationAccuracyBest;
    
    locationManager.headingOrientation = CLDeviceOrientationLandscapeLeft;
    locationManager.headingFilter = kCLHeadingFilterNone;
    
    if ([locationManager respondsToSelector:@selector(requestWhenInUseAuthorization)]) {
        [locationManager requestWhenInUseAuthorization];
    }

    accelgyroQueue = [[NSOperationQueue alloc] init];
    motionQueue = [[NSOperationQueue alloc] init];
    magnetQueue = [[NSOperationQueue alloc] init];
    
    [self accgyroSwChanged:accgyroSwitch];
    [self gpsheadSwChanged:gpsheadSwitch];
    [self motionSwChanged:motionSwitch];
    [self magnetSwChanged:magnetSwitch];
    
    //******************************
    session = [AVCaptureSession new];

    device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    
    NSError *error = nil;
    input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
    if (!input){
        dispatch_async(dispatch_get_main_queue(), ^{
            UIAlertController * alert = [UIAlertController
                                            alertControllerWithTitle:@"Error"
                                            message:[NSString stringWithFormat:@"AVCaptureDeviceInput: %@", error]
                                            preferredStyle:UIAlertControllerStyleAlert];
            
            UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
            [alert addAction:okButton];
            [self presentViewController:alert animated:YES completion:nil];
        });
    }
    
    [session addInput:input];
    
    output = [AVCaptureVideoDataOutput new];

    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    output.videoSettings = newSettings;
    
    [session addOutput:output];

    sbfQueue = dispatch_queue_create("MyQueue", NULL);
    [output setSampleBufferDelegate:self queue:sbfQueue];
    
    [device lockForConfiguration:nil];

    device.activeVideoMinFrameDuration = CMTimeMake(1, FPS);
    device.activeVideoMaxFrameDuration = CMTimeMake(1, FPS);

    [device setFocusModeLockedWithLensPosition:lensPosition completionHandler:nil];
    [device unlockForConfiguration];
    
    NSString *mediaType = AVMediaTypeVideo;
    [AVCaptureDevice requestAccessForMediaType:mediaType completionHandler:^(BOOL granted) {
        if (!granted){
            dispatch_async(dispatch_get_main_queue(), ^{
                UIAlertController * alert = [UIAlertController
                                             alertControllerWithTitle:@"Error"
                                             message:@"AVCapture doesn't have permission to use camera"
                                             preferredStyle:UIAlertControllerStyleAlert];
                
                UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
                [alert addAction:okButton];
                [self presentViewController:alert animated:YES completion:nil];
            });
        }
    }];
    
    NSArray<__kindof AVCaptureOutput *> *_outputs = session.outputs;
    for (int i = 0; i < [_outputs count]; ++i) {
            NSArray<AVCaptureConnection *> *_connections = _outputs[i].connections;
            for (int j = 0; j < [_connections count]; ++j) {
                if(_connections[j].isCameraIntrinsicMatrixDeliverySupported)
                    _connections[j].cameraIntrinsicMatrixDeliveryEnabled = true;
        }
    }
    
    if(ARWorldTrackingConfiguration.isSupported){
        arConfiguration = [[ARWorldTrackingConfiguration alloc] init];
        arConfiguration.worldAlignment = ARWorldAlignmentGravity;
        NSLog(@"ARKit configuration created successfully");
    }
    else{
        [segmentedControl removeSegmentAtIndex:3 animated:NO];
        NSLog(@"ARKit not supported on this device");
    }
    
    [self segmentedControlValueChanged:segmentedControl];
    
    NSLog(@"=== ViewDidLoad Complete ===");
    // 初始化视觉定位相关变量
    isVisualLocalizationEnabled = NO;
    visualLocalizationTimer = nil;
    logStringVisualLocalization = [NSMutableString stringWithString:@""];
    lastLocalizationSuccess = NO;
    
    // 设置默认定位参数 (可以根据需要修改)
    currentFloor = 26;
    destinationFloor = 26;
    destinationID = 8;
    
    // 初始化状态标签
    localizationStatusLabel.text = @"定位未开启";
    localizationStatusLabel.textColor = [UIColor grayColor];
    
    NSLog(@"Visual localization system initialized");
    // === init COLMAP-AR 2FPS ===
    colmapCI = [CIContext contextWithOptions:nil];
    logStringColmapAR = [NSMutableString string];
    colmapInterval = 0.5;     // 2 FPS
    lastColmapTs = 0.0;
    colmapQueue = dispatch_queue_create("com.yourapp.colmapar", DISPATCH_QUEUE_SERIAL);
}

- (void)viewWillDisappear:(BOOL)animated{
    [super viewWillDisappear:animated];
    [arSession pause];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}
- (IBAction)visualLocalizationSwChanged:(UISwitch *)sender {
    NSLog(@"Visual localization switch changed to: %@", sender.isOn ? @"ON" : @"OFF");
    
    isVisualLocalizationEnabled = sender.isOn;
    
    if (sender.isOn) {
        // 开启视觉定位
        if (visualLocalizationTimer == nil) {
            // 每2秒发送一次定位请求 (对应web端的频率)
            visualLocalizationTimer = [NSTimer scheduledTimerWithTimeInterval:2.0
                                                                        target:self
                                                                      selector:@selector(sendVisualizationRequest)
                                                                      userInfo:nil
                                                                       repeats:YES];
        }
        localizationStatusLabel.text = @"定位已开启";
        localizationStatusLabel.textColor = [UIColor blueColor];
        NSLog(@"Visual localization started with 2s interval");
    } else {
        // 关闭视觉定位
        if (visualLocalizationTimer != nil) {
            [visualLocalizationTimer invalidate];
            visualLocalizationTimer = nil;
        }
        localizationStatusLabel.text = @"定位未开启";
        localizationStatusLabel.textColor = [UIColor grayColor];
        lastLocalizationSuccess = NO;
        NSLog(@"Visual localization stopped");
    }
}
// 发送视觉定位请求
- (void)sendVisualizationRequest {
    if (!isVisualLocalizationEnabled) return;
    
    // 获取当前ARFrame或摄像头画面
    if (segmentedControl.selectedSegmentIndex == 3 && arSession != nil) {
        // ARKit模式：使用ARFrame
        ARFrame *currentFrame = arSession.currentFrame;
        if (currentFrame != nil) {
            [self sendLocalizationRequestWithARFrame:currentFrame];
        }
    } else {
        // 普通摄像头模式：使用当前显示的图像
        UIImage *currentImage = imageView.image;
        if (currentImage != nil) {
            [self sendLocalizationRequestWithImage:currentImage];
        }
    }
}
- (void)sendLocalizationRequestWithARFrame:(ARFrame *)frame {
    //检查开关
        if (!isVisualLocalizationEnabled) {
            return;
        }
        
        //直接处理CVPixelBuffer（不转UIImage）
        CVPixelBufferRef pixelBuffer = frame.capturedImage;
        size_t width = CVPixelBufferGetWidth(pixelBuffer);
        size_t height = CVPixelBufferGetHeight(pixelBuffer);
        
        //转为JPEG（复用colmapar的方法）
        NSData *imageData = [self pixelBufferToJPEGData:pixelBuffer quality:0.85f];
        if (!imageData) return;
        
        NSString *base64String = [imageData base64EncodedStringWithOptions:0];
        
        //获取真实相机内参
        matrix_float3x3 intrinsics = frame.camera.intrinsics;
        double fx = intrinsics.columns[0][0];
        double fy = intrinsics.columns[1][1];
        double cx = intrinsics.columns[2][0];
        double cy = intrinsics.columns[2][1];
        
        //使用原图尺寸和真实内参
        NSDictionary *requestParams = @{
            @"poiId": @8,
            @"timestamp": [NSString stringWithFormat:@"%.0f", [[NSDate date] timeIntervalSince1970] * 1000],
            @"Image": base64String,
            @"currentFloor": @26,
            @"destFloor": @26,
            @"width": @((int)width),
            @"height": @((int)height),
            @"cameraParams": @[@(fx), @(fy), @(cx), @(cy)]
        };
        
        [self sendHTTPRequestWithParams:requestParams];
    }
// 使用UIImage发送定位请求
- (void)sendLocalizationRequestWithImage:(UIImage *)image {
    NSLog(@"=== 详细图像调试 ===");
    NSLog(@"原始图像: %.0fx%.0f, scale:%.1f", image.size.width, image.size.height, image.scale);
    
    CGSize targetSize = CGSizeMake(640, 360);
    UIImage *resizedImage = [self resizeImage:image toSize:targetSize];
    
    NSLog(@"缩放后图像: %.0fx%.0f, scale:%.1f", resizedImage.size.width, resizedImage.size.height, resizedImage.scale);
    
    // 检查CGImage的实际像素尺寸
    CGImageRef cgImage = resizedImage.CGImage;
    size_t pixelWidth = CGImageGetWidth(cgImage);
    size_t pixelHeight = CGImageGetHeight(cgImage);
    NSLog(@"CGImage实际像素: %zux%zu", pixelWidth, pixelHeight);
    
    // 转换为Base64
    NSData *imageData = UIImageJPEGRepresentation(resizedImage, 0.8);
    NSString *base64String = [imageData base64EncodedStringWithOptions:0];
    
    NSLog(@"JPEG数据大小: %.2fKB", imageData.length / 1024.0);
    
    // 验证：重新从JPEG数据创建图像，看看尺寸是否正确
    UIImage *testImage = [UIImage imageWithData:imageData];
    NSLog(@"从JPEG重建的图像: %.0fx%.0f", testImage.size.width, testImage.size.height);
    
    NSDictionary *requestParams = @{
        @"poiId": @8,
        @"timestamp": [NSString stringWithFormat:@"%.0f", [[NSDate date] timeIntervalSince1970] * 1000],
        @"Image": base64String,
        @"currentFloor": @26,
        @"destFloor": @26,
        @"width": @((int)pixelWidth),    // 使用实际像素宽度
        @"height": @((int)pixelHeight),  // 使用实际像素高度
        @"cameraParams": @[@500, @500, @((int)pixelWidth/2), @((int)pixelHeight/2)]
    };
    
    NSLog(@"发送参数: width=%zu, height=%zu", pixelWidth, pixelHeight);
    
    [self sendHTTPRequestWithParams:requestParams];
}
// 发送HTTP请求
- (void)sendHTTPRequestWithParams:(NSDictionary *)params {
    NSURL *url = [NSURL URLWithString:@"https://10.109.253.103:9002/loc/nav/navigation"];
    NSMutableURLRequest *request = [NSMutableURLRequest requestWithURL:url];
    [request setHTTPMethod:@"POST"];
    [request setValue:@"application/json" forHTTPHeaderField:@"Content-Type"];
    [request setValue:@"application/json" forHTTPHeaderField:@"Accept"];
    [request setTimeoutInterval:30.0];
    
    NSError *error;
    NSData *jsonData = [NSJSONSerialization dataWithJSONObject:params options:0 error:&error];
    if (error) {
        NSLog(@"JSON序列化错误: %@", error);
        return;
    }
    
    [request setHTTPBody:jsonData];
    
    NSURLSessionConfiguration *config = [NSURLSessionConfiguration defaultSessionConfiguration];
    NSURLSession *session = [NSURLSession sessionWithConfiguration:config delegate:self delegateQueue:nil];
    
    NSLog(@"发送HTTPS请求到: %@", url.absoluteString);
    
    NSURLSessionDataTask *task = [session dataTaskWithRequest:request
                                            completionHandler:^(NSData *data, NSURLResponse *response, NSError *error) {
        if (error) {
            NSLog(@"请求失败: %@", error.localizedDescription);
            dispatch_async(dispatch_get_main_queue(), ^{
                self->lastLocalizationSuccess = NO;
                self->localizationStatusLabel.text = @"请求失败";
                self->localizationStatusLabel.textColor = [UIColor redColor];
            });
            return;
        }
        
        NSHTTPURLResponse *httpResponse = (NSHTTPURLResponse *)response;
        NSLog(@"响应状态码: %ld", (long)httpResponse.statusCode);
        
        if (data) {
            NSString *responseString = [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding];
            
            // 检查是否是HTTP错误页面
            if (httpResponse.statusCode == 502) {
                NSLog(@"502 Bad Gateway - 后端服务器问题");
                dispatch_async(dispatch_get_main_queue(), ^{
                    self->lastLocalizationSuccess = NO;
                    self->localizationStatusLabel.text = @"后端服务未运行";
                    self->localizationStatusLabel.textColor = [UIColor redColor];
                });
                
                if (self->isRecording) {
                    double msDate = [[NSDate date] timeIntervalSince1970] * 1000;
                    [self->logStringVisualLocalization appendString:[NSString stringWithFormat:@"%f,ERROR,502_BAD_GATEWAY\r\n", msDate]];
                }
                return;
            }
            
            NSLog(@"服务器响应: %@", responseString);
            
            // 检查是否是HTML错误页面
            if ([responseString hasPrefix:@"<html>"] || [responseString hasPrefix:@"<!DOCTYPE"]) {
                NSLog(@"收到HTML错误页面，不是JSON响应");
                dispatch_async(dispatch_get_main_queue(), ^{
                    self->localizationStatusLabel.text = @"服务器错误";
                    self->localizationStatusLabel.textColor = [UIColor orangeColor];
                });
                return;
            }
            
            NSError *jsonError;
            NSDictionary *responseDict = [NSJSONSerialization JSONObjectWithData:data options:0 error:&jsonError];
            if (jsonError) {
                NSLog(@"JSON解析失败: %@", jsonError);
                return;
            }
            
            [self processVisualizationResponse:responseDict];
        }
    }];
    
    [task resume];
}

// 添加SSL证书忽略的delegate方法
- (void)URLSession:(NSURLSession *)session didReceiveChallenge:(NSURLAuthenticationChallenge *)challenge completionHandler:(void (^)(NSURLSessionAuthChallengeDisposition, NSURLCredential *))completionHandler {
    NSLog(@"忽略SSL证书验证，信任服务器");
    NSURLCredential *credential = [NSURLCredential credentialForTrust:challenge.protectionSpace.serverTrust];
    completionHandler(NSURLSessionAuthChallengeUseCredential, credential);
}
// 处理视觉定位响应
- (void)processVisualizationResponse:(NSDictionary *)response {
    dispatch_async(dispatch_get_main_queue(), ^{
        BOOL status = [[response objectForKey:@"status"] boolValue];
        self->lastLocalizationSuccess = status;
        
        if (status) {
            // 定位成功
            self->localizationStatusLabel.text = @"定位成功";
            self->localizationStatusLabel.textColor = [UIColor greenColor];
            
            // 提取位姿信息
            NSDictionary *cameraPose = [response objectForKey:@"cameraPose"];
            if (cameraPose) {
                double x = [[cameraPose objectForKey:@"x"] doubleValue];
                double y = [[cameraPose objectForKey:@"y"] doubleValue];
                double z = [[cameraPose objectForKey:@"z"] doubleValue];
                double qx = [[cameraPose objectForKey:@"qx"] doubleValue];
                double qy = [[cameraPose objectForKey:@"qy"] doubleValue];
                double qz = [[cameraPose objectForKey:@"qz"] doubleValue];
                double qw = [[cameraPose objectForKey:@"qw"] doubleValue];
                
                NSLog(@"Localization successful: pos(%.3f, %.3f, %.3f), quat(%.3f, %.3f, %.3f, %.3f)",
                      x, y, z, qw, qx, qy, qz);
            }
        } else {
            // 定位失败
            self->localizationStatusLabel.text = @"定位失败";
            self->localizationStatusLabel.textColor = [UIColor redColor];
            NSLog(@"Localization failed");
        }
        
        // 记录完整的响应数据
        if (self->isRecording) {
            double msDate = [[NSDate date] timeIntervalSince1970] * 1000;
            NSString *responseString = [[NSString alloc] initWithData:[NSJSONSerialization dataWithJSONObject:response options:0 error:nil] encoding:NSUTF8StringEncoding];
            [self->logStringVisualLocalization appendString:[NSString stringWithFormat:@"%f,%@\r\n", msDate, responseString]];
        }
    });
}

// 图片尺寸调整函数
- (UIImage *)resizeImage:(UIImage *)image toSize:(CGSize)targetSize {
    NSLog(@"resizeImage方法被调用");
    NSLog(@"输入图像: %.0fx%.0f", image.size.width, image.size.height);
    NSLog(@"目标尺寸: %.0fx%.0f", targetSize.width, targetSize.height);
    
    CGSize imageSize = image.size;
    
    // 计算等比例缩放比例
    CGFloat widthRatio = targetSize.width / imageSize.width;
    CGFloat heightRatio = targetSize.height / imageSize.height;
    CGFloat scaleFactor = MIN(widthRatio, heightRatio);
    
    NSLog(@"缩放因子: %.3f", scaleFactor);
    
    // 计算缩放后的实际尺寸
    CGSize scaledSize = CGSizeMake(imageSize.width * scaleFactor, imageSize.height * scaleFactor);
    NSLog(@"计算的缩放尺寸: %.0fx%.0f", scaledSize.width, scaledSize.height);
    
    // 关键修改：强制scale为1.0
    UIGraphicsBeginImageContextWithOptions(scaledSize, NO, 1.0);
    [image drawInRect:CGRectMake(0, 0, scaledSize.width, scaledSize.height)];
    UIImage *resizedImage = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
    
    NSLog(@"最终输出图像: %.0fx%.0f, scale: %.1f", resizedImage.size.width, resizedImage.size.height, resizedImage.scale);
    
    return resizedImage;
}
#pragma mark - 线程安全的iPDR结果获取

- (void)getIPDRResultsWithCompletion:(void(^)(double heading, double pitch, double roll, BOOL valid))completion {
    if ([pdrLock tryLock]) {
        if (pdrEstimator) {
            dispatch_async(pdrQueue, ^{
                @try {
                    double heading = self->pdrEstimator.heading;
                    double pitch = self->pdrEstimator.pitch;
                    double roll = self->pdrEstimator.roll;
                    
                    BOOL valid = isfinite(heading) && isfinite(pitch) && isfinite(roll);
                    
                    // 更新缓存
                    if (valid) {
                        self->cachedHeading = heading;
                        self->cachedPitch = pitch;
                        self->cachedRoll = roll;
                        self->iPDRResultsValid = YES;
                    }
                    
                    completion(heading, pitch, roll, valid);
                    [self->pdrLock unlock];
                }
                @catch (NSException *exception) {
                    NSLog(@"Error getting iPDR results: %@", exception.reason);
                    [self->pdrLock unlock];
                    completion(0.0, 0.0, 0.0, NO);
                }
            });
        } else {
            [pdrLock unlock];
            completion(0.0, 0.0, 0.0, NO);
        }
    } else {
        // 如果锁不可用，返回缓存的结果
        completion(cachedHeading, cachedPitch, cachedRoll, iPDRResultsValid);
    }
}

#pragma mark - 步长检测算法

- (void)processSimpleStepDetection:(CMAccelerometerData *)accelData {
    if (!accelData) return;
    
    // 计算加速度向量的模长
    double accelMagnitude = sqrt(accelData.acceleration.x * accelData.acceleration.x +
                                accelData.acceleration.y * accelData.acceleration.y +
                                accelData.acceleration.z * accelData.acceleration.z);
    
    [accelerationBuffer addObject:@(accelMagnitude)];
    
    if (accelerationBuffer.count > 5) {
        [accelerationBuffer removeObjectAtIndex:0];
    }
    
    // 简单的峰值检测
    if (accelerationBuffer.count >= 3) {
        double prev = [[accelerationBuffer objectAtIndex:accelerationBuffer.count-3] doubleValue];
        double current = [[accelerationBuffer objectAtIndex:accelerationBuffer.count-2] doubleValue];
        double next = [[accelerationBuffer objectAtIndex:accelerationBuffer.count-1] doubleValue];
        
        // 检测峰值 - 调整阈值适应iPhone 13 Pro
        if (current > prev && current > next && current > 1.05 && current < 1.3) {
            NSTimeInterval currentTime = accelData.timestamp;
            
            if (lastStepTime > 0) {
                double timeDiff = currentTime - lastStepTime;
                if (timeDiff > 0.3 && timeDiff < 2.0) {
                    [self recordStep:currentTime];
                }
            } else {
                [self recordStep:currentTime];
            }
            lastStepTime = currentTime;
        }
    }
}

- (void)recordStep:(NSTimeInterval)timestamp {
    stepCount++;
    averageStepLength = 0.7;
    totalDistance += averageStepLength;
    
    // 线程安全地获取当前航向
    [self getIPDRResultsWithCompletion:^(double heading, double pitch, double roll, BOOL valid) {
        // 更新位置
        self->currentPosition.x += self->averageStepLength * cos(heading);
        self->currentPosition.y += self->averageStepLength * sin(heading);
        
        // 记录PDR数据
        if (self->isRecording) {
            double msDate = self->bootTime + timestamp;
            [self->logStringPDR appendString:[NSString stringWithFormat:@"%f,%d,%f,%f,%f,%f,%f\r\n",
                                           msDate,
                                           self->stepCount,
                                           self->averageStepLength,
                                           heading * 180.0 / M_PI,
                                           self->currentPosition.x,
                                           self->currentPosition.y,
                                           self->totalDistance]];
        }
        
        NSLog(@"Step %d recorded - Distance: %.2fm, Heading: %.1f°",
              self->stepCount, self->totalDistance, heading * 180.0 / M_PI);
    }];
    
    // 线程安全地通知PDR算法新步伐
    if ([pdrLock tryLock]) {
        if (pdrEstimator) {
            dispatch_async(pdrQueue, ^{
                @try {
                    [self->pdrEstimator didTakeStepWithLength:self->averageStepLength];
                }
                @catch (NSException *exception) {
                    NSLog(@"Error in PDR step update: %@", exception.reason);
                }
            });
        }
        [pdrLock unlock];
    }
}

#pragma mark - 传感器数据处理方法

-(void)outputRotationData:(CMGyroData *)gyrodata
{
    static int gyroCounter = 0;
    gyroCounter++;
    
    if (gyroCounter == 1) {
        NSLog(@"First gyro data received: (%.3f, %.3f, %.3f)",
              gyrodata.rotationRate.x, gyrodata.rotationRate.y, gyrodata.rotationRate.z);
    }
    
    if (isRecording && gyrodata != nil)
    {
        double msDate = bootTime + gyrodata.timestamp;
        [logStringGyro appendString: [NSString stringWithFormat:@"%f,%f,%f,%f\r\n",
                                      msDate,
                                      gyrodata.rotationRate.x,
                                      gyrodata.rotationRate.y,
                                      gyrodata.rotationRate.z]];
    }
}

-(void)outputAccelertionData:(CMAccelerometerData *)acceldata
{
    static int accelCounter = 0;
    accelCounter++;
    
    if (accelCounter == 1) {
        NSLog(@"First accel data received: (%.3f, %.3f, %.3f)",
              acceldata.acceleration.x, acceldata.acceleration.y, acceldata.acceleration.z);
    }
    
    // 记录原始加速度数据
    if (isRecording && acceldata != nil)
    {
        double msDate = bootTime + acceldata.timestamp;
        [logStringAccel appendString: [NSString stringWithFormat:@"%f,%f,%f,%f\r\n",
                                       msDate,
                                       acceldata.acceleration.x,
                                       acceldata.acceleration.y,
                                       acceldata.acceleration.z]];
    }
    
    // 步长检测
    [self processSimpleStepDetection:acceldata];
}

-(void)outputMagnetometerData:(CMMagnetometerData *)magnetdata
{
    static int magnetCounter = 0;
    magnetCounter++;
    
    if (magnetCounter == 1) {
        NSLog(@"First magnet data received: (%.3f, %.3f, %.3f)",
              magnetdata.magneticField.x, magnetdata.magneticField.y, magnetdata.magneticField.z);
    }
    
    if (isRecording && magnetdata != nil)
    {
        double msDate = bootTime + magnetdata.timestamp;
        [logStringMagnet appendString: [NSString stringWithFormat:@"%f,%f,%f,%f\r\n",
                                       msDate,
                                       magnetdata.magneticField.x,
                                       magnetdata.magneticField.y,
                                       magnetdata.magneticField.z]];
    }
}

-(void)outputDeviceMotionData:(CMDeviceMotion *)devmotdata
{
    static int motionCounter = 0;
    motionCounter++;
    
    if (!devmotdata) {
        NSLog(@"Warning: devmotdata is nil at call %d", motionCounter);
        return;
    }
    
    // 获取传感器数据
    CMRotationRate rotr = devmotdata.rotationRate;
    CMAcceleration grav = devmotdata.gravity;
    CMAcceleration usracc = devmotdata.userAcceleration;
    CMCalibratedMagneticField calmagnfield = devmotdata.magneticField;
    
    if (motionCounter == 1) {
        NSLog(@"First DeviceMotion data received:");
        NSLog(@"  Gyro: (%.3f,%.3f,%.3f)", rotr.x, rotr.y, rotr.z);
        NSLog(@"  Gravity: (%.3f,%.3f,%.3f)", grav.x, grav.y, grav.z);
        NSLog(@"  UserAccel: (%.3f,%.3f,%.3f)", usracc.x, usracc.y, usracc.z);
        NSLog(@"  Magnet: (%.3f,%.3f,%.3f)", calmagnfield.field.x, calmagnfield.field.y, calmagnfield.field.z);
        NSLog(@"  SystemHeading: %.2f", devmotdata.heading);
    }
    
    // 默认值
    double systemHeading = devmotdata.heading;
    double finalHeading = systemHeading;
    double finalPitch = 0.0;
    double finalRoll = 0.0;
    
    // 线程安全地更新iPDR
    if ([pdrLock tryLock]) {
        if (pdrEstimator) {
            // 异步更新iPDR，但不等待结果
            dispatch_async(pdrQueue, ^{
                @try {
                    [self->pdrEstimator updateWithDeviceMotion:devmotdata rawMagnetometer:nil];
                    
                    double iPDR_heading = self->pdrEstimator.heading;
                    double iPDR_pitch = self->pdrEstimator.pitch;
                    double iPDR_roll = self->pdrEstimator.roll;
                    
                    // 验证结果
                    if (isfinite(iPDR_heading) && isfinite(iPDR_pitch) && isfinite(iPDR_roll)) {
                        // 更新缓存
                        self->cachedHeading = iPDR_heading;
                        self->cachedPitch = iPDR_pitch;
                        self->cachedRoll = iPDR_roll;
                        self->iPDRResultsValid = YES;
                        
                        if (motionCounter % 100 == 0) {
                            NSLog(@"iPDR update %d - Heading: %.2f°, Pitch: %.2f°, Roll: %.2f°",
                                  motionCounter,
                                  iPDR_heading * 180.0 / M_PI,
                                  iPDR_pitch * 180.0 / M_PI,
                                  iPDR_roll * 180.0 / M_PI);
                        }
                    } else {
                        NSLog(@"iPDR returned invalid values at call %d", motionCounter);
                        self->iPDRResultsValid = NO;
                    }
                }
                @catch (NSException *exception) {
                    NSLog(@"Error in iPDR processing at call %d: %@", motionCounter, exception.reason);
                    self->iPDRResultsValid = NO;
                }
            });
        } else if (motionCounter % 100 == 0) {
            NSLog(@"PDR estimator is nil at call %d", motionCounter);
        }
        [pdrLock unlock];
    } else if (motionCounter % 100 == 0) {
        NSLog(@"PDR lock busy at call %d", motionCounter);
    }
    
    // 使用缓存的iPDR结果（如果可用）
    if (iPDRResultsValid) {
        finalHeading = cachedHeading * 180.0 / M_PI;
        finalPitch = cachedPitch * 180.0 / M_PI;
        finalRoll = cachedRoll * 180.0 / M_PI;
    }
    
    // 记录数据
    if (isRecording && devmotdata != nil)
    {
        double msDate = bootTime + devmotdata.timestamp;
        
        CMQuaternion quat = devmotdata.attitude.quaternion;
        [logStringMotion appendString: [NSString stringWithFormat:@"%f,%f,%f,%f,%f\r\n",
                                       msDate,
                                        quat.w,
                                        quat.x,
                                        quat.y,
                                        quat.z]];
        
        [logStringMotARH appendString: [NSString stringWithFormat:@"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
                                       msDate,
                                        rotr.x,
                                        rotr.y,
                                        rotr.z,
                                        grav.x,
                                        grav.y,
                                        grav.z,
                                        usracc.x,
                                        usracc.y,
                                        usracc.z,
                                        systemHeading,
                                        finalHeading]];
        
        [logStringMotMagnFull appendString: [NSString stringWithFormat:@"%f,%f,%f,%f,%d\r\n",
                                       msDate,
                                        calmagnfield.field.x,
                                        calmagnfield.field.y,
                                        calmagnfield.field.z,
                                        calmagnfield.accuracy]];
    }
}

#pragma mark - CLLocationManagerDelegate

- (void)locationManager:(CLLocationManager *)manager didUpdateLocations:(NSArray *)locations
{
    locationData = [locations lastObject];
    [self updateLocation:locationData];
}

-(void)updateLocation:(CLLocation *)location
{
    if (isRecording && location != nil)
    {
        double currLatitude = location.coordinate.latitude;
        double currLongitude = location.coordinate.longitude;
        double currHorAccur = location.horizontalAccuracy;
        double currAltitude = location.altitude;
        double currVertAccur = location.verticalAccuracy;
        long currFloor = location.floor.level;
        double currCource = location.course;
        double currSpeed = location.speed;
        
        double msDate = [location.timestamp timeIntervalSince1970];
        [logStringGps appendString: [NSString stringWithFormat:@"%f,%f,%f,%f,%f,%f,%ld,%f,%f\r\n",
                          msDate,
                          currLatitude,
                          currLongitude,
                          currHorAccur,
                          currAltitude,
                          currVertAccur,
                          currFloor,
                          currCource,
                          currSpeed]];
    }
}

- (void) locationManager:(CLLocationManager *)manager didUpdateHeading:(CLHeading *)newHeading
{
    headingData = newHeading;
    [self updateHeading:headingData];
}

-(void)updateHeading:(CLHeading *)heading
{
    if (isRecording && heading != nil)
    {
        double currTrueHeading = heading.trueHeading;
        double currMagneticHeading = heading.magneticHeading;
        double currHeadingAccuracy = heading.headingAccuracy;
        
        double msDate = [heading.timestamp timeIntervalSince1970];
        [logStringHeading appendString: [NSString stringWithFormat:@"%f,%f,%f,%f\r\n",
                          msDate,
                          currTrueHeading,
                          currMagneticHeading,
                          currHeadingAccuracy]];
    }
}

- (void)locationManager:(CLLocationManager *)manager didFailWithError:(NSError *)error
{
    UIAlertController * alert = [UIAlertController
                                 alertControllerWithTitle:@"Error"
                                 message:[NSString stringWithFormat:@"Location update: %@", error]
                                 preferredStyle:UIAlertControllerStyleAlert];
    
    UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
    [alert addAction:okButton];
    [self presentViewController:alert animated:YES completion:nil];
}

#pragma mark - ARSession和AVCapture处理

- (void)session:(ARSession *)session didUpdateFrame:(ARFrame *)frame {
    // === 2 FPS strictly-aligned dump ===
    // ✅ 修复：添加 isRecording 检查，确保只在录制时保存colmapar数据
    if (isRecording && segmentedControl.selectedSegmentIndex == 3) {
        double _ts = frame.timestamp; // seconds since boot
        if (_ts - lastColmapTs >= colmapInterval) {
            lastColmapTs = _ts;
            [self dumpARFrameForColmap:frame];
        }
    }
    
    double msDate = bootTime + frame.timestamp;
    
    if(prevFrTs >= 0)
        if(reduseFpsInNTimes * (msDate - prevFrTs) < (1.0/FPS)/1.5)
            reduseFpsInNTimes++;
    prevFrTs = msDate;
    
    if(ireduceFps == 0)
    {
        matrix_float3x3 camMat = frame.camera.intrinsics;
        [self processImage:frame.capturedImage Timestamp:msDate CameraMatrix:&camMat];
    }
    if(ireduceFps == (reduseFpsInNTimes-1))
        ireduceFps = 0;
    else
        ++ireduceFps;
    
    // ✅ 修复：ARPose也应该只在录制时记录
    if(isRecording) {
        simd_float4x4 trans = frame.camera.transform;
        simd_quatf quat = (simd_quaternion(trans));
        [logStringArPose appendString: [NSString stringWithFormat:@"%f,%f,%f,%f,%f,%f,%f,%f\r\n",
                                        msDate,
                                        trans.columns[3][0], trans.columns[3][1], trans.columns[3][2],
                                        quat.vector[3], quat.vector[0], quat.vector[1], quat.vector[2]
                                        ]];
    }
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection
{
    matrix_float3x3 *camMatrix = nullptr;
    
    if(isRecording || isStarted)
    {
        if(connection.isCameraIntrinsicMatrixDeliveryEnabled)
        {
            CFTypeRef cameraIntrinsicData = CMGetAttachment(sampleBuffer,
                                                            kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix,
                                                            nil);
            
            if(cameraIntrinsicData != nil)
                if (CFGetTypeID(cameraIntrinsicData) == CFDataGetTypeID()) {
                    CFDataRef cfdr = (CFDataRef)(cameraIntrinsicData);
                    camMatrix = (matrix_float3x3 *)(CFDataGetBytePtr(cfdr));
                }
        }
    }
    
    CMTime frameTime = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
    double msDate = bootTime + CMTimeGetSeconds(frameTime);
    
    CVPixelBufferRef buffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    
    CVPixelBufferLockBaseAddress(buffer, 0);
    [self processImage:buffer Timestamp:msDate CameraMatrix:camMatrix];
    CVPixelBufferUnlockBaseAddress(buffer, 0);
}

- (void)processImage:(CVPixelBufferRef)pixelBuffer Timestamp:(double)msDate CameraMatrix:(matrix_float3x3*)camMat
{
    if(isStarted)
    {
        [self updateLocation:locationData];
        [self updateHeading:headingData];
        
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsDirectory = [paths objectAtIndex:0];
        NSString *filePath = [[NSString alloc] initWithString:[NSString stringWithFormat:@"%@/%@/Frames.m4v", documentsDirectory, theDate]];
        
        NSError *error = nil;
        NSURL *outputURL = [NSURL fileURLWithPath:filePath];
        assetWriter = [AVAssetWriter assetWriterWithURL:outputURL fileType:AVFileTypeAppleM4V error:&error];
        if (!assetWriter) {
            UIAlertController * alert = [UIAlertController alertControllerWithTitle:@"Error" message:[NSString stringWithFormat:@"assetWriter: %@", error] preferredStyle:UIAlertControllerStyleAlert];
            UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
            [alert addAction:okButton];
            [self presentViewController:alert animated:YES completion:nil];
        }
        
        NSDictionary *writerInputParams = [NSDictionary dictionaryWithObjectsAndKeys:
                                           AVVideoCodecTypeHEVC, AVVideoCodecKey,
                                                     [NSNumber numberWithInt:(int)CVPixelBufferGetWidthOfPlane(pixelBuffer,0)], AVVideoWidthKey,
                                                     [NSNumber numberWithInt:(int)CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)], AVVideoHeightKey,
                                                     AVVideoScalingModeResizeAspectFill, AVVideoScalingModeKey,
                                                     nil];
           
        assetWriterInput = [AVAssetWriterInput assetWriterInputWithMediaType:AVMediaTypeVideo outputSettings:writerInputParams];
        if ([assetWriter canAddInput:assetWriterInput]) {
            [assetWriter addInput:assetWriterInput];
        } else {
            UIAlertController * alert = [UIAlertController alertControllerWithTitle:@"Error" message:[NSString stringWithFormat:@"assetWriter can't AddInput: %@", assetWriter.error] preferredStyle:UIAlertControllerStyleAlert];
            UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
            [alert addAction:okButton];
            [self presentViewController:alert animated:YES completion:nil];
        }
        
        assetWriterInputPixelBufferAdaptor = [AVAssetWriterInputPixelBufferAdaptor assetWriterInputPixelBufferAdaptorWithAssetWriterInput:assetWriterInput sourcePixelBufferAttributes:nil];
        
        [assetWriter startWriting];
        [assetWriter startSessionAtSourceTime:kCMTimeZero];
        
        isStarted = NO;
        isRecording = YES;
    }
    
    if(isRecording)
    {
        if(assetWriterInput.isReadyForMoreMediaData)
        {
            if(![assetWriterInputPixelBufferAdaptor appendPixelBuffer:pixelBuffer withPresentationTime:CMTimeMake(frameNum, FPS)])
                NSLog(@"assetWriterInput cant appendPixelBuffer!");
            else
            {
                if(camMat != nullptr){
                    [logStringFrameStamps appendString: [NSString stringWithFormat:@"%f,%lu,%f,%f,%f,%f\r\n",
                                                         msDate,
                                                         frameNum,
                                                         camMat->columns[0][0],
                                                         camMat->columns[1][1],
                                                         camMat->columns[2][0],
                                                         camMat->columns[2][1]]];
                }
                else{
                    [logStringFrameStamps appendString: [NSString stringWithFormat:@"%f,%lu\r\n",
                                                         msDate,
                                                         frameNum]];
                }
                
                frameNum += 1;
            }
        }
        else
            NSLog(@"assetWriterInput.isReadyForMoreMediaData = NO!");
    }

    CIImage *ciimage = [CIImage imageWithCVImageBuffer:pixelBuffer];
    UIImage *img = [UIImage imageWithCIImage:ciimage];
    dispatch_async(dispatch_get_main_queue(), ^{
        self->imageView.image = img;
    });
}

#pragma mark - UI事件处理

- (void)_timerFired:(NSTimer *)timer {
    iTimer += 1;
    NSString *timeStr = [NSString stringWithFormat:@"%.2d:%.2d",iTimer/60,iTimer%60];
    runTimeLabel.text = timeStr;
}

- (IBAction)toggleButton:(id)sender
{
    if (!isRecording && !isStarted)
    {
        NSLog(@"=== STARTING RECORDING ===");
        NSLog(@"Current segment: %ld", (long)segmentedControl.selectedSegmentIndex);
        NSLog(@"Switches - Motion: %@, AccelGyro: %@, Magnet: %@, GPS: %@, VisualLoc: %@",
              motionSwitch.isOn ? @"ON" : @"OFF",
              accgyroSwitch.isOn ? @"ON" : @"OFF",
              magnetSwitch.isOn ? @"ON" : @"OFF",
              gpsheadSwitch.isOn ? @"ON" : @"OFF",
              visualLocalizationSwitch.isOn ? @"ON" : @"OFF");
        
        // 重置所有状态
        stepCount = 0;
        totalDistance = 0;
        currentPosition = CGPointMake(0, 0);
        lastStepTime = 0;
        [accelerationBuffer removeAllObjects];
        
        // 重置缓存
        cachedHeading = 0.0;
        cachedPitch = 0.0;
        cachedRoll = 0.0;
        iPDRResultsValid = NO;
        
        // 重置视觉定位状态
        lastLocalizationSuccess = NO;
        
        // 线程安全地重新创建PDR估计器
        [pdrLock lock];
        dispatch_sync(pdrQueue, ^{
            @try {
                self->pdrEstimator = [[iPDRHeadingEstimator alloc] init];
                NSLog(@"PDR estimator recreated for recording");
            }
            @catch (NSException *exception) {
                NSLog(@"Error recreating PDR estimator: %@", exception.reason);
                self->pdrEstimator = nil;
            }
        });
        [pdrLock unlock];
        
        isStarted = YES;
        
        if (!_timer) {
            _timer = [NSTimer scheduledTimerWithTimeInterval:1.
                                                      target:self
                                                    selector:@selector(_timerFired:)
                                                    userInfo:nil
                                                     repeats:YES];
        }
        iTimer = 0;
        
        runTimeLabel.text = @"00:00";
        afButton.enabled = NO;
        if(!isAf){
            afSlider.enabled = NO;
            afLabel.enabled = NO;
        }
        segmentedControl.enabled = NO;
        accgyroSwitch.enabled = NO;
        gpsheadSwitch.enabled = NO;
        motionSwitch.enabled = NO;
        magnetSwitch.enabled = NO;
        visualLocalizationSwitch.enabled = NO;  // 禁用视觉定位开关
        
        // 清空所有日志
        [logStringAccel setString:@""];
        [logStringGyro setString:@""];
        [logStringMotion setString:@""];
        [logStringMotARH setString:@""];
        [logStringMotMagnFull setString:@""];
        [logStringMagnet setString:@""];
        [logStringGps setString:@""];
        [logStringHeading setString:@""];
        [logStringFrameStamps setString:@""];
        [logStringArPose setString:@""];
        [logStringPDR setString:@""];
        [logStringVisualLocalization setString:@""];  // 清空视觉定位日志
        
        frameNum = 0;
        ireduceFps = 0;
        
        NSDate *now = [[NSDate alloc] init];
        theDate = [dateFormat stringFromDate:now];
        
        [self createFolderInDocuments:theDate];
        // Prepare colmapar folder
        {
            NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSString *documentsDirectory = [paths objectAtIndex:0];
            NSString *colmapDir = [documentsDirectory stringByAppendingPathComponent:[NSString stringWithFormat:@"%@/colmapar", theDate]];
            [[NSFileManager defaultManager] createDirectoryAtPath:colmapDir
                                      withIntermediateDirectories:YES
                                                       attributes:nil
                                                            error:nil];
            [logStringColmapAR setString:@""];
            lastColmapTs = 0.0;
        }
        
        [sender setTitle:@"STOP" forState:UIControlStateNormal];
        
        NSLog(@"Recording started, date folder: %@", theDate);
    }
    else if (!isStarted)
    {
        NSLog(@"=== STOPPING RECORDING ===");
        NSLog(@"Final stats - Steps: %d, Distance: %.2fm", stepCount, totalDistance);
        
        isRecording = NO;
        
        if ([_timer isValid]) {
            [_timer invalidate];
        }
        _timer = nil;
        
        // 恢复UI
        afButton.enabled = YES;
        if(!isAf){
            afSlider.enabled = YES;
            afLabel.enabled = YES;
        }
        segmentedControl.enabled = YES;
        accgyroSwitch.enabled = YES;
        gpsheadSwitch.enabled = YES;
        motionSwitch.enabled = YES;
        magnetSwitch.enabled = YES;
        visualLocalizationSwitch.enabled = YES;  // 重新启用视觉定位开关
        
        // 保存所有文件
        if(accgyroSwitch.isOn){
            BOOL gyroSaved = [self writeStringToFile:logStringGyro FileName:@"Gyro"];
            BOOL accelSaved = [self writeStringToFile:logStringAccel FileName:@"Accel"];
            NSLog(@"Gyro saved: %@ (length: %lu), Accel saved: %@ (length: %lu)",
                  gyroSaved ? @"YES" : @"NO", (unsigned long)logStringGyro.length,
                  accelSaved ? @"YES" : @"NO", (unsigned long)logStringAccel.length);
        }
        if(motionSwitch.isOn){
            BOOL motionSaved = [self writeStringToFile:logStringMotion FileName:@"Motion"];
            BOOL motARHSaved = [self writeStringToFile:logStringMotARH FileName:@"MotARH"];
            BOOL motMagnSaved = [self writeStringToFile:logStringMotMagnFull FileName:@"MotMagnFull"];
            NSLog(@"Motion saved: %@ (length: %lu)", motionSaved ? @"YES" : @"NO", (unsigned long)logStringMotion.length);
            NSLog(@"MotARH saved: %@ (length: %lu)", motARHSaved ? @"YES" : @"NO", (unsigned long)logStringMotARH.length);
            NSLog(@"MotMagn saved: %@ (length: %lu)", motMagnSaved ? @"YES" : @"NO", (unsigned long)logStringMotMagnFull.length);
        }
        if(magnetSwitch.isOn){
            BOOL magnetSaved = [self writeStringToFile:logStringMagnet FileName:@"Magnet"];
            NSLog(@"Magnet saved: %@ (length: %lu)", magnetSaved ? @"YES" : @"NO", (unsigned long)logStringMagnet.length);
        }
        if(gpsheadSwitch.isOn){
            [self writeStringToFile:logStringGps FileName:@"GPS"];
            [self writeStringToFile:logStringHeading FileName:@"Head"];
        }
        [self writeStringToFile:logStringFrameStamps FileName:@"Frames"];
        if(segmentedControl.selectedSegmentIndex == 3)
            [self writeStringToFile:logStringArPose FileName:@"ARposes"];
        
        // 保存PDR数据
        BOOL pdrSaved = [self writeStringToFile:logStringPDR FileName:@"PDR"];
        NSLog(@"PDR saved: %@ (length: %lu)", pdrSaved ? @"YES" : @"NO", (unsigned long)logStringPDR.length);
        
        // 输出PDR数据的前几行用于调试
        if (logStringPDR.length > 0) {
            NSArray *lines = [logStringPDR componentsSeparatedByString:@"\r\n"];
            NSLog(@"PDR data sample (first 3 lines):");
            for (int i = 0; i < MIN(3, lines.count); i++) {
                if (((NSString*)lines[i]).length > 0) {
                    NSLog(@"  %@", lines[i]);
                }
            }
        }
        
        // 保存视觉定位数据
        if(visualLocalizationSwitch.isOn){
            BOOL visualLocSaved = [self writeStringToFile:logStringVisualLocalization FileName:@"VisualLocalization"];
            NSLog(@"Visual Localization saved: %@ (length: %lu)",
                  visualLocSaved ? @"YES" : @"NO", (unsigned long)logStringVisualLocalization.length);
            
            // 输出视觉定位数据的前几行用于调试
            if (logStringVisualLocalization.length > 0) {
                NSArray *lines = [logStringVisualLocalization componentsSeparatedByString:@"\r\n"];
                NSLog(@"Visual Localization data sample (first 3 lines):");
                for (int i = 0; i < MIN(3, lines.count); i++) {
                    if (((NSString*)lines[i]).length > 0) {
                        NSLog(@"  %@", lines[i]);
                    }
                }
            }
        }
        
        // 视频处理
        // Write colmapar/colmapar.txt (aligned image→pose)
        {
            NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSString *documentsDirectory = [paths objectAtIndex:0];
            NSString *colmapDir = [documentsDirectory stringByAppendingPathComponent:[NSString stringWithFormat:@"%@/colmapar", theDate]];
            [[NSFileManager defaultManager] createDirectoryAtPath:colmapDir
                                      withIntermediateDirectories:YES
                                                       attributes:nil
                                                            error:nil];
            NSString *trajPath = [colmapDir stringByAppendingPathComponent:@"colmapar.txt"];
            [[logStringColmapAR dataUsingEncoding:NSUTF8StringEncoding] writeToFile:trajPath atomically:YES];
            NSLog(@"COLMAP-AR wrote trajectory: %@", trajPath);
            NSArray *lines = [logStringColmapAR componentsSeparatedByString:@"\r\n"];
            NSInteger imageCount = 0;
            for (NSString *line in lines) {
                if (line.length > 0) {
                    imageCount++;
                }
            }
            NSLog(@"COLMAP-AR saved %ld images at 2 FPS", (long)imageCount);
        }
        [self->assetWriterInput markAsFinished];
        [self->assetWriter finishWritingWithCompletionHandler:^{
            if (self->assetWriter.error) {
                NSLog(@"Video save error: %@", self->assetWriter.error);
                UIAlertController * alert = [UIAlertController alertControllerWithTitle:@"Error" message:[NSString stringWithFormat:@"assetWriter: %@", self->assetWriter.error] preferredStyle:UIAlertControllerStyleAlert];
                UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
                [alert addAction:okButton];
                [self presentViewController:alert animated:YES completion:nil];
            } else {
                NSLog(@"Video saved successfully");
            }
        }];
        
        self->assetWriter = nil;
        self->assetWriterInput = nil;
        self->assetWriterInputPixelBufferAdaptor = nil;

        [sender setTitle:@"START" forState:UIControlStateNormal];
    }
}

- (IBAction)toggleAfButton:(id)sender {
    if(!isAf){
        if(segmentedControl.selectedSegmentIndex == 3)
        {
            arSession = [ARSession new];
            arSession.delegate = self;
            if (@available(iOS 11.3, *)) {
                arConfiguration.autoFocusEnabled = true;
            }
            [arSession runWithConfiguration:arConfiguration];
            NSLog(@"ARSession started with autofocus ON");
        }
        else
        {
            afSlider.enabled = NO;
            afLabel.hidden = YES;
            
            [device lockForConfiguration:nil];
            device.focusMode = AVCaptureFocusModeContinuousAutoFocus;
            [device unlockForConfiguration];
        }
        
        isAf = YES;
        [sender setTitle:@"AF:ON" forState:UIControlStateNormal];
    }
    else{
        if(segmentedControl.selectedSegmentIndex == 3)
        {
            arSession = [ARSession new];
            arSession.delegate = self;
            if (@available(iOS 11.3, *)) {
                arConfiguration.autoFocusEnabled = false;
            }
            [arSession runWithConfiguration:arConfiguration];
            NSLog(@"ARSession started with autofocus OFF");
        }
        else
        {
            afSlider.enabled = YES;
            afLabel.hidden = NO;
            
            [device lockForConfiguration:nil];
            [device setFocusModeLockedWithLensPosition:lensPosition completionHandler:nil];
            [device unlockForConfiguration];
        }
        
        isAf = NO;
        [sender setTitle:@"AF:OFF" forState:UIControlStateNormal];
    }
}

- (IBAction)afSliderValueChanged:(id)sender {
    afLabel.text = [NSString stringWithFormat:@"%5.3f",afSlider.value];
}

- (IBAction)afSliderEndEditing:(id)sender {
    lensPosition = afSlider.value;
    [device lockForConfiguration:nil];
    [device setFocusModeLockedWithLensPosition:lensPosition completionHandler:nil];
    [device unlockForConfiguration];
}

- (IBAction)segmentedControlValueChanged:(UISegmentedControl *)sender {
    NSLog(@"Segment changed to: %ld", (long)sender.selectedSegmentIndex);
    
    switch(sender.selectedSegmentIndex)
    {
        case 0:
            reduseFpsInNTimes = 1;
            if(arSession != nil)
            {
                arSession = nil;
                afSlider.hidden = NO;
                afLabel.hidden = NO;
                if((device.focusMode == AVCaptureFocusModeLocked) && isAf){
                    isAf = !isAf;
                    [self toggleAfButton:afButton];
                }
            }
            if([session canSetSessionPreset:AVCaptureSessionPreset640x480])
                session.sessionPreset = AVCaptureSessionPreset640x480;
            if(![session isRunning])
            {
                [session startRunning];
            }
            NSLog(@"Switched to 640x480 camera mode");
            break;
        case 1:
            reduseFpsInNTimes = 1;
            if(arSession != nil)
            {
                arSession = nil;
                afSlider.hidden = NO;
                afLabel.hidden = NO;
                if((device.focusMode == AVCaptureFocusModeLocked) && isAf){
                    isAf = !isAf;
                    [self toggleAfButton:afButton];
                }
            }
            if([session canSetSessionPreset:AVCaptureSessionPreset1280x720])
                session.sessionPreset = AVCaptureSessionPreset1280x720;
            if(![session isRunning])
            {
                [session startRunning];
            }
            NSLog(@"Switched to 1280x720 camera mode");
            break;
        case 2:
            reduseFpsInNTimes = 1;
            if(arSession != nil)
            {
                arSession = nil;
                afSlider.hidden = NO;
                afLabel.hidden = NO;
                if((device.focusMode == AVCaptureFocusModeLocked) && isAf){
                    isAf = !isAf;
                    [self toggleAfButton:afButton];
                }
            }
            if([session canSetSessionPreset:AVCaptureSessionPreset1920x1080])
                session.sessionPreset = AVCaptureSessionPreset1920x1080;
            if(![session isRunning])
            {
                [session startRunning];
            }
            NSLog(@"Switched to 1920x1080 camera mode");
            break;
        case 3:
            afSlider.hidden = YES;
            afLabel.hidden = YES;
            if([session isRunning])
                    [session stopRunning];
            isAf = !isAf;
            [self toggleAfButton:afButton];
            NSLog(@"Switched to ARKit mode");
            break;
        default: sender.selectedSegmentIndex = UISegmentedControlNoSegment;
    }
}

#pragma mark - 传感器开关处理

- (IBAction)accgyroSwChanged:(UISwitch *)sender {
    NSLog(@"AccelGyro switch changed to: %@", sender.isOn ? @"ON" : @"OFF");
    
    if ( sender.isOn ){
        [motionManager startGyroUpdatesToQueue:accelgyroQueue
                                   withHandler:^(CMGyroData *gyroData, NSError *error) {
                                       [self outputRotationData:gyroData];
                                       if(error){
                                           NSLog(@"Gyro error: %@", error);
                                           UIAlertController * alert = [UIAlertController
                                                                        alertControllerWithTitle:@"Error"
                                                                        message:[NSString stringWithFormat:@"Gyroscope update: %@", error]
                                                                        preferredStyle:UIAlertControllerStyleAlert];
                                           
                                           UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
                                           [alert addAction:okButton];
                                           [self presentViewController:alert animated:YES completion:nil];
                                       }
                                   }];
        
        [motionManager startAccelerometerUpdatesToQueue:accelgyroQueue
                                            withHandler:^(CMAccelerometerData *accelerometerData, NSError *error) {
                                                [self outputAccelertionData:accelerometerData];
                                                if(error){
                                                    NSLog(@"Accel error: %@", error);
                                                    UIAlertController * alert = [UIAlertController
                                                                                 alertControllerWithTitle:@"Error"
                                                                                 message:[NSString stringWithFormat:@"Accelerometer update: %@", error]
                                                                                 preferredStyle:UIAlertControllerStyleAlert];
                                                    
                                                    UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
                                                    [alert addAction:okButton];
                                                    [self presentViewController:alert animated:YES completion:nil];
                                                }
                                            }];
        NSLog(@"Started accelerometer and gyroscope updates");
    }
    else{
        [motionManager stopAccelerometerUpdates];
        [motionManager stopGyroUpdates];
        NSLog(@"Stopped accelerometer and gyroscope updates");
    }
}

- (IBAction)gpsheadSwChanged:(UISwitch *)sender {
    NSLog(@"GPS/Head switch changed to: %@", sender.isOn ? @"ON" : @"OFF");
    
    if ( sender.isOn ){
         [locationManager startUpdatingLocation];
         
         if([CLLocationManager headingAvailable])
             [locationManager startUpdatingHeading];
    }
    else{
        [locationManager stopUpdatingLocation];
        
        if([CLLocationManager headingAvailable])
            [locationManager stopUpdatingHeading];
    }
}

- (IBAction)motionSwChanged:(UISwitch *)sender {
    NSLog(@"Motion switch changed to: %@", sender.isOn ? @"ON" : @"OFF");
    
    if ( sender.isOn ){
        [motionManager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXTrueNorthZVertical toQueue:motionQueue
                                    withHandler:^(CMDeviceMotion *devmotData, NSError *error) {
                                        [self outputDeviceMotionData:devmotData];
                                        if(error){
                                            NSLog(@"DeviceMotion error: %@", error);
                                            UIAlertController * alert = [UIAlertController
                                                                         alertControllerWithTitle:@"Error"
                                                                         message:[NSString stringWithFormat:@"DeviceMotion update: %@", error]
                                                                         preferredStyle:UIAlertControllerStyleAlert];
                                            
                                            UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
                                            [alert addAction:okButton];
                                            [self presentViewController:alert animated:YES completion:nil];
                                        }
        }];
        NSLog(@"Started device motion updates");
    }
    else{
        [motionManager stopDeviceMotionUpdates];
        NSLog(@"Stopped device motion updates");
    }
}

- (IBAction)magnetSwChanged:(UISwitch *)sender {
    NSLog(@"Magnet switch changed to: %@", sender.isOn ? @"ON" : @"OFF");
    
    if ( sender.isOn ){
        [motionManager startMagnetometerUpdatesToQueue:magnetQueue
                                            withHandler:^(CMMagnetometerData *magnetData, NSError *error) {
                                                [self outputMagnetometerData:magnetData];
                                                if(error){
                                                    NSLog(@"Magnetometer error: %@", error);
                                                    UIAlertController * alert = [UIAlertController
                                                                                 alertControllerWithTitle:@"Error"
                                                                                 message:[NSString stringWithFormat:@"Magnetometer update: %@", error]
                                                                                 preferredStyle:UIAlertControllerStyleAlert];
                                                    
                                                    UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
                                                    [alert addAction:okButton];
                                                    [self presentViewController:alert animated:YES completion:nil];
                                                }
                                            }];
    }
    else{
        [motionManager stopMagnetometerUpdates];
    }
}

#pragma mark - 文件操作

-(BOOL) writeStringToFile:(NSMutableString *)aString FileName:(NSString *)nameString
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    
    NSString *filePath= [[NSString alloc] initWithString:[NSString stringWithFormat:@"%@/%@/%@.txt",documentsDirectory, theDate, nameString]];
    
    BOOL success = [[aString dataUsingEncoding:NSUTF8StringEncoding] writeToFile:filePath atomically:YES];
    
    return success;
}

-(BOOL) createFolderInDocuments:(NSString *)folderName
{
    NSError *error = nil;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *dataPath = [documentsDirectory stringByAppendingPathComponent:folderName];
    
    BOOL success = YES;
    if (![[NSFileManager defaultManager] fileExistsAtPath:dataPath])
        success = [[NSFileManager defaultManager] createDirectoryAtPath:dataPath withIntermediateDirectories:NO attributes:nil error:&error];
    
    if(error){
        UIAlertController * alert = [UIAlertController
                                     alertControllerWithTitle:@"Error"
                                     message:[NSString stringWithFormat:@"Create folder: %@", error]
                                     preferredStyle:UIAlertControllerStyleAlert];
        
        UIAlertAction* okButton = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action){}];
        [alert addAction:okButton];
        [self presentViewController:alert animated:YES completion:nil];
    }
    
    return success;
}


// Save CVPixelBuffer → JPEG (keep sensor resolution/orientation)
- (BOOL)savePixelBuffer:(CVPixelBufferRef)pb toPath:(NSString *)path quality:(float)q {
    if (!pb) return NO;
    if (!colmapCI) colmapCI = [CIContext contextWithOptions:nil];
    CIImage *ci = [CIImage imageWithCVPixelBuffer:pb options:nil];
    size_t W = CVPixelBufferGetWidth(pb), H = CVPixelBufferGetHeight(pb);
    CGImageRef cg = [colmapCI createCGImage:ci fromRect:CGRectMake(0, 0, W, H)];
    if (!cg) return NO;

    CFStringRef jpegUTI = CFSTR("public.jpeg");

    CGImageDestinationRef dst =
        CGImageDestinationCreateWithURL((__bridge CFURLRef)[NSURL fileURLWithPath:path],
                                        jpegUTI,
                                        1, NULL);
    NSDictionary *opts = @{(__bridge NSString*)kCGImageDestinationLossyCompressionQuality: @(q)};
    CGImageDestinationAddImage(dst, cg, (__bridge CFDictionaryRef)opts);
    BOOL ok = CGImageDestinationFinalize(dst);
    if (dst) CFRelease(dst);
    CGImageRelease(cg);
    return ok;
}

// Dump a single ARFrame (image + pose) strictly aligned into colmapar/
- (void)dumpARFrameForColmap:(ARFrame *)frame {
    // filenames by ns timestamp
    long long ts_ns = (long long) llround(frame.timestamp * 1e9);
    NSString *imgName = [NSString stringWithFormat:@"f_%lld.jpg", ts_ns];

    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *colmapDir = [documentsDirectory stringByAppendingPathComponent:[NSString stringWithFormat:@"%@/colmapar", theDate]];
    [[NSFileManager defaultManager] createDirectoryAtPath:colmapDir
                              withIntermediateDirectories:YES
                                               attributes:nil
                                                    error:nil];
    NSString *imgPath = [colmapDir stringByAppendingPathComponent:imgName];

    // Pose: ARKit gives T_wc (camera→world). Convert to world→camera and fix axes to COLMAP (X right, Y down, Z forward)
    simd_float4x4 T_wc = frame.camera.transform;
    simd_float3 Cw = (simd_float3){ T_wc.columns[3][0], T_wc.columns[3][1], T_wc.columns[3][2] };
    simd_float3x3 R_wc = (simd_float3x3){
        (simd_float3){ T_wc.columns[0][0], T_wc.columns[0][1], T_wc.columns[0][2] },
        (simd_float3){ T_wc.columns[1][0], T_wc.columns[1][1], T_wc.columns[1][2] },
        (simd_float3){ T_wc.columns[2][0], T_wc.columns[2][1], T_wc.columns[2][2] },
    };
    // world→camera (pre axis-fix)
    simd_float3x3 R_cw = simd_transpose(R_wc);
    simd_float3  t_cw = - simd_mul(R_cw, Cw);
    // Axis fix: S = diag(1,-1,-1)
    simd_float3x3 S = (simd_float3x3){
        (simd_float3){ 1,  0,  0},
        (simd_float3){ 0, -1,  0},
        (simd_float3){ 0,  0, -1},
    };
    R_cw = simd_mul(S, R_cw);
    t_cw = simd_mul(S, t_cw);

    simd_quatf q = simd_quaternion(R_cw);
    double qw = q.vector[3], qx = q.vector[0], qy = q.vector[1], qz = q.vector[2];

    // IO offload
    CVPixelBufferRef pb = frame.capturedImage;
    CVPixelBufferRetain(pb);
    dispatch_async(colmapQueue, ^{
        [self savePixelBuffer:pb toPath:imgPath quality:0.85f];
        [self->logStringColmapAR appendFormat:@"%@ %.9f %.9f %.9f %.9f %.9f %.9f %.9f\r\n",
             imgName, qw, qx, qy, qz, t_cw.x, t_cw.y, t_cw.z];
        CVPixelBufferRelease(pb);
    });
}
- (NSData *)pixelBufferToJPEGData:(CVPixelBufferRef)pb quality:(float)q {
    if (!pb) return nil;
    if (!colmapCI) colmapCI = [CIContext contextWithOptions:nil];
    
    CIImage *ci = [CIImage imageWithCVPixelBuffer:pb options:nil];
    size_t W = CVPixelBufferGetWidth(pb), H = CVPixelBufferGetHeight(pb);
    CGImageRef cg = [colmapCI createCGImage:ci fromRect:CGRectMake(0, 0, W, H)];
    if (!cg) return nil;
    
    NSMutableData *imageData = [NSMutableData data];
    CGImageDestinationRef dst = CGImageDestinationCreateWithData(
        (__bridge CFMutableDataRef)imageData, CFSTR("public.jpeg"), 1, NULL);
    if (!dst) { CGImageRelease(cg); return nil; }
    
    NSDictionary *opts = @{(__bridge NSString*)kCGImageDestinationLossyCompressionQuality: @(q)};
    CGImageDestinationAddImage(dst, cg, (__bridge CFDictionaryRef)opts);
    BOOL ok = CGImageDestinationFinalize(dst);
    
    CFRelease(dst);
    CGImageRelease(cg);
    return ok ? imageData : nil;
}
@end
