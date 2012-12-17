//
//  ViewController.m
//  logMagandAccel
//
//  Created by Robert Ritchie on 11/24/12.
//  Copyright (c) 2012 Robert Ritchie. All rights reserved.
//

#import "ViewController.h"
#import <Accelerate/Accelerate.h>

@interface ViewController ()

@property (nonatomic, strong) CMMotionManager *motionManager;
@property(nonatomic, strong) NSOperationQueue *filterQue;

@end

@implementation ViewController

@synthesize xLabel, yLabel, zLabel, doneLabel;
@synthesize motionManager;
@synthesize filterQue;

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
    testRunning = NO;
    
    // Stop the screen from turning off
    [[UIApplication sharedApplication] setIdleTimerDisabled:YES];
    
    // Create a thread for our filters
    self.filterQue = [[NSOperationQueue alloc] init];
    
    initialized = NO;
}


- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void)viewWillDisappear:(BOOL)animated{
    
    NSLog(@"ViewWillDisappear");
    
    [[UIApplication sharedApplication] setIdleTimerDisabled:NO];
    [self.motionManager stopGyroUpdates];

}

// called when start button pressed
- (IBAction)startPressed:(UIButton *)sender {
    
    if (!testRunning && initializeStarted) {
        
        
        [self startDataCollectionWithInterval:5.0];

        testRunning = YES;
    }
    

}

// this function starts the motion manager and puts the devices on their own threads
-(void)startDataCollectionWithInterval:(double) interval{
    
    self.motionManager = [[CMMotionManager alloc] init];
    
    
    self.motionManager.gyroUpdateInterval = interval;
    self.motionManager.accelerometerUpdateInterval = interval;
    self.motionManager.deviceMotionUpdateInterval = interval;
    
    
    [self.motionManager startAccelerometerUpdatesToQueue:[[NSOperationQueue alloc] init] withHandler:^(CMAccelerometerData *accelerometerData, NSError *error) {
        
        aX = accelerometerData.acceleration.x;
        aY = accelerometerData.acceleration.y;
        aZ = accelerometerData.acceleration.z;
        aT = accelerometerData.timestamp;
        
        // send message to self to start a new filter 
        if (initialized) {
            [self newDataAvail];
        }else{
            
            [self getMagAndGrav];
        }
        
        
    }];
    NSOperationQueue *opQueue3 = [[NSOperationQueue alloc] init];
    
    [self.motionManager startMagnetometerUpdatesToQueue:opQueue3 withHandler:^(CMMagnetometerData *magnetometerData, NSError *error){
        
        mX = magnetometerData.magneticField.x;
        mY = magnetometerData.magneticField.y;
        mZ = magnetometerData.magneticField.z;
        mT = magnetometerData.timestamp;
        

        
    }];
}

-(void)newDataAvail{
    
    GLKVector3 magVec = GLKVector3Make(mX, mY, mZ);
    GLKVector3 accelVec = GLKVector3Make(aX, aY, aZ);
    magVec = GLKVector3Normalize(magVec);
    accelVec = GLKVector3Normalize(accelVec);
    
    fprintf(filePointMag, "%f\t%f\t%f\n",mX,mY,mZ);
    fprintf(filePointAccel, "%f\t%f\t%f\n",aX,aY,aZ);
    
    //NSLog(@"mag %f %f %f acc %f %f %f",aX,aY,aZ,mX,mY,mZ );
    //NSLog(@"time difference %f",(mT-aT));
    
    // Create a new filter and send it off to thread
    Filter *newFilter = [[Filter alloc] initWithVectors:magVec :accelVec andLeftMatrix:Lm delegate:self];
    [self.filterQue addOperation:newFilter];
}

// This is the delegate method called back on ourselves once a filter finishes
-(void)filterDidFinish:(GLKVector3)magVec :(GLKVector3)accelVec{
    
    NSLog(@"inVC %f",magVec.x);
    fprintf(filePointAtt, "%f\t%f\t%f\n",magVec.x,magVec.y,magVec.z);
dispatch_async(dispatch_get_main_queue(), ^{
    self.xLabel.text = [NSString stringWithFormat:@"%f",magVec.x];
    self.yLabel.text = [NSString stringWithFormat:@"%f",magVec.y];
    self.zLabel.text = [NSString stringWithFormat:@"%f",magVec.z];

});
}
- (IBAction)initializePressed:(UIButton *)sender {
    initializeStarted = YES;
    //[self startDataCollectionWithInterval:1.0];
    [self formLeftMatrix];
    // Form The left matrix for least squares
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *docs_dir = [paths objectAtIndex:0];
    
    
    NSDateFormatter *outputFormatter = [[NSDateFormatter alloc] init];
    
    [outputFormatter setDateFormat:@"HH_mm_SS-MM_d"];
    NSDate *date = [[NSDate alloc] init];
    NSString *newDateString = [outputFormatter stringFromDate:date];
    NSLog(@"newDateString %@", newDateString);
    
    NSString *file = [NSString stringWithFormat:@"%@/%@%@",docs_dir,newDateString,@"q.txt"];
    const char *cDocumentPath = [file cStringUsingEncoding:NSUTF8StringEncoding];
    filePointGyro = fopen(cDocumentPath, "a");
    
    file = [NSString stringWithFormat:@"%@/%@%@",docs_dir,newDateString,@"accel.txt"];
    cDocumentPath = [file cStringUsingEncoding:NSUTF8StringEncoding];
    filePointAccel = fopen(cDocumentPath, "a");
    
    file = [NSString stringWithFormat:@"%@/%@%@",docs_dir,newDateString,@"mag.txt"];
    cDocumentPath = [file cStringUsingEncoding:NSUTF8StringEncoding];
    filePointMag = fopen(cDocumentPath, "a");
    
    file = [NSString stringWithFormat:@"%@/%@%@",docs_dir,newDateString,@"attitude.txt"];
    cDocumentPath = [file cStringUsingEncoding:NSUTF8StringEncoding];
    filePointAtt = fopen(cDocumentPath, "a");
}

// The left matrix in the least squares problem is formed here to send to filter
-(void)formLeftMatrix{
    // Vectors at MSP *source WolframAlpha
    magneticField = GLKVector3Make(-59, 24, 95);
    magneticField = GLKVector3Normalize(magneticField);
    
    gravity = GLKVector3Make(0.0, 0.0, -1.0);
    gravity = GLKVector3Normalize(gravity);
    
    magSkew = [self SkewMake2:magneticField];
    accelSkew = [self SkewMake2:gravity];
    NSLog(@"skew = [%f %f %f]",magSkew.m00,magSkew.m01,magSkew.m02);
    
    double H[] = {magSkew.m00, magSkew.m01, magSkew.m02, magSkew.m10, magSkew.m11, magSkew.m12, magSkew.m20, magSkew.m21, magSkew.m22, accelSkew.m00, accelSkew.m01, accelSkew.m02, accelSkew.m10, accelSkew.m11, accelSkew.m12, accelSkew.m20, accelSkew.m21, accelSkew.m22 };
    
    for (int i =0; i<18; i++) {
        NSLog(@"THE H = [%f, %f, %f]",H[i],H[i+1],H[i+2]);
        i = i+2;
    }
    int lda2 = 3; // number of columns
    
    int ldb2 = 3; // number of columns
    int ldc2 = 3;
    double C2[] = { 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00,
        0.00, 0.00, 0.00};
    
    // cblas_dgemm Compute C = A B the result will be stored in the dummy C
    
    
    // Note include constants as if already transposed
    cblas_dgemm (CblasRowMajor,
                 CblasTrans, CblasNoTrans, 3, 3, 6,
                 1.0, H, lda2, H, ldb2, 0.0, C2, ldc2);
    for (int i =0; i<9; i++) {
        NSLog(@"X = [%f, %f, %f]",C2[i],C2[i+1],C2[i+2]);
        i = i+2;
    }
    // invert C2
    int N = 3;
    int *IPIV[4];
    int LWORK = 9;
    double *WORK[9];
    int INFO;
    
    dgetrf_(&N,&N,C2,&N,IPIV,&INFO);
    dgetri_(&N,C2,&N,IPIV,WORK,&LWORK,&INFO);
    
    for (int i =0; i<9; i++) {
        NSLog(@"Inverse = [%f, %f, %f]",C2[i],C2[i+1],C2[i+2]);
        i = i+2;
    }
    double C4[9];
    for (int i =0; i<9; i++) {
        C4[i] = C2[i];
    }
    
    double C3[] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                    0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

    /* Compute C = A B */
    // Note ldb is rows of untranspose B

    ldb2 = 6;
    ldc2 = 6;
    cblas_dgemm (CblasRowMajor,
                 CblasNoTrans, CblasTrans, 3, 6, 3,
                 1.0, C4, lda2, H, 3, 0.0, C3, ldc2);
    
    for (int i =0; i<15; i++) {
        NSLog(@"H is = [%f, %f, %f, %f, %f, %f]",C3[i],C3[i+1],C3[i+2],C3[i+3],C3[i+4],C3[i+5]);
        i = i+5;
    }
    
    for (int i =0; i<18; i++) {
        Lm.LM[i] = C3[i];
        NSLog(@" print H = %f",C3[i]);
    }
    self.doneLabel.text = @"Finished";
    initialized = YES;

}

// idea for using measured gravity and mag as "0"
//  this function is unused
-(void)getMagAndGrav{
    
    gravity.x = (gravity.x + aX)/2;
    gravity.y = (gravity.y + aY)/2;
    gravity.z = (gravity.z + aZ)/2;
    
    magneticField.x = (magneticField.x + mX)/2;
    magneticField.y = (magneticField.z + mY)/2;
    magneticField.z = (magneticField.z + mZ)/2;
    aveCount++;
    
    if (aveCount > 10) {
        [self stopPressed:nil];
        [self formLeftMatrix];
    }
    
}

- (IBAction)deletePressed:(UIButton *)sender {
    
#define kDOCSFOLDER [NSHomeDirectory() stringByAppendingPathComponent:@"Documents"]
    
    NSFileManager *fileManager = [[NSFileManager alloc] init];
    NSDirectoryEnumerator *en = [fileManager enumeratorAtPath:kDOCSFOLDER];
    NSString *fileEn;
    while (fileEn = [en nextObject]) {
        NSLog(@"File To Delete : %@",fileEn);
        
        NSLog(@"yup");
        NSError *error;
        [fileManager removeItemAtPath:[kDOCSFOLDER stringByAppendingPathComponent:fileEn] error:&error];
        //NSLog([error description]);
        
    }
    
}

- (IBAction)stopPressed:(UIButton *)sender {
    
    [self.motionManager stopGyroUpdates];
    [self.motionManager stopAccelerometerUpdates];
    [self.motionManager stopDeviceMotionUpdates];
    fclose(filePointGyro);
    fclose(filePointAccel);
    fclose(filePointMag);
    fclose(filePointAtt);
    testRunning = NO;
    
}


-(RRVector6)RRVector6Make:(GLKVector3) v1:(GLKVector3) v2{
    
    RRVector6 v = {v1.x, v1.y, v1.z, v2.x, v2.y, v2.z};
    
    return v;
}

-(GLKMatrix3)SkewMake2:(GLKVector3) v1{
    
    GLKMatrix3 m = GLKMatrix3Make(0.0, 2*v1.z, 2*-1*v1.y, -1*2*v1.z, 0.0, 2*v1.x, 2*v1.y, -1*2*v1.x, 0.0);
    return m;
}
@end
