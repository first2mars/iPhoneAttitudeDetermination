//
//  ViewController.h
//  logMagandAccel
//
//  Created by Robert Ritchie on 11/24/12.
//  Copyright (c) 2012 Robert Ritchie. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <CoreMotion/CoreMotion.h>
#import <GLKit/GLKit.h>
#import "Filter.h"


@interface ViewController : UIViewController<FilterDelegate>{
    
    FILE *filePointGyro;
    FILE *filePointAccel;
    FILE *filePointMag;
    FILE *filePointAtt;
    BOOL testRunning;
    BOOL initialized;
    BOOL initializeStarted;
    int aveCount;
    
    CMMotionManager *motionManager;
    
    double aX,aY,aZ,aT,mX,mY,mZ,mT;
    GLKVector3 combo;
    GLKMatrix3 magSkew,accelSkew;
    GLKVector3 gravity;
    GLKVector3 magneticField;
    
    RRLeftMatrix Lm;
    

}

@property (weak, nonatomic) IBOutlet UILabel *xLabel;
@property (weak, nonatomic) IBOutlet UILabel *yLabel;
@property (weak, nonatomic) IBOutlet UILabel *zLabel;
@property (weak, nonatomic) IBOutlet UILabel *doneLabel;

- (IBAction)startPressed:(UIButton *)sender;
- (IBAction)deletePressed:(UIButton *)sender;
- (IBAction)stopPressed:(UIButton *)sender;


@end
