//
//  Filter.h
//  updateTest
//
//  Created by Robert Ritchie on 12/8/12.
//  Copyright (c) 2012 Robert Ritchie. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <Accelerate/Accelerate.h>
#import <GLKit/GLKit.h>

union _RRVector6
{
    struct { float x, y, z, x1, y2, y3; };
    struct { float r, g, b, a, c, d; };
    struct { float s, t, p, q, h, l; };
    float v[6];
};
typedef union _RRVector6 RRVector6;

struct _RRMatrix6 {
    double m00, m01, m02, m03, m04, m05, m06;
    double m10, m11, m12, m13, m14, m15, m16;
    double m20, m21, m22, m23, m24, m25, m26;
    double m30, m31, m32, m33, m34, m35, m36;
    double m40, m41, m42, m43, m44, m45, m46;
    double m50, m51, m52, m53, m54, m55, m56;
};
typedef struct _RRMatrix6 RRMatrix6;

struct _RRLeftMatrix {
    double LM[18];
};
typedef struct _RRLeftMatrix RRLeftMatrix;

struct _RRHatVec {
    double v[6];
};
typedef struct _RRHatVec RRHatVec;


@protocol FilterDelegate <NSObject>

-(void)filterDidFinish:(GLKVector3)magVec :(GLKVector3)accelVec;

@end

@interface Filter : NSOperation{
    
    GLKVector3 gravity;
    GLKVector3 magneticField;
    
    GLKQuaternion qHat;
    GLKQuaternion qe;
    
    GLKQuaternion magHatNq, accelHatNq;
    GLKVector3 magHatN, accelHatN;
    
    GLKVector3 magHatb,accelHatb;
    GLKVector3 dmagHat, daccelHat;
    RRHatVec hatVec;
    
    RRLeftMatrix Lm;
    
}

@property (nonatomic, weak) id <FilterDelegate> delegate;

-(id)initWithVectors:(GLKVector3)magVec :(GLKVector3)accelVec andLeftMatrix:(RRLeftMatrix)LM delegate:(id<FilterDelegate>)theDelegate;

@end
