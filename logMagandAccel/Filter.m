//
//  Filter.m
//  updateTest
//
//  Created by Robert Ritchie on 12/8/12.
//  Copyright (c) 2012 Robert Ritchie. All rights reserved.
//

#import "Filter.h"

@implementation Filter

-(id)initWithVectors:(GLKVector3)magVec :(GLKVector3)accelVec andLeftMatrix:(RRLeftMatrix)LM delegate:(id<FilterDelegate>)theDelegate{
    
    if (self = [super init]) {
        magHatb = magVec;
        accelHatb = accelVec;
        self.delegate = theDelegate;
        magneticField = GLKVector3Make(40, 166, -224 );
        magneticField = GLKVector3Normalize(magneticField);
        
        gravity = GLKVector3Make(-.06, -.04, -1);
        gravity = GLKVector3Normalize(gravity);
        Lm = LM;
    }
    return self;
    
}

#pragma mark - Main operation

-(void)main{
    @autoreleasepool {
        NSLog(@"in filter %f",magHatb.x);
        
        // 1) Initialize the attitude quaternion as
        qHat = GLKQuaternionMake(0.0, 0.0, 0.0, 1.0); // Note: scaler last
        // and the attitude error quaternion as
        qe = GLKQuaternionMake(0.0, 0.0, 0.0, 1.0);
        for (int i =0; i<9; i++) {
            NSLog(@"XinFilter = [%f, %f, %f]",Lm.LM[i],Lm.LM[i+1],Lm.LM[i+2]);
            i = i+2;
        }
                
        int i = 0;
        // Use while loop of 30 steps so calculation time is consistant
        
        while (i<30) {
            // 2) Use q to map the body measurement of ~u and ~accel to the navigation frame. That is, compute
            NSLog(@"magHat = [%f %f %f]",magHatb.x,magHatb.y,magHatb.z);
            NSLog(@"qhat = [%f %f %f %f]",qHat.x,qHat.y,qHat.z,qHat.w);
            magHatNq = GLKQuaternionMultiply(qHat, GLKQuaternionMultiply(GLKQuaternionMake(magHatb.x, magHatb.y, magHatb.z, 0.0), GLKQuaternionInvert(qHat)));
            NSLog(@"magHatNq = [%f %f %f %f]",magHatNq.x,magHatNq.y,magHatNq.z,magHatNq.w);
            magHatN = GLKVector3Make(magHatNq.x, magHatNq.y, magHatNq.z);
            
            // and likewise for accelHatn
            
            accelHatNq = GLKQuaternionMultiply(qHat, GLKQuaternionMultiply(GLKQuaternionMake(accelHatb.x, accelHatb.y, accelHatb.z, 0.0), GLKQuaternionInvert(qHat)));
            accelHatN = GLKVector3Make(accelHatNq.x, accelHatNq.y, accelHatNq.z);
            
            // 3) Formulate the errors
            
            dmagHat = GLKVector3Subtract(magneticField, magHatN);
            daccelHat = GLKVector3Subtract(gravity, accelHatN);
            
            hatVec = [self RRHatVecMake:dmagHat :daccelHat];
            
            // 4) H is a constant computed during initialization
            
            // 5) Estimate the quaternion error
            
            double C2[] = { 0.00, 0.00, 0.00};
            /* Compute C = A B */
            // Note include constants as if already transposed
            cblas_dgemv (CblasRowMajor,CblasNoTrans, 3, 6, 1.0, Lm.LM, 6, hatVec.v, 1, 0.0, C2, 1);
            NSLog(@"Vector is = [%f, %f, %f].", C2[0], C2[1], C2[2]);
            
            qe = GLKQuaternionMake(C2[0], C2[1], C2[2], 1.0);
            
            // 6) Update the quaternion estimate
            
            qHat = GLKQuaternionMultiply(qHat, qe);
            
            // 7) Normalize the updated quaternion estimate
            
            qHat = GLKQuaternionNormalize(qHat);
            
            // 8) Repeat
            
            i++;
        }
        
        NSLog(@"qHat = [%f %f %f]",qHat.x,qHat.y,qHat.z);
        
        GLKVector3 vec = [self quatToEuler:qHat];
        
        NSLog(@"qHat = [%f %f %f]",vec.x,vec.y,vec.z);
        
        [self.delegate filterDidFinish:vec :vec];
    }
}

-(RRVector6)RRVector6Make:(GLKVector3) v1:(GLKVector3) v2{
    
    RRVector6 v = {v1.x, v1.y, v1.z, v2.x, v2.y, v2.z};
    
    return v;
}

-(RRHatVec)RRHatVecMake:(GLKVector3) v1:(GLKVector3) v2{
    
    RRHatVec v;
    v.v[0] = v1.x;
    v.v[1] = v1.y;
    v.v[2] = v1.z;
    v.v[3] = v2.x;
    v.v[4] = v2.y;
    v.v[5] = v2.z;
    
    return v;
}

-(GLKMatrix3)SkewMake2:(GLKVector3) v1{
    
    GLKMatrix3 m = GLKMatrix3Make(0.0, 2*v1.z, 2*-1*v1.y, -1*2*v1.z, 0.0, 2*v1.x, 2*v1.y, -1*2*v1.x, 0.0);
    return m;
}

-(GLKVector3)quatToEuler:(GLKQuaternion)q{
    
    GLKVector3 v;
    v.x = atan2(2*(q.w*q.x+q.y*q.z), 1-2*(pow(q.x,2)+pow(q.y, 2)));
    v.y = asin(2*(q.w*q.y - q.z*q.x));
    v.z = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(pow(q.y, 2)+pow(q.z, 2)));
    
    return v;
}


-(void)invertTest{
    
    // colA   = number of columns of A.
    // colB= number of columns of B, usually 1.
    // rowA = number of rows of A.
    // IPIV= pivot indices.
    // rowB = number of rows of B.
    
    int colA = 3, rowA = 3, rowB = 3, colB = 6;
    int IPIV[colA], INFO;
    
    double A[9] = {7.9050, -0.3443, 0.2130, -0.3443, 6.6042, 1.7156, 0.2130, 1.7157,
        1.4908};
 
    double B[18] = {0, -1.5906, -1.1787, 1.5906, 0, 0.2840, 1.1787, -0.2840, 0, 0, -1.9948, 0.0798, 1.9948, 0,  -0.1197, -0.0798, 0.1197, 0};


    NSLog(@"B = [%f, %f, %f].", B[0], B[1], B[2]);
    
    dgesv_(&colA, &colB, A, &rowA, IPIV, B, &rowB, &INFO);
    
    NSLog(@"X = [%f, %f, %f].", B[0], B[1], B[2]);
}




@end
