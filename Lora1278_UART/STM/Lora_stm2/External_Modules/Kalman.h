/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
#include "stm32f4xx.h"
#ifndef _Kalman_h_
#define _Kalman_h_


extern double a,b,c;

    /* Kalman filter variables */
    float Q_angle= 0.008f; // Process noise variance for the accelerometer
    float Q_bias= 0.003f; // Process noise variance for the gyro bias
    float R_measure= 0.05f;// Measurement noise variance - this is actually the variance of the measurement noise

    float angle = 0.0f;; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias =0.0f;; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
		float S;
		float K[2];
		float y;
		 float P00_temp;
		  float P01_temp;
  
    /* Kalman filter variables */
    float Q_anglez= 0.008f;; // Process noise variance for the accelerometer
    float Q_biasz= 0.003f; // Process noise variance for the gyro bias
    float R_measurez= 0.05f; // Measurement noise variance - this is actually the variance of the measurement noise

    float anglez = 0.0f; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float biasz =0.0f;// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float ratez; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float Pz[2][2]; // Error covariance matrix - This is a 2x2 matrix
		float Sz;
		float Kz[2];
		float yz;
		 float P00_tempz;
		  float P01_tempz;

#endif
