#include "Kalman.h"


float Kalman_getAngle_roll(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += (Q_bias) * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
     S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
     // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
     y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P00_temp = P[0][0];
    P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};
//
float Kalman_getAngle_pitch(float newAnglez, float newRatez, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    ratez = newRatez - biasz;
    anglez += dt * ratez;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    Pz[0][0] += dt * (dt*Pz[1][1] - Pz[0][1] - Pz[1][0] + Q_anglez);
    Pz[0][1] -= dt * Pz[1][1];
    Pz[1][0] -= dt * Pz[1][1];
    Pz[1][1] += (Q_biasz ) * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
     Sz = Pz[0][0] + R_measurez; // Estimate error
    /* Step 5 */
     // Kalman gain - This is a 2x1 vector
    Kz[0] = Pz[0][0] / Sz;
    Kz[1] = Pz[1][0] / Sz;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
     yz = newAnglez - anglez; // Angle difference
    /* Step 6 */
    anglez += Kz[0] * yz;
    biasz += Kz[1] * yz;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P00_tempz = Pz[0][0];
    P01_tempz = Pz[0][1];

    Pz[0][0] -= Kz[0] * P00_tempz;
    Pz[0][1] -= Kz[0] * P01_tempz;
    Pz[1][0] -= Kz[1] * P00_tempz;
    Pz[1][1] -= Kz[1] * P01_tempz;

    return anglez;
};
