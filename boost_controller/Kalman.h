#ifndef KALMAN_H
#define KALMAN_H


class Kalman
{
  private:
    float uHat = 0; // initial estimated state
    float H = 1; // measurement map scalar
    float Q = 10; // initial estimated covariance
    float P = 0; // initial error covariance (must be 0)
    float K = 0; // initial kalman gain

  public:
    float R = 0;
    void init(float R);

    float filter(float U);
};

#endif