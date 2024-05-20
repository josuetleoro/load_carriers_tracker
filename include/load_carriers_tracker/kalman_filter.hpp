#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

namespace Filter
{

  class KalmanFilter
  {
  private:
    double k_; // Kalman gain
    double p_; // Estimation variance
    double q_; // Process noise variance
    double r_; // Sensor noise
    double x_; // Estimation

  public:
    KalmanFilter()
    {
      this->p_ = 0.0;
      this->q_ = 0.0;
      this->r_ = 1.0;
      this->x_ = 0.0;
    }
    KalmanFilter(double p, double q, double r, double x0 = 0.0)
    {
      p_ = p;
      q_ = q;
      r_ = r;
      x_ = x0;
    }
    void reset(double x, double p)
    {
      x_ = x;
      p_ = p;
    }
    void update(double meas)
    {
      p_ = p_ + q_;
      k_ = p_ / (p_ + r_);
      x_ = x_ + k_ * (meas - x_);
      p_ = (1 - k_) * p_;
    }
    double getEstimation() { return x_; }
    double getVariance() { return p_; }
  };

} // namespace Filter

#endif