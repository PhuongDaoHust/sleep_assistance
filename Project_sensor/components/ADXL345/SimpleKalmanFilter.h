#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;

  void SimpleKalman_Init(float mea_e, float est_e, float q);
  float updateEstimate(float mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();




#endif
