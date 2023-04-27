#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PIDController {
 public:
  PIDController();
  PIDController(double kp, double ki, double kd, double max_output, double max_integral);
  ~PIDController();

  double calculate(double error);
  void set_gains(double kp, double ki, double kd);

 private:
  double _kp;
  double _ki;
  double _kd;
  double _max_output;
  double _max_integral;
  double _last_error;
  double _total_error;
};

#endif  // __PID_CONTROLLER_H__
