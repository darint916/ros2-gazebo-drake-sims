#include "robot_controller/pid_controller.hpp"
PIDController::PIDController() : _kp(0), _ki(0), _kd(0), _max_output(0), _max_integral(0), _last_error(0), _total_error(0)
{}

PIDController::PIDController(double kp, double ki, double kd, double max_output, double max_integral)
    : _kp(kp), _ki(ki), _kd(kd), _max_output(max_output), _max_integral(max_integral), _last_error(0), _total_error(0)
{}

PIDController::~PIDController()
{}

double PIDController::calculate(double error, double dt)
{
    if (dt == 0) return _kp * _last_error;
    double derivative = (error - _last_error) / dt;
    _last_error = error;
    _total_error += error * dt;

    if(_total_error > _max_integral) _total_error = _max_integral;
    else if(_total_error < -_max_integral) _total_error = -_max_integral;

    double output = _kp * error + _ki * _total_error + _kd * derivative;

    if(output > _max_output) output = _max_output;
    else if(output < -_max_output) output = -_max_output;

    return output;
}

void PIDController::set_gains(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}