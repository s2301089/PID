#include "PID.hpp"

namespace snct{
	PID::PID(void){
		_period = 0.001f;
		_limit = 10000.0f;
		_limit_i = 3000.0f;
		_tolerance = 0.0f;
		_target = 0.0f;
		_kp = 0.5f;
		_kd = 0.0f;
		_ki = 0.0f;

		_delta_p = 0.0f;
		_delta_d = 0.0f;
		_delta_i = 0.0f;
		_pout = 0.0f;
		_dout = 0.0f;
		_iout = 0.0f;
		_output = 0.0f;
		_err = 0.0f;
		_last_err = 0.0f;
		_last_err2 = 0.0f;
	}

	PID::PID(float period, float limit, float limit_i, float tolerance, float target, float kp, float kd, float ki){
		_period = period;
		_limit = limit;
		_limit_i = limit_i;
		_tolerance = tolerance;
		_target = target;
		_kp = kp;
		_kd = kd;
		_ki = ki;

		_delta_p = 0.0f;
		_delta_d = 0.0f;
		_delta_i = 0.0f;
		_pout = 0.0f;
		_dout = 0.0f;
		_iout = 0.0f;
		_output = 0.0f;
		_err = 0.0f;
		_last_err = 0.0f;
		_last_err2 = 0.0f;
	}

	float PID::calc_velocity(const float feedback){
		_err = _target - feedback;
		if(-_tolerance < _err && _err < _tolerance){
			_err = 0.0f;
		}

		_delta_p = _kp * (_err - _last_err);
		_delta_d = _kd * (_err - 2.0f * _last_err + _last_err2) / _period;
		_delta_i = _ki * _err * _period;

		_output += _delta_p + _delta_d + _delta_i;

		if(_output > _limit){
			_output = _limit;
		}else if(_output < -_limit){
			_output = -_limit;
		}

		_last_err2 = _last_err;
		_last_err = _err;

		return _output;
	}

	float PID::calc_location(const float feedback){
		_err = _target - feedback;
		if(-_tolerance < _err && _err < _tolerance){
			_err = 0.0f;
		}

		_pout = _kp * _err;
		_dout = _kd * (_err - _last_err) / _period;
		_iout += _ki * _err * _period;

		if(_iout > _limit_i){
			_iout = _limit_i;
		}else if(_iout < -_limit_i){
			_iout = -_limit_i;
		}

		_output = _pout + _dout + _iout;

		if(_output > _limit){
			_output = _limit;
		}else if(_output < -_limit){
			_output = -_limit;
		}

		_last_err = _err;

		return _output;
	}
}