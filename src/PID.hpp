#ifndef _INCLUDE_PID_H_
#define _INCLUDE_PID_H_

namespace snct{
	class PID{
		public:
			PID(void);
			PID(float period, float limit, float limit_i, float tolerance, float target, float kp, float kd, float ki);
			~PID(void);

			float calc_velocity(const float feedback);
			float calc_location(const float feedback);

			float getPeriod(void) const{
				return _period;
			}

			float getLimit(void) const{
				return _limit;
			}

			float getLimitI(void) const{
				return _limit_i; 
			}

			float getTolerance(void) const{
				return _tolerance; 
			}

			float getTarget(void) const{
				return _target; 
			}

			float getDeltaP(void) const{
				return _delta_p; 
			}

			float getDeltaD(void) const{
				return _delta_d; 
			}

			float getDeltaI(void) const{
				return _delta_i; 
			}

			float getPout(void) const{
				return _pout; 
			}

			float getDout(void) const{
				return _dout; 
			}

			float getIout(void) const{
				return _iout; 
			}

			float getOutput(void) const{
				return _output; 
			}

			float getErr(void) const{
				return _err; 
			}

			float getLastErr(void) const{
				return _last_err; 
			}

			float getLastErr2(void) const{
				return _last_err2; 
			}

			float getKp(void) const{
				return _kp; 
			}

			float getKd(void) const{
				return _kd; 
			}

			float getKi(void) const{
				return _ki; 
			}

			void setPeriod(const float period){
				_period = period;
			}

			void setLimit(const float limit){
				_limit = limit;
			}

			void setLimitI(const float limit_i){
				_limit_i = limit_i;
			}

			void setTolerance(const float tolerance){
				_tolerance = tolerance;
			}

			void setTarget(const float target){
				_target = target;
			}

			void setKp(const float kp){
				_kp = kp;
			}

			void setKd(const float kd){
				_kd = kd;
			}

			void setKi(const float ki){
				_ki = ki;
			}

		private:
			float
				_period,
				_limit,
				_limit_i,
				_tolerance,
				_target,
				_delta_p,
				_delta_d,
				_delta_i,
				_pout,
				_dout,
				_iout,
				_output,
				_err,
				_last_err,
				_last_err2,
				_kp,
				_kd,
				_ki
			;
	}
}

#endif // _INCLUDE_PID_H_