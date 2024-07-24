class PID {
private:
	float Kp, Ki, Kd, I, dE;
public:
	void SetCoefficients(float Kp, float Ki, float Kd);
	void SetCoefficients(float *Kp, float *Ki, float *Kd);
	float Calculate(float setpoint, float feedback);
};
