// PID controller

#include <limits>

class PID {
public:
    PID(
        float kp = 1.f,
        float ki = 0.f,
        float kd = 0.f,
        float outputMin = std::numeric_limits<float>::lowest(),
        float outputMax = std::numeric_limits<float>::max())
        : kp(kp), ki(ki), kd(kd),
          outputMin(outputMin),
          outputMax(outputMax)
    {}

    float inline pid(float error, float dt)
    {
        if (dt == 0)
        {
            std::cerr << "WARNING: dt = 0 in PID controller" << std::endl;
            return 0;
        }
        integral += (error * dt);
        float derivative = (error - error_prior) / dt;
        error_prior = error;
        float output = kp * error + ki * integral + kd * derivative;

        if (output > outputMax)
        {
            integral -= output - outputMax;
            return outputMax;
        }
        else if (output < outputMin)
        {
            integral += outputMin - output;
            return outputMin;
        }
        else
        {
            return output;
        }
    }

    void inline reset()
    {
        integral = 0.f;
        error_prior = 0.f;
    }

    float kp = 1;
	float ki = 0;
	float kd = 0;
    float outputMin = std::numeric_limits<float>::lowest();
	float outputMax = std::numeric_limits<float>::max();

protected:
	float integral = 0.f;
	float error_prior = 0.f;
};
