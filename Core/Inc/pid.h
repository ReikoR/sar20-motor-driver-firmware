#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct PI_Control {
	float pGain;
	float iGain;
	float dGain;
	float maxI;
	float i;
	float period;
	float prevError;
} PI_Control;

float PI_Control_update(PI_Control* control, float ref, float feedback) {
	float error = ref - feedback;

	control->i += control->iGain * error;

	if (control->i > control->maxI) {
		control->i = control->maxI;
	} else if (control->i < -control->maxI) {
		control->i = -control->maxI;
	}

	float output = control->pGain * error + control->i + control->dGain * (error - control->prevError);

	control->prevError = error;

	return output;
}

#endif /* INC_PID_H_ */
