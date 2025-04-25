
// PID controller code derived from the following video:
// https://www.youtube.com/watch?v=jY6bBcMtseY
float ts = 0.01;

// calculate desired rates for roll, pitch, and yaw
float desiredRate(float inputValue){
    float output = 0.15 * (inputValue-1500);
    return output;
}

float inputThrottle(float inputValue){
    float output = inputValue;
    return output;
}

// calculate error values for roll, pitch, and yaw
float errorValue(float desiredRate, float rate) {
    float output = desiredRate - rate;
    return output;
}

float inputRoll(float p, float i, float d, float currError, float &prevError, float &prevI){
    float output = p * currError + prevI + i * (currError + prevError) * ts / 2 + d * (currError - prevError) / ts; 
    prevError = currError;
    prevI = i;
    return output;
}

int motor1(float throttle, float roll, float pitch, float yaw){
    int output = throttle - pitch - roll - yaw;
    return output;
}