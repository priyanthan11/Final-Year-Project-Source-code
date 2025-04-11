#pragma once

#include "CoreMinimal.h"
#include <cfloat>


class PIDController {
private:
    float Kp, Ki, Kd;
    float Integral;
    float PreviousError;
    float OutputMin; // Minimum output value (clamping)
    float OutputMax; // Maximum output value (clamping)
    float IntegralMin; // Minimum value for the integral term
    float IntegralMax; // Maximum value for the integral term

public:
    PIDController(float kp, float ki, float kd, float outputMin = -FLT_MAX, float outputMax = FLT_MAX)
        : Kp(kp), Ki(ki), Kd(kd),
        Integral(0), PreviousError(0),
        OutputMin(outputMin), OutputMax(outputMax),
        IntegralMin(-FLT_MAX), IntegralMax(FLT_MAX) {}

    float Compute(float Target, float Current, float DeltaTime) {
        if (DeltaTime <= 0.0f) return 0.0f; // Guard against division by zero

        float Error = Target - Current;

        // Compute integral with clamping to prevent windup
        Integral += Error * DeltaTime;
        Integral = FMath::Clamp(Integral, IntegralMin, IntegralMax);

        // Compute derivative
        float Derivative = (Error - PreviousError) / DeltaTime;

        // Compute output
        float Output = Kp * Error + Ki * Integral + Kd * Derivative;

        // Clamp output to the specified range
        Output = FMath::Clamp(Output, OutputMin, OutputMax);

        // Update previous error for the next iteration
        PreviousError = Error;

        return Output;
    }

    void Reset(float InitialError = 0.0f) {
        Integral = 0;
        PreviousError = InitialError;
    }

    void SetOutputLimits(float min, float max) {
        OutputMin = min;
        OutputMax = max;
    }

    void SetIntegralLimits(float min, float max) {
        IntegralMin = min;
        IntegralMax = max;
    }
};






//class PIDController {
//private:
//    float Kp, Ki, Kd;
//    float Integral;
//    float PreviousError;
//
//public:
//    PIDController(float kp, float ki, float kd)
//        : Kp(kp), Ki(ki), Kd(kd), Integral(0), PreviousError(0) {}
//
//    float Compute(float Target, float Current, float DeltaTime) {
//        float Error = Target - Current;
//        Integral += Error * DeltaTime;
//        float Derivative = (Error - PreviousError) / DeltaTime;
//        PreviousError = Error;
//
//
//        return Kp * Error + Ki * Integral + Kd * Derivative;
//    }
//
//    void Reset() {
//        Integral = 0;
//        PreviousError = 0;
//    }
//};