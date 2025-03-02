package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    // PID constants
    private double Kp, Ki, Kd;

    // Error terms
    private double previousError;
    private double integralSum;
    private double derivative;

    ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        // Initialize previous error and integral to zero
        this.previousError = 0;
        this.integralSum = 0;
        this.derivative = 0;
        this.timer.reset();
    }


    public double update(double target, double state) {
        double error = target - state;
        double derivative = (error - previousError) / timer.seconds();
        previousError = error;
        integralSum = integralSum + (error * timer.seconds());
        this.derivative = derivative;
        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    public double getDerivative() {
        return derivative;
    }
}