package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.util.ElapsedTime;

public class pidLib {
    double error;
    double integralSum;
    double derivative;
    double lastError;
    double out;

    double p;
    double i;
    double d;


    ElapsedTime timer = new ElapsedTime();

    /* Constructors */
    public pidLib() {
        p = 0;
        i = 0;
        d = 0;
    }
    public pidLib(double kP) {
        p = kP;
        i = 0;
        d = 0;
    }
    public pidLib(double kP, double kI) {
        p = kP;
        i = kI;
        d = 0;
    }
    public pidLib(double kP, double kI, double kD) {
        p = kP;
        i = kI;
        d = kD;
    }



    public void updatePid(double kP, double kI, double kD) {
        p = kP;
        i = kI;
        d = kD;
    }



    public double getPid(double target, double real) {
        error = target - real;
        integralSum = integralSum + (error * timer.seconds());
        derivative = (error - lastError) / timer.seconds();

        out = (p * error) + (i * integralSum) + (d * derivative);

        lastError = error;

        timer.reset();

        return out;
    }
}
