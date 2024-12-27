package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class autoHelpMove {
    private final Robot robot;
    private final LinearOpMode linearOpMode;

    private boolean atTarget = false;

    private final int ticksInInch;

    private final ElapsedTime timer = new ElapsedTime();

    private void moveToPos(double backLeftTarget, double backRightTarget, double frontLeftTarget, double frontRightTarget) {
        ((LeoOne)robot).moveWithEncoder(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);

        if (
//            (Math.abs(backLeftTarget - ((LeoOne)robot).getBackLeftPos())) < 25 &&
//            (Math.abs(backRightTarget - ((LeoOne)robot).getBackRightPos())) < 25 &&
//            (Math.abs(frontLeftTarget - ((LeoOne)robot).getFrontLeftPos())) < 25 &&
            (Math.abs(frontRightTarget - ((LeoOne)robot).getFrontRightPos())) < 25
        ) {
            atTarget = true;
            robot.move(0, 0, 0);
        }
    }

    public autoHelpMove(LinearOpMode LOM, Robot theRobot) {
        linearOpMode = LOM;
        robot = theRobot;
        ticksInInch = robot.getTicksInInch();
    }

    public void driveInches(double in) {
        double pos = ticksInInch * in;

        while (linearOpMode.opModeIsActive() && !atTarget) {
            moveToPos(pos, pos, pos, pos);
        }

        // Flush bad data
        atTarget = false;
        robot.resetDriveEncoders();
    }

    public void turn(double deg, boolean clockwise) {
        /*
        Robot is 18 inches, or 9 inches in radius.

        2πr = 2π9 = 360 = π/20 inches needed per degree
        */
        robot.resetYaw();

        double error;
        double integralSum = 0;
        double derivative;
        double out;
        double lastError = 0;

        double p = 0.05;
        double i = 0.001;
        double d = 0.0005;

        while (linearOpMode.opModeIsActive() && !atTarget) {
            error = deg - robot.getYaw();
            integralSum = integralSum + (error * timer.seconds());
            derivative = (error - lastError) / timer.seconds();

            out = (p * error) + (i * integralSum) + (d * derivative);

            ((LeoOne)robot).move(
                (clockwise ? -1 : 1) * out,
                (clockwise ? 1 : -1) * out,
                (clockwise ? -1 : 1) * out,
                (clockwise ? 1 : -1) * out
            );

            lastError = error;

            timer.reset();

            if (Math.abs(deg - robot.getYaw()) < 2) {
                atTarget = true;
            }
        }
        robot.move(0, 0, 0);

        // Flush bad data
        atTarget = false;
        robot.resetDriveEncoders();
    }
}
