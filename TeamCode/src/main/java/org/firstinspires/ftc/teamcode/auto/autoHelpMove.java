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


    private double driveIntegralSum = 0;
    private double driveLastError = 0;

    private void moveToPos(double backLeftTarget, double backRightTarget, double frontLeftTarget, double frontRightTarget) {
        ((LeoOne)robot).moveWithEncoder(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);

        if (
            (Math.abs(backLeftTarget - ((LeoOne)robot).getBackLeftPos())) < 10 &&
            (Math.abs(backRightTarget - ((LeoOne)robot).getBackRightPos())) < 10 &&
            (Math.abs(frontLeftTarget - ((LeoOne)robot).getFrontLeftPos())) < 10 &&
            (Math.abs(frontRightTarget - ((LeoOne)robot).getFrontRightPos())) < 10
        ) {
            atTarget = true;
            robot.move(0, 0, 0);
        }
    }

    private void _turn(double deg, boolean clockwise) {
        double error;
        double derivative;
        double out;

        double p = 0.05;
        double i = 0.001;
        double d = 0.0005;

        error = deg - robot.getYaw();
        driveIntegralSum = driveIntegralSum  + (error * timer.seconds());
        derivative = (error - driveLastError) / timer.seconds();

        out = (p * error) + (i * driveIntegralSum) + (d * derivative);

        ((LeoOne)robot).move(
                (clockwise ? -1 : 1) * out,
                (clockwise ? 1 : -1) * out,
                (clockwise ? -1 : 1) * out,
                (clockwise ? 1 : -1) * out
        );

        driveLastError = error;

        timer.reset();

        if (Math.abs(deg - robot.getYaw()) < 2) {
            atTarget = true;
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
        robot.resetYaw();

        while (linearOpMode.opModeIsActive() && !atTarget) {
            _turn(deg, clockwise);
        }
        robot.move(0, 0, 0);

        // Flush bad data
        atTarget = false;
        robot.resetDriveEncoders();
    }
}
