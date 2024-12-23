package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class autoHelpDrive {
    private final Robot robot;
    private final LinearOpMode linearOpMode;

    private boolean atTarget = false;

    private final int ticksInInch;

    private void moveToPos(double backLeftTarget, double backRightTarget, double frontLeftTarget, double frontRightTarget) {
        ((LeoOne)robot).moveWithEncoder(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);

        if (
            (Math.abs(backLeftTarget - ((LeoOne)robot).getBackLeftPos())) < 25 &&
            (Math.abs(backRightTarget - ((LeoOne)robot).getBackRightPos())) < 25 &&
            (Math.abs(frontLeftTarget - ((LeoOne)robot).getFrontLeftPos())) < 25 &&
            (Math.abs(frontRightTarget - ((LeoOne)robot).getFrontRightPos())) < 25
        ) {
            atTarget = true;
            robot.move(0, 0, 0);
        }
    }

    public autoHelpDrive(LinearOpMode LOM, Robot theRobot) {
        linearOpMode = LOM;
        robot = theRobot;
        ticksInInch = ((LeoOne) robot).getTicksInInch();
    }

    public void driveInches(double in) {
        double pos = ticksInInch * in;

        while (linearOpMode.opModeIsActive() && !atTarget) {
            moveToPos(pos, pos, pos, pos);
        }

        // Flush bad data
        atTarget = false;
        ((LeoOne)robot).resetDriveEncoders();
    }

    public void turn(double deg, boolean clockwise) {
        /*
        Robot is 18 inches, or 9 inches in radius.

        2πr = 2π9 = 360 = π/20 inches needed per degree
        */
        double pos = ticksInInch * deg * (Math.PI / 20);
        double backLeftTarget = (clockwise ? -1 : 1) * pos;
        double backRightTarget = (clockwise ? 1 : -1) * pos;
        double frontLeftTarget = (clockwise ? -1 : 1) * pos;
        double frontRightTarget = (clockwise ? 1 : -1) * pos;


        while (linearOpMode.opModeIsActive() && !atTarget) {
            moveToPos(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
        }

        // Flush bad data
        atTarget = false;
        ((LeoOne)robot).resetDriveEncoders();
    }
}
