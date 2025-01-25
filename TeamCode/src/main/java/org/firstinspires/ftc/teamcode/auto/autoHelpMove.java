package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.pidLib;
import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class autoHelpMove {
    /* Variables */
    private final Robot robot;
    private final LinearOpMode linearOpMode;

    private boolean atTarget = false;

    private final int ticksInInch;

    private final pidLib drivePid = new pidLib(0.05, 0.001, 0.0005);
    private final pidLib turnPid = new pidLib(0.05, 0.001, 0.0005);

    /* Private Methods */

    private void moveToPos(double backLeftTarget, double backRightTarget, double frontLeftTarget, double frontRightTarget) {
        ((LeoOne)robot).moveWithEncoder(-backLeftTarget, -backRightTarget, -frontLeftTarget, -frontRightTarget);

        if (
            (Math.abs(backLeftTarget - ((LeoOne)robot).getBackLeftPos())) < 10 &&
            (Math.abs(backRightTarget - ((LeoOne)robot).getBackRightPos())) < 10 &&
            (Math.abs(frontLeftTarget - ((LeoOne)robot).getFrontLeftPos())) < 10 &&
            (Math.abs(frontRightTarget - ((LeoOne)robot).getFrontRightPos())) < 10
        ) {
            atTarget = true;
            robot.stopDrive();
        }
    }

    private void _turn(double deg, boolean clockwise) {
        double out = turnPid.getPid(deg, robot.getYaw());

        ((LeoOne)robot).move(
                (clockwise ? -1 : 1) * out,
                (clockwise ? 1 : -1) * out,
                (clockwise ? -1 : 1) * out,
                (clockwise ? 1 : -1) * out
        );

        if (Math.abs(deg - robot.getYaw()) < 2) {
            atTarget = true;
        }
    }

    private void _moveArmToPos(int pos) {
        robot.moveArmToPos(pos);
    }

    private void _extendToPos(int pos) {
        ((LeoOne)robot).extendToPos(pos);
    }

    /* Constructor */

    public autoHelpMove(LinearOpMode LOM, Robot theRobot) {
        linearOpMode = LOM;
        robot = theRobot;
        ticksInInch = robot.getTicksInInch();
    }

    /* Public Methods */

    public void driveInches(double in) {
        double pos = ticksInInch * in;

        while (linearOpMode.opModeIsActive() && !atTarget) {
            moveToPos(pos, pos, pos, pos);
        }

        // Flush bad data
        robot.stopDrive();
        atTarget = false;
        robot.resetDriveEncoders();
    }

    public void driveUntilDist(double in) {
        linearOpMode.telemetry.addData("Dist: ", ((LeoOne)robot).getDist());
        while (linearOpMode.opModeIsActive() && ((LeoOne)robot).getDist() > in) {
            linearOpMode.telemetry.addData("Dist: ", ((LeoOne)robot).getDist());
            robot.move(0, -1, 0);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.update();
        robot.stopDrive();

        // Flush bad data
        robot.resetDriveEncoders();
    }

    public void turn(double deg, boolean clockwise) {
        robot.resetYaw();

        while (linearOpMode.opModeIsActive() && !atTarget) {
            _turn(deg, clockwise);
        }

        // Flush bad data
        robot.stopDrive();
        atTarget = false;
        robot.resetDriveEncoders();
    }

    public void extendToPos(int pos) {
        while (Math.abs(pos - ((LeoOne)robot).getExtendPosition()) > 10) {
            _extendToPos(pos);
        }
    }

    public void moveArmToPos(int pos) {
        while (Math.abs(pos - robot.getArmPosition()) > 10) {
            _moveArmToPos(pos);
        }
    }

    public void killArmToPos(int pos) {
        while (Math.abs(pos - robot.getArmPosition()) > 10) {
            _moveArmToPos(pos);
        }
        robot.stopArm();
    }

    public void openClaw() {
        robot.moveClaw(0);
    }

    public void closeClaw() {
        robot.moveClaw(1);
    }

    /* MultiThread */
     public void moveUntilDistAndArm(double in, int pos) {
         while (
                 linearOpMode.opModeIsActive() &&
                 ((LeoOne)robot).getDist() > in &&
                 Math.abs(pos - robot.getArmPosition()) > 10
         ) {
             robot.move(drivePid.getPid(in, ((LeoOne)robot).getDist()), 0, 0);
             _moveArmToPos(pos);
         }

         // Flush bad data
         robot.stopDrive();
         robot.resetDriveEncoders();
     }

    public void turnAndArm(double deg, boolean clockwise, int pos) {
        robot.resetYaw();

        while (
                linearOpMode.opModeIsActive() &&
                !atTarget &&
                Math.abs(pos - robot.getArmPosition()) > 10
        ) {
            _turn(deg, clockwise);
            _moveArmToPos(pos);
        }

        // Flush bad data
        robot.stopDrive();
        atTarget = false;
        robot.resetDriveEncoders();
    }
}