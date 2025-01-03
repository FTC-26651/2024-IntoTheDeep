package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.extensions.LionsDcMotorEx;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LeoTwo extends Robot {
    double left;
    double right;
    double max;

    double driveSpeed = 1;

    double targetPosition;

    double lastPos;
    boolean isArmZeroing = false;

    private final ElapsedTime armTime = new ElapsedTime();
    private final ElapsedTime extendTime = new ElapsedTime();

    DcMotorEx armMotor          = null;
    LionsDcMotorEx armMotorEx   = null;
    DcMotor   extensionMotor    = null;
    DcMotor   leftDrive         = null;
    DcMotor   rightDrive        = null;
    CRServo   wrist             = null;
    Servo     claw              = null;

    IMU imu;

    YawPitchRollAngles orientation;

    public LeoTwo(HardwareMap hm, Telemetry tm) {
        super(hm, tm);
    }
    public void initRobot() {
        armMotor = this.hardwareMap.get(DcMotorEx.class, "left_arm");
        armMotorEx = new LionsDcMotorEx(armMotor);
        extensionMotor = this.hardwareMap.get(DcMotor.class, "extender");
        leftDrive = this.hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive = this.hardwareMap.get(DcMotor.class, "right_front_drive");
        wrist = this.hardwareMap.get(CRServo.class, "wrist");
        claw = this.hardwareMap.get(Servo.class, "claw");

        imu = this.hardwareMap.get(IMU.class, "imu");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorEx.setCurrentAlert(9.2, CurrentUnit.AMPS);

        armMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist.setDirection(DcMotor.Direction.REVERSE);

        armMotorEx.setPid(0.0065, 0.001, 0.00015);

        claw.setPosition(0);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        orientation = imu.getRobotYawPitchRollAngles();
    }

    public void move(double x_axis, double y_axis, double tilt) {
        left  = y_axis + tilt;
        right = y_axis - tilt;

        /* Normalize the values so neither exceed +/- 1.0 */
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        leftDrive.setPower(left * driveSpeed);
        rightDrive.setPower(right * driveSpeed);
    }

    public void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public int getTicksInInch() {
        return 58;
    }

    public void resetDriveEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void increaseMoveSpeed() {
        if (driveSpeed < 0.91) {
            driveSpeed = driveSpeed + 0.1;
        }
    }
    public void decreaseMoveSpeed() {
        if (driveSpeed > 0.11) {
            driveSpeed = driveSpeed - 0.1;
        }
    }

    public void extendArm(double inOrOut) {
        if (!isArmZeroing && armMotorEx.getCurrentPosition() < 3300 && armMotorEx.getCurrentPosition() > 2400) {
            extensionMotor.setPower(inOrOut);
            extendTime.reset();
        } else if (extendTime.seconds() < 1.5) {
            // Retract the arm to stay legal
            extensionMotor.setPower(-1);
        } else {
            extensionMotor.setPower(0);
        }
    }

    public void moveArm(double direction) {
        if(!isArmZeroing) {
            if (armMotorEx.isOverCurrent()){
                this.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
                armMotorEx.setPower(0);
                direction = 0;
            }
            if (direction != 0) {
                armMotorEx.setVelocity(1000 * (direction));
                targetPosition = armMotorEx.getCurrentPosition();
            } else {
                armMotorEx.Pid(targetPosition);
            }
            this.telemetry.addData("Arm motor turning to: ", targetPosition);
            this.telemetry.addData("Arm motor is currently: ", armMotorEx.getCurrentPosition());
        }
    }

    public void moveArmToPos(int pos) {
        targetPosition = pos;
        armMotorEx.Pid(targetPosition);
    }

    public void stopArm() {
        armMotorEx.setPower(0);
    }

    public int getArmPosition() {
        return armMotorEx.getCurrentPosition();
    }

    public void setArmPositionZero() {
        isArmZeroing = true;
        lastPos = armMotorEx.getCurrentPosition();
        armMotorEx.setVelocity(10000);
    }

    public void moveClaw(double direction) {
        claw.setPosition(direction);
    }

    public void moveWrist(double direction) {
        wrist.setPower(direction);
    }

    public double getYaw() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public double getPitch() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getPitch(AngleUnit.DEGREES);
    }
    public double getRoll() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getRoll(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void runEveryLoop() {
        this.telemetry.addData("Robot speed: ", driveSpeed);

        if (isArmZeroing && (armTime.seconds() > 0.2)) {
            double pos = armMotorEx.getCurrentPosition();
            if ((Math.abs(pos - lastPos)) < 2) {
                armMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetPosition = 0;
                armMotorEx.setVelocity(0);

                isArmZeroing = false;
            }
            lastPos = pos;
            armTime.reset();

            this.telemetry.addData("currentPos: ", pos);
            this.telemetry.addData("lastPos: ", lastPos);
        }
    }
}