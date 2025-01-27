package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.extensions.LionsDcMotorEx;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LeoOne extends Robot {
    double armTargetPosition;
    double extendTargetPosition;
    double lastArmPos;
    double lastExtendPos;

    double driveSpeed = 1;

    boolean isArmZeroing = false;
    boolean extensionAtZero = false;

    private final ElapsedTime armTime = new ElapsedTime();
    private final ElapsedTime extendTime = new ElapsedTime();

    DcMotorEx armMotor = null;
    DcMotorEx exM      = null;
    DcMotorEx FLD      = null;
    DcMotorEx FRD      = null;
    DcMotorEx BLD      = null;
    DcMotorEx BRD      = null;
    Servo     wrist    = null;
    Servo     claw     = null;

    LionsDcMotorEx armMotorEx   = null;
    LionsDcMotorEx extensionMotor = null;
    LionsDcMotorEx frontLeftDrive   = null;
    LionsDcMotorEx frontRightDrive   = null;
    LionsDcMotorEx backLeftDrive   = null;
    LionsDcMotorEx backRightDrive   = null;

    IMU imu;
    DistanceSensor distanceSensor;

    YawPitchRollAngles orientation;

    public LeoOne(HardwareMap hm, Telemetry tm) {
        super(hm, tm);
    }

    public void initRobotEncoders() {
        armMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void initRobot() {
        armMotor = this.hardwareMap.get(DcMotorEx.class, "left_arm");
        exM = this.hardwareMap.get(DcMotorEx.class, "extender");
        FLD = this.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        FRD = this.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        BLD = this.hardwareMap.get(DcMotorEx.class, "left_back_drive");
        BRD = this.hardwareMap.get(DcMotorEx.class, "right_back_drive");
        wrist = this.hardwareMap.get(Servo.class, "wrist");
        claw = this.hardwareMap.get(Servo.class, "claw");

        imu = this.hardwareMap.get(IMU.class, "imu");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");

        armMotorEx = new LionsDcMotorEx(armMotor);
        extensionMotor = new LionsDcMotorEx(exM);
        frontLeftDrive = new LionsDcMotorEx(FLD);
        frontRightDrive = new LionsDcMotorEx(FRD);
        backLeftDrive = new LionsDcMotorEx(BLD);
        backRightDrive = new LionsDcMotorEx(BRD);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotorEx.setDirection(DcMotor.Direction.FORWARD);
        extensionMotor.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);

        armMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorEx.setCurrentAlert(9.2, CurrentUnit.AMPS);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotorEx.setPid(0.0025, 0.001, 0.00015);
        extensionMotor.setPid(0.00009, 0.00005, 0.00005);
        frontLeftDrive.setPid(0.05, 0.001, 0.001);
        frontRightDrive.setPid(0.05, 0.001, 0.001);
        backLeftDrive.setPid(0.05, 0.001, 0.001);
        backRightDrive.setPid(0.05, 0.001, 0.001);

        claw.scaleRange(0.30, 1);
        wrist.scaleRange(0.25, 0.75);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        orientation = imu.getRobotYawPitchRollAngles();

        //extensionMotor.setPower(-0.1);
    }

    public void move(double x_axis, double y_axis, double tilt) {
        double leftBackPower = Range.clip(y_axis + x_axis + tilt, -1.0, 1.0);
        double rightBackPower = Range.clip(y_axis - x_axis - tilt, -1.0, 1.0);
        double leftFrontPower = Range.clip(y_axis - x_axis + tilt, -1.0, 1.0);
        double rightFrontPower = Range.clip(y_axis + x_axis - tilt, -1.0, 1.0);

        backLeftDrive.setPower(leftBackPower * driveSpeed);
        backRightDrive.setPower(rightBackPower * driveSpeed);
        frontLeftDrive.setPower(leftFrontPower * driveSpeed);
        frontRightDrive.setPower(rightFrontPower * driveSpeed);
    }

    public void move(double leftBackPower, double rightBackPower, double leftFrontPower, double rightFrontPower) {
        leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
        rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);
        leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
        rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);

        backLeftDrive.setPower(leftBackPower * driveSpeed);
        backRightDrive.setPower(rightBackPower * driveSpeed);
        frontLeftDrive.setPower(leftFrontPower * driveSpeed);
        frontRightDrive.setPower(rightFrontPower * driveSpeed);
    }

    public void moveWithEncoder(double backLeft, double backRight, double frontLeft, double frontRight) {
        backLeftDrive.Pid(backLeft);
        backRightDrive.Pid(backRight);
        frontLeftDrive.Pid(frontLeft);
        frontRightDrive.Pid(frontRight);
    }

    public void stopDrive() {
        backLeftDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontRightDrive.setPower(0);
    }

    public int getTicksInInch() {
        return 58;
    }

    public int getBackLeftPos() {
        return backLeftDrive.getCurrentPosition();
    }
    public int getBackRightPos() {
        return backRightDrive.getCurrentPosition();
    }
    public int getFrontLeftPos() {
        return frontLeftDrive.getCurrentPosition();
    }
    public int getFrontRightPos() {
        return frontRightDrive.getCurrentPosition();
    }

    public void resetDriveEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double extendPos = extensionMotor.getCurrentPosition();
        if ((Math.abs(extensionMotor.getCurrentPosition() - lastExtendPos)) > 2) {
            extensionAtZero = false;
        }

        if (!isArmZeroing &&
                armMotorEx.getCurrentPosition() < 2800 &&
                armMotorEx.getCurrentPosition() > 2300 &&
                getPitch() < 15
        ) {
            if (inOrOut != 0 && extensionMotor.getCurrentPosition() < 1200) {
                extensionMotor.setPower(inOrOut);
                extendTargetPosition = extensionMotor.getCurrentPosition();
            } else {
                extensionMotor.Pid(extendTargetPosition);
            }
            extendTime.reset();
        } else if (!extensionAtZero) {
            if (extendTime.seconds() > 0.2) {
                if ((Math.abs(extendPos - lastExtendPos)) < 2) {
                    extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendTargetPosition = 0;
                    extensionMotor.Pid(0);
                    extensionAtZero = true;
                } else {
                    extensionMotor.setPower(-1);
                }
                lastExtendPos = extendPos;
                extendTime.reset();
            }
        }
        this.telemetry.addData("extendPos: ", lastExtendPos);
        this.telemetry.addData("Ex motor is currently: ", extensionMotor.getCurrentPosition());
    }
    
    public int getExtendPosition() {
        return extensionMotor.getCurrentPosition();
    }

    public void extendToPos(int pos) {
        extendTargetPosition = pos;
        extensionMotor.Pid(extendTargetPosition);
    }

    public void stopExtend() {
        extensionMotor.setPower(0);
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
                armTargetPosition = armMotorEx.getCurrentPosition();
            } else {
                armMotorEx.Pid(armTargetPosition);
            }
        }
    }

    public void moveArmToPos(int pos) {
        armTargetPosition = pos;
        armMotorEx.Pid(armTargetPosition);
        telemetry.addData("TArmget pos: ", armTargetPosition);
        telemetry.addData("Arm pos: ", armMotorEx.getCurrentPosition());
        telemetry.update();
    }

    public int getArmPosition() {
        return armMotorEx.getCurrentPosition();
    }

    public void stopArm() {
        armMotorEx.setPower(0);
    }

    public void setArmPositionZero() {
        isArmZeroing = true;
        lastArmPos = armMotorEx.getCurrentPosition();
        armMotorEx.setVelocity(1000 * (-0.8));
    }

    public void moveClaw(double direction) {
        claw.setPosition(direction);

    }

    public void moveWrist(double direction) {
        if (armMotorEx.getCurrentPosition() < 1500) {
            wrist.setPosition(1);
        } else {
            wrist.setPosition(direction);
        }
    }

    public double getDist() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
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

        this.telemetry.addData("Extend Power: ", extensionMotor.getPower());

        this.telemetry.addData("currentPos: ", armMotorEx.getCurrentPosition());
        this.telemetry.addData("lastPos: ", lastArmPos);

        if (isArmZeroing && (armTime.seconds() > 0.2)) {
            double pos = armMotorEx.getCurrentPosition();
            if ((Math.abs(pos - lastArmPos)) < 2) {
                armMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armTargetPosition = 0;
                armMotorEx.setVelocity(0);
                isArmZeroing = false;
            }
            lastArmPos = pos;
            armTime.reset();
        }
    }
}
