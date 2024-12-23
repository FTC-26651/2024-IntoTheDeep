package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.extensions.LionsDcMotorEx;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LeoOne extends Robot {
    double targetPosition;

    double lastPos;
    boolean isArmZeroing = false;

    private final ElapsedTime extendTime = new ElapsedTime();
    private final ElapsedTime armTime = new ElapsedTime();

    DcMotorEx armMotor       = null;
    DcMotorEx extensionMotor = null;
    DcMotorEx FLD            = null;
    DcMotorEx FRD            = null;
    DcMotorEx BLD            = null;
    DcMotorEx BRD            = null;
    CRServo   wrist          = null;
    Servo     claw           = null;

    LionsDcMotorEx armMotorEx   = null;
    LionsDcMotorEx frontLeftDrive   = null;
    LionsDcMotorEx frontRightDrive   = null;
    LionsDcMotorEx backLeftDrive   = null;
    LionsDcMotorEx backRightDrive   = null;

    public LeoOne(HardwareMap hm, Telemetry tm) {
        super(hm, tm);
    }
    public void initRobot() {
        armMotor = this.hardwareMap.get(DcMotorEx.class, "left_arm");
        extensionMotor = this.hardwareMap.get(DcMotorEx.class, "extender");
        FLD = this.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        FRD = this.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        BLD = this.hardwareMap.get(DcMotorEx.class, "left_back_drive");
        BRD = this.hardwareMap.get(DcMotorEx.class, "right_back_drive");
        wrist = this.hardwareMap.get(CRServo.class, "wrist");
        claw = this.hardwareMap.get(Servo.class, "claw");

        armMotorEx = new LionsDcMotorEx(armMotor);
        frontLeftDrive = new LionsDcMotorEx(FLD);
        frontRightDrive = new LionsDcMotorEx(FRD);
        backLeftDrive = new LionsDcMotorEx(BLD);
        backRightDrive = new LionsDcMotorEx(BRD);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorEx.setCurrentAlert(9.2, CurrentUnit.AMPS);

        armMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void move(double x_axis, double y_axis, double tilt) {
        double leftBackPower = Range.clip(y_axis + x_axis + tilt, -1.0, 1.0);
        double rightBackPower = Range.clip(y_axis - x_axis - tilt, -1.0, 1.0);
        double leftFrontPower = Range.clip(y_axis - x_axis + tilt, -1.0, 1.0);
        double rightFrontPower = Range.clip(y_axis + x_axis - tilt, -1.0, 1.0);

        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
    }

    public void moveWithEncoder(double backLeft, double backRight, double frontLeft, double frontRight) {
        backLeftDrive.PID(backLeft, 0.05, 0.001, 0.0005);
        backRightDrive.PID(backRight, 0.05, 0.001, 0.0005);
        frontLeftDrive.PID(frontLeft, 0.05, 0.001, 0.0005);
        frontRightDrive.PID(frontRight, 0.05, 0.001, 0.0005);

        this.telemetry.addData("FRP Power: ", frontRightDrive.getPower());
        this.telemetry.addData("FRP Delta: ", frontRightDrive.getCurrentPosition() - frontRight);
        this.telemetry.addData("FRP Pos: ", frontRightDrive.getCurrentPosition());
        this.telemetry.update();

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

    public void extendArm(int inOrOut) {
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
                armMotorEx.PID(targetPosition, 0.0065, 0.001, 0.00015);
            }
            this.telemetry.addData("Arm motor turning to: ", targetPosition);
            this.telemetry.addData("Arm motor is currently: ", armMotorEx.getCurrentPosition());
        }
    }

    public void moveClaw(int direction) {
        claw.setPosition(direction);
    }

    public void moveWrist(int direction) {
        wrist.setPower(direction);
    }

    public int getArmPosition() {
        return armMotorEx.getCurrentPosition();
    }

    public void setArmPositionZero() {
        isArmZeroing = true;
        lastPos = armMotorEx.getCurrentPosition();
        armMotorEx.setVelocity(1000 * (-0.8));
    }

    public void runEveryLoop() {
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