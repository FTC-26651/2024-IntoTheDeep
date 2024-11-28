package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="One Player Op Mode", group="Robot")

public class OpModeOnePlayer extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorEx armMotor       = null;
    public DcMotor   extensionMotor = null;
    public DcMotor   leftDrive      = null;
    public DcMotor   rightDrive     = null;
    public CRServo   wrist          = null;
    public Servo     claw           = null;

    int targetPosition = 0;

    double Kp = 0.008;
    double Ki = 0.0015;
    double Kd = 0.00015;

    double error;
    double derivative;
    double out;

    double integralSum;

    double lastError;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        armMotor       = hardwareMap.get(DcMotorEx.class, "left_arm");
        extensionMotor = hardwareMap.get(DcMotor.class, "extender");
        leftDrive      = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive     = hardwareMap.get(DcMotor.class, "right_front_drive");
        wrist          = hardwareMap.get(CRServo.class, "wrist");
        claw           = hardwareMap.get(Servo.class, "claw");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setCurrentAlert(9.2, CurrentUnit.AMPS);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ///////////////
            // --DRIVE-- //
            ///////////////

            forward = -gamepad2.left_stick_y;
            rotate  = gamepad2.right_stick_x;

            left  = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            leftDrive.setPower(left);
            rightDrive.setPower(right);

            /////////////
            // --ARM-- //
            /////////////

            if (gamepad1.left_bumper) {
                extensionMotor.setPower(1);
            }
            else if (gamepad1.right_bumper) {
                extensionMotor.setPower(-1);
            }
            else {
                extensionMotor.setPower(0);
            }

            if (armMotor.isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(0);
            } else if (gamepad1.left_trigger > 0  && armMotor.getCurrentPosition() < 0) {
                telemetry.addLine("ARM TRYING TO GO BEYOND DOCK");
                targetPosition = 10;
            } else if (gamepad1.right_trigger > 0  && armMotor.getCurrentPosition() >= 5000) {
                telemetry.addLine("ARM TRYING TO GO BEYOND CURRENT LIMIT");
                targetPosition = 4090;
            } else {
                if (gamepad1.right_trigger - gamepad1.left_trigger == 0) {
                    if (armMotor.getCurrentPosition() != targetPosition) {
                        error = targetPosition - armMotor.getCurrentPosition();
                        integralSum = integralSum + (error * timer.seconds());
                        derivative = (error - lastError) / timer.seconds();

                        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                        armMotor.setPower(out);

                        lastError = error;

                        timer.reset();
                    }
                } else {
                    armMotor.setVelocity(1000 * (gamepad1.right_trigger - gamepad1.left_trigger));
                    targetPosition = armMotor.getCurrentPosition();
                }
            }

            // send telemetry to the driver of the arm's current position and target position
            telemetry.addData("Arm motor turning to: ", targetPosition);
            telemetry.addData("Arm motor is currently: ", armMotor.getCurrentPosition());
            telemetry.update();

            //////////////
            // --CLAW-- //
            //////////////

            if (gamepad1.x) {
                wrist.setPower(1);
            } else if (gamepad1.y) {
                wrist.setPower(-1);
            } else {
                wrist.setPower(0);
            }

            if (gamepad1.a) {
                claw.setPosition(1);
            } else if (gamepad1.b) {
                claw.setPosition(0);
            }
        }
    }
}