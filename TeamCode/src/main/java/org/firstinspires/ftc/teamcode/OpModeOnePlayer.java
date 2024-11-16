package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="One Player Op Mode", group="Robot")

public class OpModeOnePlayer extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftDrive      = null;
    public DcMotor  rightDrive     = null;
    public DcMotor  armMotor       = null;
    public DcMotor  extensionMotor = null;
    public CRServo  wrist          = null;
    public Servo    claw           = null;

    double targetPosition = 0;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        leftDrive      = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive     = hardwareMap.get(DcMotor.class, "right_front_drive");
        armMotor       = hardwareMap.get(DcMotor.class, "left_arm");
        extensionMotor = hardwareMap.get(DcMotor.class, "extender");
        wrist          = hardwareMap.get(CRServo.class, "wrist");
        claw           = hardwareMap.get(Servo.class, "claw");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) armMotor).setCurrentAlert(9.2, CurrentUnit.AMPS);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            ///////////////
            // --DRIVE-- //
            ///////////////

            forward = -gamepad1.left_stick_y;
            rotate  = gamepad1.left_stick_x;

            left  = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftDrive.setPower(left);
            rightDrive.setPower(right);

            /////////////
            // --ARM-- //
            /////////////

            if (gamepad1.left_bumper)
            {
                extensionMotor.setPower(1);
            }
            else if (gamepad1.right_bumper)
            {
                extensionMotor.setPower(-1);
            }
            else
            {
                extensionMotor.setPower(0);
            }

            if ((gamepad1.right_trigger + (-gamepad1.left_trigger)) == 0)
            {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition((int) targetPosition);
            }
            else
            {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(gamepad1.right_trigger + (-gamepad1.left_trigger));
                targetPosition = armMotor.getCurrentPosition();
            }

            if ((((DcMotorEx) armMotor).isOverCurrent())){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("Power: ", armMotor.getPower());
            telemetry.addData("Arm motor turning to: ", armMotor.getTargetPosition());
            telemetry.addData("Arm motor is currently: ", armMotor.getCurrentPosition());
            telemetry.update();

            //////////////
            // --CLAW-- //
            //////////////

            if (gamepad1.x)
            {
                wrist.setPower(1);
            }
            else if (gamepad1.y)
            {
                wrist.setPower(-1);
            }
            else
            {
                wrist.setPower(0);
            }

            if (gamepad1.a)
            {
                claw.setPosition(1);
            }
            else if (gamepad1.b)
            {
                claw.setPosition(0);
            }
        }
    }
}