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
    public CRServo  claw           = null;
    public Servo    wrist          = null;


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    double targetPosition = 0;
    final double armUp = 0.25;
    final double armDown = 0.25;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        leftDrive        = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive       = hardwareMap.get(DcMotor.class, "right_front_drive");
        armMotor         = hardwareMap.get(DcMotor.class, "left_arm");
        extensionMotor   = hardwareMap.get(DcMotor.class, "extender");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx)armMotor).setCurrentAlert(9.2, CurrentUnit.AMPS);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            forward = -gamepad1.left_stick_y;
            rotate  = gamepad1.right_stick_x;

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

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            if(gamepad1.right_bumper){
                armPosition = ARM_COLLECT;
            }

            else if (gamepad1.left_bumper){
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.y){
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
            }

            else if (gamepad1.dpad_right){
                armPosition = ARM_SCORE_SPECIMEN;
            }

            else if (gamepad1.dpad_up){
                armPosition = ARM_ATTACH_HANGING_HOOK;
            }

            else if (gamepad1.dpad_down){
                armPosition = ARM_WINCH_ROBOT;
            }

            targetPosition = armPosition + armPositionFudgeFactor;
            if (!(((DcMotorEx) armMotor).isOverCurrent())){
                armMotor.setTargetPosition((int) (targetPosition));
            }
            else {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("Moving to: ", armMotor.getTargetPosition());
            telemetry.addData("Currently: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}