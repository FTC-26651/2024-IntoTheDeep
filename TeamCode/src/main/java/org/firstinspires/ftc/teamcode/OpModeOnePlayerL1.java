package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Leo 1 - 1P", group = "Robot")
public class OpModeOnePlayerL1 extends LinearOpMode {

    /* Declare OpMode member. */
    public CRServo wrist = null;
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new LeoOne(this.hardwareMap, this.telemetry);

        wrist = hardwareMap.get(CRServo.class, "wrist");

        robot.setName("Leo One");
        robot.initRobot();

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.runEveryLoop();
            ///////////////
            // --DRIVE-- //
            ///////////////

            robot.move(gamepad1.left_stick_x, -1 * gamepad1.left_stick_y, -1 * gamepad1.right_stick_x);

            /////////////
            // --ARM-- //
            /////////////

            robot.extendArm((gamepad1.right_bumper ? 0.5 : 0) - (gamepad1.left_bumper ? 0.5 : 0));

            robot.moveArm(gamepad1.right_trigger - gamepad1.left_trigger);

            //////////////
            // --CLAW-- //
            //////////////

            if (gamepad1.x) {
                robot.moveWrist(1);
            } else if (gamepad1.y) {
                robot.moveWrist(-0.5);
            } else {
                robot.moveWrist(0);
            }

            if (gamepad1.a) {
                robot.moveClaw(1);
            } else if (gamepad1.b) {
                robot.moveClaw(0);
            }

            if (gamepad1.start) {
                robot.setArmPositionZero();
            }

            /////////////////
            // --DISPLAY-- //
            /////////////////
            telemetry.update();
        }
    }
}