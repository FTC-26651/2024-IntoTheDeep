package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoTwo;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto by Time - Sample Start, near post", group = "Robot")
public class TimeBasedSample extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    double speed = 0.5;
    double short_turn = 0.2;
    double long_turn = 1.0;
    double path_to_submersible = 0.65;
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new LeoTwo(this.hardwareMap, this.telemetry);
        robot.setName("Leo Two");
        robot.initRobot();

        robot.moveClaw(1);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        // Step 1 - Drive Forward //
        robot.move(0.0, speed, 0.0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2  - Turn Left //
        robot.move(0.0, 0.0, -1 * speed);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < short_turn) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3 - Drive Forward //
        robot.move(0.0, speed, 0.0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.55) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4  - Turn Right //
        robot.move(0.0, 0.0, speed);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < short_turn) {
            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 5 - Drive Forward //
        robot.move(0.0, speed, 0.0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < path_to_submersible) {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 6  - Turn to submersible //
        robot.move(0.0, 0.0, speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < long_turn)) {
            telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 7 - Drive to Submersible //
        robot.move(0.0, speed, 0.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 7: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 8 - Stop //
        robot.move(0.0, 0.0, 0.0);

        // Step 9 - Lift the wrist out of the way //
        robot.moveWrist(-1);

        // Step 10 - Touch the Submersible //
        while (opModeIsActive() && robot.getArmPosition() < 2900) {
            robot.moveArm(0.7);
            telemetry.addData("Arm motor is currently at: ", robot.getArmPosition());
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
