package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto by Time - Specimen Start", group = "Robot")
public class TimeBasedSpecimen extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new LeoTwo(this.hardwareMap, this.telemetry);
        robot.setName("Leo Two");
        robot.initRobot();

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        // Step 1 //
        robot.move(0.0, 0.5, 0.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//        // Step 2 //
//        robot.move(0.0, 0.0, -0.5);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.57)) {
//            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3 //
//        robot.move(0.0, 0.5, 0.0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4 //
//        robot.move(0.0, 0.0, 0.5);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.57)) {
//            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 5 //
//        robot.move(0.0, 0.5, 0.0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 6 //
//        robot.move(0.0, 0.0, 0.5);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.57)) {
//            telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 7 //
//        robot.move(0.0, 0.5, 0.0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg : %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 8 //
//        robot.move(0.0, 0.0, 0.0);
//
//        // Step 9 //
//        robot.moveWrist(-1);
//
//        // Step 10 //
//        while (opModeIsActive() && robot.getArmPosition() < 4000) {
//            robot.moveArm(0.7);
//            telemetry.addData("Arm motor is currently at: ", robot.getArmPosition());
//        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
