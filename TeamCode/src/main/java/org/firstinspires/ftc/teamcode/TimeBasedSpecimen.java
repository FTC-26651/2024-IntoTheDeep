package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoTwo;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto by Time - Specimen Start, near post", group = "Robot")
public class TimeBasedSpecimen extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    double speed = 0.5;
    double short_turn = 0.4;
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

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
