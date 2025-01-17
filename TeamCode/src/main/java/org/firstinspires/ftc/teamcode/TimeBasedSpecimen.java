package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto by Time - Specimen Start, near post", group = "Robot")
public class TimeBasedSpecimen extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    double speed = 0.5;
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new LeoOne(this.hardwareMap, this.telemetry);
        robot.setName("LeoOne");
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
