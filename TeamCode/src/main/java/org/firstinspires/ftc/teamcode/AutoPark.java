package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.autoHelpMove;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto - Park", group = "Robot")
public class AutoPark extends LinearOpMode {
    Robot robot;

    autoHelpMove auto;

    @Override
    public void runOpMode() {
        robot = new LeoOne(this.hardwareMap, this.telemetry);
        robot.setName("Leo One");
        robot.initRobot();

        auto = new autoHelpMove(this, robot);

        auto.closeClaw();

        waitForStart();

        auto.driveInches(60);
        auto.turn(90, true);
        auto.driveInches(10);
    }
}