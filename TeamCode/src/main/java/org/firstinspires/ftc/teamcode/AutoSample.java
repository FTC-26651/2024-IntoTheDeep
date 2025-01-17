package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.autoHelpMove;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto by Time - Sample Start", group = "Robot")
public class AutoSample extends LinearOpMode {
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

        /* Sample */
        auto.moveUntilDistAndArm(25, 4000);
//        auto.driveUntilDist(25);
//        auto.moveArmToPos(4000);
        auto.openClaw();

        /* Specimen 1 */
        auto.turn(90, false);
    }
}
