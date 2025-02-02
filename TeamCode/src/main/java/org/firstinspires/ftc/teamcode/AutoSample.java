package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.autoHelpMove;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto - Sample Start", group = "Robot")
public class AutoSample extends LinearOpMode {
    Robot robot;

    autoHelpMove auto;

    @Override
    public void runOpMode() {
        robot = new LeoOne(this.hardwareMap, this.telemetry);
        robot.setName("Leo One");
        robot.initRobot();

        ((LeoOne)robot).initRobotEncoders();

        auto = new autoHelpMove(this, robot);

        auto.closeClaw();

        waitForStart();

        /* Sample */
        //auto.moveUntilDistAndArm(25, 4000);
        auto.driveUntilDist(25.75);
        auto.moveArmToPos(1800);
        robot.moveWrist(0.5);
        auto.killArmToPos(3600);
        auto.openClaw();
//
//        /* Specimen 1 */
//        auto.turn(90, false);
//        auto.driveInches(50);
//        auto.turn(90, true);
//        auto.moveArmToPos(5000);
//        auto.closeClaw();
//
//        auto.turnAndArm(120, false, 2500);
//        auto.extendToPos(2000);
//        auto.openClaw();
//
//        /* Park */
//        auto.turnAndArm(180, true, 0);
//        auto.driveInches(45);
//        auto.turn(30, true);
//        auto.moveArmToPos(2900);
        sleep(100000);
    }
}
