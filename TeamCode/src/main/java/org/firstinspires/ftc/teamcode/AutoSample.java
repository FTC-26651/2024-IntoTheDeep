package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.LeoOne;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.autoHelpMove;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto - ample Start", group = "Robot")
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
        //auto.moveUntilDistAndArm(25, 4000);
        robot.moveWrist(2);
        //auto.driveUntilDist(20);
        //auto.moveArmToPos(2500);
//        auto.openClaw();
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
