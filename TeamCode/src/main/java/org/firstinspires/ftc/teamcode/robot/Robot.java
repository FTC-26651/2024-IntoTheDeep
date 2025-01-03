package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
    private char team;
    private String name;
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    // Constructor
    public Robot(HardwareMap hm, Telemetry tm) {
        hardwareMap = hm;
        telemetry = tm;
    }

    public abstract void initRobot();

    public abstract int getTicksInInch();

    public abstract void move(double x_axis, double y_axis, double tilt);
    public abstract void stopDrive();
    public abstract void resetDriveEncoders();
    public abstract void increaseMoveSpeed();
    public abstract void decreaseMoveSpeed();

    public abstract void extendArm(double inOrOut);
    public abstract void moveArm(double direction);
    public abstract void moveArmToPos(int pos);
    public abstract void stopArm();

    public abstract void moveClaw(double direction);
    public abstract void moveWrist(double direction);

    public abstract int getArmPosition();

    public abstract double getYaw();
    public abstract double getPitch();
    public abstract double getRoll();

    public abstract void resetYaw();

    public abstract void setArmPositionZero();

    // Define a function that is called every time through the main loop
    public void runEveryLoop() {}

    public void setTeam(char color) {
        if (color == 'b') {
            team = 'b';
        } else if (color == 'r') {
            team = 'r';
        } else {
            telemetry.addLine("Invalid Team");
        }
    }

    public char getTeam() {
        return team;
    }

    public void setName(String iName) {
        name = iName;
    }

    public String getName() {
        return name;
    }
}
