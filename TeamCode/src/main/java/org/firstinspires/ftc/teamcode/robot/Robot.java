package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
    private char team;
    private String name;
    private String errors;
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    // Conostructor
    public Robot(HardwareMap hm, Telemetry tm) {
        hardwareMap = hm;
        telemetry = tm;
    }

    public abstract void initRobot();

    public abstract void move(double x_axis, double y_axis, double tilt);
    public abstract void extendArm(int inOrOut);
    public abstract void moveArm(double direction);
    public abstract void moveClaw(int direction);
    public abstract void moveWrist(int direction);

    public abstract int getArmPosition();

    public abstract void setArmPositionZero();

    // Define a function that is called every time through the main loop
    public void runEveryLoop() {};

    public void setTeam(char color) {
        if (color == 'b') {
            team = 'b';
        } else if (color == 'r') {
            team = 'r';
        } else {
            // Complain about an invalid team
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

    public boolean hasErrors() {
        return !(errors.isEmpty());
    }
    public void setErrors(String iErrors) {
        errors = iErrors;
    }
    public String getErrors() {
        return errors;
    }
}
