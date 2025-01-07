package org.firstinspires.ftc.teamcode.robot.extensions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.libs.pidLib;

public class LionsDcMotorEx implements DcMotorEx {
    private final DcMotorEx theMotor;

    private final pidLib pid = new pidLib();

    public LionsDcMotorEx(DcMotorEx theMotor) {
        this.theMotor = theMotor;
    }

    public void setPid(double kP, double kI, double kD) {
        pid.updatePid(kP, kI, kD);
    }

    public void Pid(double target) {
        theMotor.setPower(pid.getPid(target, theMotor.getCurrentPosition()));
    }
    public void Pid(double target, double kP, double kI, double kD) {
        pid.updatePid(kP, kI, kD);
        theMotor.setPower(pid.getPid(target, theMotor.getCurrentPosition()));
    }

    // Implement ALL of the necessary methods...wugh //
    public MotorConfigurationType getMotorType() {
        return theMotor.getMotorType();
    }

    public void setMotorType(MotorConfigurationType motorType) {
        theMotor.setMotorType(motorType);
    }

    public DcMotorController getController() {
        return theMotor.getController();
    }

    public int getPortNumber() {
        return theMotor.getPortNumber();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        theMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return theMotor.getZeroPowerBehavior();
    }

    public void setPowerFloat() {
    }

    public boolean getPowerFloat() {
        return theMotor.getPowerFloat();
    }

    public void setTargetPosition(int position) {
        theMotor.setTargetPosition(position);
    }

    public int getTargetPosition() {
        return theMotor.getTargetPosition();
    }

    public void setMotorEnable() {
        theMotor.setMotorEnable();
    }

    public void setMotorDisable() {
        theMotor.setMotorDisable();
    }

    public boolean isMotorEnabled() {
        return theMotor.isMotorEnabled();
    }

    public boolean isBusy() {
        return theMotor.isBusy();
    }

    public int getCurrentPosition() {
        return theMotor.getCurrentPosition();
    }

    public void setMode(DcMotor.RunMode mode) {
        theMotor.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return theMotor.getMode();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        theMotor.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return theMotor.getDirection();
    }

    public void setPower(double power) {
        theMotor.setPower(power);
    }

    public double getPower() {
        return theMotor.getPower();
    }

    public double getCurrent(CurrentUnit unit) {
        return theMotor.getCurrent(unit);
    }

    public double getCurrentAlert(CurrentUnit unit) {
        return theMotor.getCurrentAlert(unit);
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode mode) {
        return new PIDCoefficients();
    }

    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode) {
        return theMotor.getPIDFCoefficients(mode);
    }

    public int getTargetPositionTolerance() {
        return theMotor.getTargetPositionTolerance();
    }

    public double getVelocity() {
        return theMotor.getVelocity();
    }

    public double getVelocity(AngleUnit unit) {
        return theMotor.getVelocity(unit);
    }

    public boolean isOverCurrent() {
        return theMotor.isOverCurrent();
    }

    public void setCurrentAlert(double current, CurrentUnit unit) {
        theMotor.setCurrentAlert(current, unit);
    }

    public void setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        theMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    public void setPositionPIDFCoefficients(double p) {
        theMotor.setPositionPIDFCoefficients(p);
    }

    public void setTargetPositionTolerance(int tolerance) {
        theMotor.setTargetPositionTolerance(tolerance);
    }

    public void setVelocity(double angularRate) {
        theMotor.setVelocity(angularRate);
    }

    public void setVelocity(double angularRate, AngleUnit unit) {
        theMotor.setVelocity(angularRate, unit);
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        theMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public Manufacturer getManufacturer() {
        return theMotor.getManufacturer();
    }

    public String getDeviceName() {
        return theMotor.getDeviceName();
    }

    public String getConnectionInfo() {
        return theMotor.getConnectionInfo();
    }

    public int getVersion() {
        return theMotor.getVersion();
    }

    public void resetDeviceConfigurationForOpMode() {
        theMotor.resetDeviceConfigurationForOpMode();
    }

    public void close() {
        theMotor.close();
    }
}
