package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Drive", group="Robot")
public class OpModeMecanum extends OpMode {
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;
	private DcMotor leftBackDrive;
	private DcMotor rightBackDrive;

	private DcMotor shoulder;

	private IMU imu;

	@Override
	public void init() {
		leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
		leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
		rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

		leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
		leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
		rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

		/*
		leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		*/

		imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);

	}

	@Override
	public void loop() {
		if((gamepad1.left_stick_x > 0) ||
			(gamepad1.left_stick_y > 0) ||
			(gamepad1.right_stick_x > 0)) {

			double translation_x = -gamepad1.left_stick_x;
			double translation_y = gamepad1.left_stick_y;
			double rotation = -gamepad1.right_stick_x;

			// Clamp all values to between 0 and 1
			double leftFrontPower = Range.clip(translation_y + translation_x + rotation, -1.0, 1.0);
			double rightFrontPower = Range.clip(translation_y - translation_x - rotation, -1.0, 1.0);
			double leftBackPower = Range.clip(translation_y - translation_x + rotation, -1.0, 1.0);
			double rightBackPower = Range.clip(translation_y + translation_x - rotation, -1.0, 1.0);

			leftFrontDrive.setPower(leftFrontPower);
			rightFrontDrive.setPower(rightFrontPower);
			leftBackDrive.setPower(leftBackPower);
			rightBackDrive.setPower(rightBackPower);
		}
	}
}
