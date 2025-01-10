package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class BetaDriveThing extends LinearOpMode {
    private double iF;
    private double iR;
    ArmSubsystem armSubsystem;

    @Override
    public void runOpMode() {
        final DcMotor front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
        final DcMotor back_right_motor = hardwareMap.dcMotor.get("back_right_motor");
        final DcMotor back_left_motor = hardwareMap.dcMotor.get("back_left_motor");
        final DcMotor front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
        ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap, telemetry);
        WristSubsystem wrist = new WristSubsystem(hardwareMap, telemetry);
        ClawSubsystem claw = new ClawSubsystem(hardwareMap, telemetry);
        front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y * 0.5;  // Forward/backward
            double x = gamepad1.left_stick_x * 0.5;  // Left/right (strafing)
            double rx = gamepad1.right_stick_x;  // Turning


            double turningSpeed = 0.6;
            if (Math.abs(rx) > 0.1) {
                rx = turningSpeed * Math.signum(rx);
            } else {
                rx = 0;
            }


            iF = gamepad2.right_trigger;
            iR = gamepad2.left_trigger * -1;

            double slidePower = (iF + iR);
            if (gamepad2.left_trigger > 0.1) {
                slideSubsystem.setPower(slidePower);
            } else if (gamepad2.right_trigger > 0.1) {
                slideSubsystem.setPower(slidePower);
            } else {
                slideSubsystem.setPower(0);
            }

            if (gamepad2.dpad_left) {
                claw.open();
            } else if (gamepad2.dpad_right) {
                claw.close();
            }


            if (gamepad2.x) {
                arm.setArmPosition(ArmSubsystem.Arm_Start_Position);
            } else if (gamepad2.a) {
                arm.setArmPosition(ArmSubsystem.Arm_Collect_Position);
            } else if (gamepad2.b) {
                arm.setArmPosition(ArmSubsystem.Arm_Low_Bucket);
            } else if (gamepad2.y) {
                arm.setArmPosition(ArmSubsystem.Arm_High_Bucket);
            } else if (gamepad2.left_stick_button) {
                arm.setArmPosition(ArmSubsystem.Arm_Low_Bar);
            } else if (gamepad2.right_stick_button) {
                arm.setArmPosition(ArmSubsystem.Arm_Clear_Barrier);
            } else if (gamepad2.start) {
                arm.resetArmEncoder();
            }

            if (gamepad2.dpad_up) {
                wrist.moveToPosition(.75);
            } else if (gamepad2.dpad_down) {
                wrist.moveToPosition(0);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double front_right_power = (y + x + rx) / denominator;
            double back_right_power = (y - x + rx) / denominator;
            double back_left_power = (y + x - rx) / denominator;
            double front_left_power = (y - x - rx) / denominator;

            front_right_motor.setPower(front_right_power);
            back_right_motor.setPower(back_right_power);
            back_left_motor.setPower(back_left_power);
            front_left_motor.setPower(front_left_power);
        }
    }
}
