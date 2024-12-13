package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.SlideSubsystem;
import org.firstinspires.ftc.teamcode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.WristSubsystem;

@TeleOp
public class BetaDriveThing extends LinearOpMode {
    private double iF;
    private double iR;
    ArmSubsystem armSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        final DcMotor rearRight = hardwareMap.dcMotor.get("backRightMotor");
        final DcMotor rearLeft = hardwareMap.dcMotor.get("backLeftMotor");
        final DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap, telemetry);
        WristSubsystem wrist = new WristSubsystem(hardwareMap, telemetry);
        ClawSubsystem claw = new ClawSubsystem(hardwareMap, telemetry);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y * 0.5;  // Forward/backward
            double x = gamepad1.left_stick_x * 0.5;  // Left/right (strafing)
            double rx = gamepad1.right_stick_x;  // Turning


            double turningSpeed = 0.3;
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

            if (gamepad1.dpad_left) {
                claw.open();
            } else if (gamepad1.dpad_right) {
                claw.close();
            }


            if (gamepad2.a) {
                arm.setArmPosition(ArmSubsystem.Arm_Start_Position);
            } else if (gamepad2.y) {
                arm.setArmPosition(ArmSubsystem.Arm_Collect_Position);
            } else if (gamepad2.x) {
                arm.setArmPosition(ArmSubsystem.Arm_Low_Bucket);
            } else if (gamepad2.b) {
                arm.setArmPosition(ArmSubsystem.Arm_High_Bucket);
            } else if (gamepad2.dpad_up) {
                arm.setArmPosition(ArmSubsystem.Arm_Low_Bar);
            } else if (gamepad2.dpad_down) {
                arm.setArmPosition(ArmSubsystem.Arm_Clear_Barrier);
            } else if (gamepad2.start) {
                arm.resetArmEncoder();


                if (gamepad2.right_stick_button) {
                    wrist.moveToPosition(.75);
                } else if (gamepad2.left_stick_button) {
                    wrist.moveToPosition(0);
                }

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontRightPower = (y + x + rx) / denominator;
                double rearRightPower = (y - x + rx) / denominator;
                double rearLeftPower = (y + x - rx) / denominator;
                double frontLeftPower = (y - x - rx) / denominator;

                frontRight.setPower(frontRightPower);
                rearRight.setPower(rearRightPower);
                rearLeft.setPower(rearLeftPower);
                frontLeft.setPower(frontLeftPower);


            }
        }
    }
}
