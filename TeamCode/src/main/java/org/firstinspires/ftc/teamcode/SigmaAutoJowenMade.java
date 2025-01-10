package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class SigmaAutoJowenMade extends LinearOpMode {
    private SlideSubsystem slideSubsystem;
    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private ClawSubsystem clawSubsystem;

    @Override
    public void runOpMode() {
        DcMotor front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor back_right_motor = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor back_left_motor = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;


        armSubsystem.setArmPosition(ArmSubsystem.Arm_High_Bucket);
        sleep(2000);
        slideSubsystem.setPower(1);
        sleep(2000);
        wristSubsystem.moveToPosition(0);
        sleep(500);
        clawSubsystem.open();
        sleep(250);
        telemetry.addData("status", "auto finish");
        telemetry.update();


    }

    private void moveDrivetrain(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor, double power, int duration) {
        front_left_motor.setPower(power);
        back_left_motor.setPower(power);
        front_right_motor.setPower(power);
        back_right_motor.setPower(power);
        sleep(duration);
    }

    private void stopDrivetrain(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor) {
        front_left_motor.setPower(0);
        back_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);
    }

    private void rotateDrivetrainLeft(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor, double power, int duration) {
        front_left_motor.setPower(-power);
        back_left_motor.setPower(-power);
        front_right_motor.setPower(power);
        back_right_motor.setPower(power);
        sleep(duration);
    }

    private void rotateDrivetrainRight(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor, double power, int duration) {
        front_left_motor.setPower(power);
        back_left_motor.setPower(power);
        front_right_motor.setPower(-power);
        back_right_motor.setPower(-power);
        sleep(duration);
    }


}