package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.SlideSubsystem;
import org.firstinspires.ftc.teamcode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.WristSubsystem;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class SigmaAutoOwenMade extends LinearOpMode {
    // Declare variables
    private SlideSubsystem slideSubsystem;
    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private ClawSubsystem clawSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
        telemetry.addData("status", "auto finish");
        telemetry.update();


    }

    private void moveDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) throws InterruptedException {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        sleep(duration);
    }

    private void stopDrivetrain(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight) {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    private void rotateDrivetrainLeft(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) throws InterruptedException {
        frontLeft.setPower(-power);
        rearLeft.setPower(-power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        sleep(duration);
    }

    private void rotateDrivetrainRight(DcMotor frontLeft, DcMotor rearLeft, DcMotor frontRight, DcMotor rearRight, double power, int duration) throws InterruptedException {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(-power);
        rearRight.setPower(-power);
        sleep(duration);
    }


}