package org.firstinspires.ftc.teamcode;

import android.net.wifi.aware.PublishConfig;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveOpmode extends OpMode {

    private DriveSubsystem driveSubsystem;

    @Override
    public void init() {
        driveSubsystem = new DriveSubsystem();
        driveSubsystem.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        driveSubsystem.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
    }

    @Override
    public void stop() {
        driveSubsystem.setPower(0, 0, 0, 1);
    }
}