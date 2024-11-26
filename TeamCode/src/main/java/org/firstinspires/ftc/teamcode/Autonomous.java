package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    private DriveSubsystem driveSubsystem;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        runtime.reset();
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

    while (opModeIsActive())
        driveSubsystem.driveForward(1, 2);
        while (driveSubsystem.isMoving()) sleep(1);
        driveSubsystem.strafeLeft(1, -24);
        while (driveSubsystem.isMoving()) sleep(1);

    }
}
//yhyuhuhuhuhuh