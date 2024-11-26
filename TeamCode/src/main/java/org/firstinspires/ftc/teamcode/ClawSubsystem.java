package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem {

    public static final String INTAKE_SERVO = "claw";

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Servo servo;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.servo = hardwareMap.get(Servo.class, INTAKE_SERVO);
    }

    public void open() {
        servo.setPosition(1);
    }

    public void close() {
        servo.setPosition(.5);
    }

}

