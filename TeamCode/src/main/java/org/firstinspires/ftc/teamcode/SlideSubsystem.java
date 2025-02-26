package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideSubsystem {

    private final DcMotor slideMotor;
    private final Telemetry telemetry;

    public static final String SLIDE_MOTOR="slide_motor";

    public static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    public static final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    public static final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    public static final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    public SlideSubsystem (HardwareMap hm, Telemetry telemetry){
        this.telemetry = telemetry;

        slideMotor = hm.get(DcMotor.class, SLIDE_MOTOR);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changePosition(int positionDelta){
        this.liftPosition = this.liftPosition + positionDelta;
        setPosition(this.liftPosition);
    }

    public void setPower(double power) {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(power);
    }

    public void setPosition(double liftPosition) {
        slideMotor.setTargetPosition((int) (liftPosition));

        ((DcMotorEx) slideMotor).setVelocity(1000);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getPosition() {
        return slideMotor.getCurrentPosition();
    }

    public void writeTelemetry() {
        telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
    }
}
