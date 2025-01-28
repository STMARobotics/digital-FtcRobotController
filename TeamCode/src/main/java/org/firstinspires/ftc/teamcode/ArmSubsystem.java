package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmSubsystem {

    private final DcMotor armMotor;
    private final Telemetry telemetry;

    public static final double ARM_TICKS_PER_DEGREE = 5;
    public static final double Arm_High_Bucket = 67.5 * ARM_TICKS_PER_DEGREE;
    public static final double Arm_Start_Position = 0 * ARM_TICKS_PER_DEGREE;
    public static final double Arm_Collect_Position = -10 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    public static final double Arm_Low_Bucket = 47.5 * ARM_TICKS_PER_DEGREE;
    public static final double Arm_Clear_Barrier = 12.5 * ARM_TICKS_PER_DEGREE;
    public static final double Arm_Low_Bar = 10 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    static double FUDGE_FACTOR_DEGREES = 15;


    public static final String ARM_MOTOR = "arm_motor";

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        armMotor = hardwareMap.get(DcMotor.class, ARM_MOTOR);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setArmPosition(double position) {
        armMotor.setTargetPosition((int) (position));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getPosition() {
        return armMotor.getCurrentPosition();
    }

    public void writeTelemetry() {
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
    }
}
