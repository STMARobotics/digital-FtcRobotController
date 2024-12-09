/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Drive Only OpMode", group="Linear OpMode")
//@Disabled
public class DriveOnlyOpMode extends LinearOpMode {


    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap, telemetry);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);
        WristSubsystem wrist = new WristSubsystem(hardwareMap, telemetry);
        ClawSubsystem claw = new ClawSubsystem(hardwareMap, telemetry);
         //Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        double iF;
        double iR;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            float forward = -gamepad1.left_stick_y;
            float strafe = gamepad1.left_stick_x;
            float turn = gamepad1.right_stick_x;

            iF = gamepad2.right_trigger;
            iR = gamepad2.left_trigger * -1;


            double reductionFactor = 1.5;
            if (gamepad1.left_bumper) {
                reductionFactor = 3.0576878987678;
            }

            double slidePower = (iF + iR);
            if (gamepad2.left_trigger > 0.1) {
                slideSubsystem.setPower(slidePower);
            } else if (gamepad2.right_trigger > 0.1) {
                slideSubsystem.setPower(slidePower);
            } else {
                slideSubsystem.setPower(0);
            }

            if (gamepad1.dpad_left){
                claw.open();
            } else if (gamepad1.dpad_right){
                claw.close();
            }

            // Handles move arm to set positions with a fudge factor
            double fudgeFactorPercentage = gamepad2.right_trigger + (-gamepad2.left_trigger);
            if (gamepad2.a){
                arm.setArmPosition(ArmSubsystem.Arm_Start_Position);
            } else if (gamepad2.y){
                arm.setArmPosition(ArmSubsystem.Arm_Collect_Position);
            } else if (gamepad2.x){
                arm.setArmPosition(ArmSubsystem.Arm_Low_Bucket);
            } else if (gamepad2.b){
                arm.setArmPosition(ArmSubsystem.Arm_High_Bucket);
            } else if (gamepad2.dpad_up) {
                arm.setArmPosition(ArmSubsystem.Arm_Low_Bar);
            } else if (gamepad2.dpad_down){
                arm.setArmPosition(ArmSubsystem.Arm_Clear_Barrier);
            } else if (gamepad2.start) {
                arm.resetArmEncoder();




            if (gamepad2.right_stick_button){
                wrist.moveToPosition(.75);
            } else if (gamepad2.left_stick_button){
                wrist.moveToPosition(0);
            }

            driveSubsystem.moveFieldCentric(strafe, forward, turn, reductionFactor);

            if (gamepad1.back) {
                driveSubsystem.resetYaw();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

}
