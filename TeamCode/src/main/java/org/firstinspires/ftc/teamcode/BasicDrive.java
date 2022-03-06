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
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Basic Drive")
//@Disabled
public class BasicDrive extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final OurBot robot = new OurBot();
    private double basePower = 0.8;
    private boolean tapeMeasureMode = false;

    private final double FAST_DRIVE_POWER = 0.8;
    private final double SLOW_DRIVE_POWER = 0.5;
    private final double ARM_POWER = 0.4;
    private final double INTAKE_POWER = 0.8;
    private final double DUCK_SPINNER_POWER = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Seb", "Is Cool");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.armHold.setPosition(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Adjust power
            if (gamepad1.dpad_left)
            {
                basePower = SLOW_DRIVE_POWER;
            }
            else if (gamepad1.dpad_right)
            {
                basePower = FAST_DRIVE_POWER;
            }

            if (gamepad1.triangle)
            {
                tapeMeasureMode = true;
            }
            else if (gamepad1.square)
            {
                tapeMeasureMode = false;
            }

            robot.tapeMeasureExtend.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            if (tapeMeasureMode)
            {
                robot.tapeMeasureUpDown.setPower(gamepad1.left_stick_y * 0.9);
                robot.tapeMeasureRotate.setPower(-gamepad1.right_stick_x * 0.7);
            }
            else
            {
                // Drive
                double drive = gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;
                double leftPower = Range.clip(drive + turn, -1.0, 1.0);
                double rightPower = Range.clip(drive - turn, -1.0, 1.0);
                leftPower *= basePower;
                rightPower *= basePower;

                robot.leftFront.setPower(leftPower);
                robot.leftBack.setPower(leftPower);
                robot.rightFront.setPower(rightPower);
                robot.rightBack.setPower(rightPower);
            }

            // Controller 2 controls
            double armPower = ARM_POWER * gamepad2.left_stick_y;
            robot.arm1.setPower(armPower);
            robot.arm2.setPower(armPower);
            robot.intake.setPower(INTAKE_POWER * gamepad2.right_stick_y);
            robot.duckSpinner.setPower(DUCK_SPINNER_POWER * (gamepad2.left_trigger - gamepad2.right_trigger));
            if(gamepad2.triangle)
            {
                robot.armHold.setPosition(1);
            }

            if(gamepad2.square)
            {
                robot.armHold.setPosition(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Power", basePower);
            telemetry.addData("Mode", tapeMeasureMode ? "Tape Measure" : "Drive");
            telemetry.update();

            // Sleep to make the loop have more consistent timing
            // Possibly have to remove if things are updating too slow
            sleep(5);
            idle();
        }
    }
}
