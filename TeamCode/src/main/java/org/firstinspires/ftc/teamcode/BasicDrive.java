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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="Basic Drive")
//@Disabled
public class BasicDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor armMotor = null;
    private boolean tankDrive = true;
    private double basePower = 1;
    private double lastPowerChangeTime;
    private double lastArmMoveTime;

    private final double ARM_POWER = 0.4;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Seb", "Is Cool");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        armMotor = hardwareMap.get(DcMotor.class, "arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //Reset encoders
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set all motors to use encoders
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        lastPowerChangeTime = runtime.time();
        lastArmMoveTime = runtime.time();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            // Setup a variable for each drive wheel
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            if (tankDrive)
            {
                leftPower =  gamepad1.left_stick_y;
                rightPower = gamepad1.right_stick_y;
            }
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            else
            {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);
            }

            leftPower *= basePower;
            rightPower *= basePower;

            if (runtime.time() > lastArmMoveTime + 0.1)
            {
                if (gamepad1.left_bumper)
                {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition() - 1);
                    lastPowerChangeTime = runtime.time();
                }
                if (gamepad1.right_bumper)
                {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition() + 1);
                    lastPowerChangeTime = runtime.time();
                }
            }

            // Send calculated power to wheels
            leftFrontMotor.setPower(leftPower);
            leftBackMotor.setPower(leftPower);
            rightFrontMotor.setPower(rightPower);
            rightBackMotor.setPower(rightPower);
            armMotor.setPower(ARM_POWER);

            // Switch modes
            if (gamepad1.dpad_up)
            {
                tankDrive = false;
            }
            else if (gamepad1.dpad_down)
            {
                tankDrive = true;
            }


            // Adjust Power
            if (runtime.time() > lastPowerChangeTime + 0.5)
            {
                if (gamepad1.dpad_left)
                {
                    basePower = Math.max(0, basePower - 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad1.dpad_right)
                {
                    basePower = Math.min(1, basePower + 0.1);
                    lastPowerChangeTime = runtime.time();
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Power", basePower);
            telemetry.addData("Mode", tankDrive ? "Tank Drive" : "POV Drive");
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();

            // Sleep to make the loop have more consistent timing
            // Possibly have to remove if things are updating too slow
            sleep(5);
            idle();
        }
    }
}
