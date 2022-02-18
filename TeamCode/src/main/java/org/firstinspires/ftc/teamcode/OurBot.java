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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class OurBot
{
    public static final double COUNTS_PER_MOTOR_REV = 28;    // REV Ultraplanetary
    public static final double DRIVE_GEAR_REDUCTION = 2.89 * 3.61;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 3.54330709;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) * -1; // Negated because the encoders are backwards for some reason
    public static final double INTAKE_COUNTS_PER_REV = 288;

    /* Public OpMode members. */
    public DcMotor  leftFront           = null;
    public DcMotor  leftBack            = null;
    public DcMotor  rightFront          = null;
    public DcMotor  rightBack           = null;
    public DcMotor  arm                 = null;
    public DcMotor  intake              = null;
    public DcMotor  duckSpinner         = null;

    public Servo    armHold             = null;
    public CRServo  tapeMeasureRotate   = null;
    public CRServo  tapeMeasureUpDown   = null;
    public CRServo  tapeMeasureExtend   = null;

    /* local OpMode members. */
    HardwareMap hwMap           = null;

    /* Constructor */
    public OurBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define Motors
        leftFront           = hwMap.get(DcMotor.class, "leftFront");
        leftBack            = hwMap.get(DcMotor.class, "leftBack");
        rightFront          = hwMap.get(DcMotor.class, "rightFront");
        rightBack           = hwMap.get(DcMotor.class, "rightBack");
        arm                 = hwMap.get(DcMotor.class, "arm");
        intake              = hwMap.get(DcMotor.class, "intake");
        duckSpinner         = hwMap.get(DcMotor.class, "duckSpinner");
        armHold             = hwMap.get(Servo.class, "armHold");
        tapeMeasureRotate   = hwMap.get(CRServo.class, "tapeMeasureRotate");
        tapeMeasureUpDown   = hwMap.get(CRServo.class, "tapeMeasureUpDown");
        tapeMeasureExtend   = hwMap.get(CRServo.class, "tapeMeasureExtend");

        //Initialize Motor Direction
        leftFront.setDirection(Direction.REVERSE);
        leftBack.setDirection(Direction.REVERSE);
        rightFront.setDirection(Direction.FORWARD);
        rightBack.setDirection(Direction.FORWARD);
        arm.setDirection(Direction.REVERSE);
        intake.setDirection(Direction.REVERSE);
        duckSpinner.setDirection(Direction.FORWARD);

        //Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        arm.setPower(0);
        intake.setPower(0);
        duckSpinner.setPower(0);

        //Reset all encoders
        leftFront.setMode(RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //duckSpinner.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //Set motors to run with encoder
        leftFront.setMode(RunMode.RUN_USING_ENCODER);
        leftBack.setMode(RunMode.RUN_USING_ENCODER);
        rightFront.setMode(RunMode.RUN_USING_ENCODER);
        rightBack.setMode(RunMode.RUN_USING_ENCODER);
        arm.setMode(RunMode.RUN_USING_ENCODER);

        //Intake and Duck Spinner don't have encoders
        duckSpinner.setMode(RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
    }
 }

