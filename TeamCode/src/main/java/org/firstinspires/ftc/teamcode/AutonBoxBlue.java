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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ShippingElementDetectorPipeline.ShippingElementPosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * i
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Box - Blue")
//@Disabled
public class AutonBoxBlue extends BaseAuton {
    OpenCvWebcam webcam;
    ShippingElementDetectorPipeline pipeline;
    ShippingElementPosition shippingElementPosition = ShippingElementPosition.NOT_DETECTED;

    @Override
    public void runOpMode() {
        super.runOpMode();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ShippingElementDetectorPipeline();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        // Wait for the game to start (driver presses PLAY)
        do {
            ShippingElementPosition newShippingElementPosition = pipeline.getShippingElementPosition();
            if (newShippingElementPosition != ShippingElementPosition.NOT_DETECTED) {
                shippingElementPosition = newShippingElementPosition;
            }
            sleep(50);
        } while (!isStarted());

        while(shippingElementPosition == ShippingElementPosition.NOT_DETECTED)
        {
            shippingElementPosition = pipeline.getShippingElementPosition();
            sleep(50);
        }


        //poopoo
        // plus minus 13 or 11 idk brah
        // encoderDrive(0.4, -6, -6, 10);
        //encoderDrive(0.4,1,1,8);
        //encoderDrive(0.4, -11, 11, 10);
        //encoderDrive(0.4, 25, 25, 8);
        //encoderDrive(0.2, -0, -5.5, 10);
        //encoderDrive(0.3,5,5,5);

        //This code is outside the switch statement because it happens no matter what
        encoderDrive(0.4, 0, 6, 3);
        encoderDrive(0.2, 17, 17, 4);

        switch (shippingElementPosition) {
            case LEFT:
                //Bottom Level
                robot.intake.setPower(-0.8);
                sleep(1000);
                robot.intake.setPower(0);
                break;

            case MIDDLE:
                //Middle Level
                robot.arm1.setTargetPosition(-63);
                robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm2.setTargetPosition(-63);
                robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm1.setPower(0.2);
                robot.arm2.setPower(0.2);
                encoderDrive(0.2,6,6,2);
                robot.intake.setPower(-0.55);
                sleep(1000);
                robot.intake.setPower(0);
                encoderDrive(0.2,-8,-8,3);
                robot.arm1.setPower(0);
                robot.arm2.setPower(0);
                break;

            case RIGHT:
                //Top Level
                robot.arm1.setTargetPosition(-195);
                robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm2.setTargetPosition(-195);
                robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm1.setPower(0.2);
                robot.arm2.setPower(0.2);
                encoderDrive(0.2,6,6,2);
                robot.intake.setPower(-0.8);
                sleep(1000);
                robot.intake.setPower(0);
                encoderDrive(0.2,-8,-8,3);
                robot.arm1.setPower(0);
                robot.arm2.setPower(0);
                break;
        }

        //This code is outside the switch statement again because once again it is done no matter what
        encoderDrive(0.2,-5, -5, 5);

        //CAROUSEL CODE GOES HERE


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
