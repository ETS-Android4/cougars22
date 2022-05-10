package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.LameDriveBot;


/**
 * This is a simplified version of the basic drive op mode that has a low power and only runs the drive base
 * We used it for allowing small children (elementary school) to drive the robot without being worried they'd kill someone
 * The code is a subset of BasicDrive, so see that class for more details on how everything works
 */


@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Training Wheels")
//@Disabled
public class TrainingWheels extends LinearOpMode {

    private final LameDriveBot robot = new LameDriveBot();

    private final double DRIVE_POWER = 0.15;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Seb", "Is Cool");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Setup a variable for each drive wheel
            double leftPower;
            double rightPower;

            // Calculate drive power
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            leftPower *= DRIVE_POWER;
            rightPower *= DRIVE_POWER;

            // Send calculated power to wheels
            robot.leftFront.setPower(leftPower);
            robot.leftBack.setPower(leftPower);
            robot.rightFront.setPower(rightPower);
            robot.rightBack.setPower(rightPower);
        }
    }
}
