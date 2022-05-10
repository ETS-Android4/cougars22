package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is a super basic example of a tank drive that we used to drive a random Quad car that Chris made
 */

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Quad")
public class Quad extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final double DRIVE_POWER = 0.7;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Seb", "Is Cool");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        waitForStart();

        while (opModeIsActive())
        {
            leftDrive.setPower(gamepad1.left_stick_y * DRIVE_POWER);
            rightDrive.setPower(gamepad1.right_stick_y * DRIVE_POWER);
        }
    }
}
