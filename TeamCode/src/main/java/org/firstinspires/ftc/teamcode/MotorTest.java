package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@TeleOp(name="Motor Test")
public class MotorTest extends LinearOpMode
{
    private final double POWER = 0.5;
    private int motorIndex = 0;
    private boolean leftDownLastFrame = false;
    private boolean rightDownLastFrame = false;

    @Override
    public void runOpMode()
    {
        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        List<String> motorNames = new ArrayList<String>();
        for (DcMotor motor : motors)
        {
            motorNames.add(hardwareMap.getNamesOf(motor).iterator().next());
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        waitForStart();

        while(opModeIsActive())
        {
            if (gamepad1.dpad_left)
            {
                if (!leftDownLastFrame)
                {
                    motorIndex = (motorIndex + motors.size() - 1) % motors.size();
                }
                leftDownLastFrame = true;
            }
            else
            {
                leftDownLastFrame = false;
            }

            if (gamepad1.dpad_right)
            {
                if (!rightDownLastFrame)
                {
                    motorIndex = (motorIndex + 1) % motors.size();
                }
                rightDownLastFrame = true;
            }
            else
            {
                rightDownLastFrame = false;
            }

            DcMotor currMotor = motors.get(motorIndex);
            currMotor.setPower(gamepad1.left_stick_y * POWER);

            telemetry.addData("Motor", motorNames.get(motorIndex));
            telemetry.addData("Motor Position", currMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
