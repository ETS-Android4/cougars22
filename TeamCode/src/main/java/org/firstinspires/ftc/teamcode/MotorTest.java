package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@TeleOp(name="Motor Test")
public class MotorTest extends LinearOpMode
{
    private double power = 0.5;
    private double lastPowerChangeTime;
    private int motorIndex = 0;
    private boolean leftDownLastFrame = false;
    private boolean rightDownLastFrame = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        List<String> motorNames = new ArrayList<>();
        for (DcMotor motor : motors)
        {
            motorNames.add(hardwareMap.getNamesOf(motor).iterator().next());
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            // Adjust Power
            if (runtime.time() > lastPowerChangeTime + 0.5)
            {
                if (gamepad1.dpad_down)
                {
                    power = Math.max(0, power - 0.1);
                    lastPowerChangeTime = runtime.time();
                }
                else if (gamepad1.dpad_up)
                {
                    power = Math.min(1, power + 0.1);
                    lastPowerChangeTime = runtime.time();
                }
            }

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
            currMotor.setPower(gamepad1.left_stick_y * power);

            telemetry.addData("Motor", motorNames.get(motorIndex));
            telemetry.addData("Motor Position", currMotor.getCurrentPosition());
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
