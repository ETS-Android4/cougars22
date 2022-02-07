package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PID Tuning")
public class AutonTuning extends BaseAuton
{
    @Override
    public void runOpMode()
    {
        super.runOpMode();

        double Kp = 0.01;
        double Ki = 0;
        double Kd = 0;
        double delta = 0.01;
        int selectedItem = 0;
        boolean upHeld = false;
        boolean downHeld = false;
        boolean leftHeld = false;
        boolean rightHeld = false;
        boolean aHeld = false;

        waitForStart();

        while (opModeIsActive())
        {
            boolean upPressed = gamepad1.dpad_up;
            boolean downPressed = gamepad1.dpad_down;
            boolean leftPressed = gamepad1.dpad_left;
            boolean rightPressed = gamepad1.dpad_right;
            boolean aPressed = gamepad1.a;

            if (rightPressed && !rightHeld)
            {
                switch(selectedItem)
                {
                    case 0:
                        Kp += delta;
                        break;
                    case 1:
                        Ki += delta;
                        break;
                    case 2:
                        Kd += delta;
                        break;
                    case 3:
                        delta *= 10;
                        break;
                }
            }
            else if (leftPressed && !leftHeld)
            {
                switch(selectedItem)
                {
                    case 0:
                        Kp -= delta;
                        break;
                    case 1:
                        Ki -= delta;
                        break;
                    case 2:
                        Kd -= delta;
                        break;
                    case 3:
                        delta /= 10;
                        break;
                }
            }
            else if (upPressed && !upHeld)
            {
                selectedItem = (selectedItem + 4) % 5;
            }
            else if (downPressed && !downHeld)
            {
                selectedItem = (selectedItem + 1) % 5;
            }
            else if (aPressed && !aHeld)
            {
                gyroTurn(1, 90, Kp, Ki, Kd);
                sleep(1000);
                gyroTurn(1, 0, Kp, Ki, Kd);
            }

            upHeld = upPressed;
            downHeld = downPressed;
            leftHeld = leftPressed;
            rightHeld = rightPressed;
            aHeld = aPressed;

            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.addData("delta", delta);
            telemetry.addData("Selected", selectedItem == 0 ? "Kp" : selectedItem == 1 ? "Ki" : selectedItem == 2 ? "Kd" : "delta");
            telemetry.update();
        }
    }
}
