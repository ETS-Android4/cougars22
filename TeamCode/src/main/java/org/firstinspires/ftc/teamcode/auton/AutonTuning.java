package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PID Tuning")
public class AutonTuning extends BaseAuton
{
    @Override
    public void runOpMode()
    {
        super.runOpMode();

        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        double maxI = 1;
        double a = 0;
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
                        maxI += delta;
                        break;
                    case 4:
                        a += delta;
                        break;
                    case 5:
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
                        maxI -= delta;
                        break;
                    case 4:
                        a -= delta;
                        break;
                    case 5:
                        delta /= 10;
                        break;
                }
            }
            else if (upPressed && !upHeld)
            {
                selectedItem = (selectedItem + 5) % 6;
            }
            else if (downPressed && !downHeld)
            {
                selectedItem = (selectedItem + 1) % 6;
            }
            else if (aPressed && !aHeld)
            {
                gyroTurn(90, Kp, Ki, Kd, maxI, a);
                sleep(1000);
                gyroTurn(0, Kp, Ki, Kd, maxI, a);
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
            telemetry.addData("Selected", selectedItem == 0 ? "Kp" : selectedItem == 1 ? "Ki" : selectedItem == 2 ? "Kd" : selectedItem == 3 ? "maxI" : selectedItem == 4 ? "a" : selectedItem == 5 ? "delta" : "N/A");
            telemetry.update();
        }
    }
}
