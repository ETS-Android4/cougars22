package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Warehouse - Red")
public class AutonWarehouseRed extends BaseAuton
{
    @Override
    public void runOpMode()
    {
        super.runOpMode();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();

        encoderDrive(0.2, 22, 22, 7);
        encoderDrive(0.2 , -11, 11, 5);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 3)
        {
            double power = 0.4;
            robot.leftFront.setPower(power);
            robot.leftBack.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightBack.setPower(power);
        }
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }
}
