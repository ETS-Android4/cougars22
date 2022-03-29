package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase
{
    private final DifferentialDrive drive;

    private double maxSpeed = 1;

    public static final double WHEEL_DIAMETER_INCHES = 3.54330709;

    public DriveSubsystem(final HardwareMap hwMap)
    {
        MotorGroup left = new MotorGroup(new Motor(hwMap, "leftFront"), new Motor(hwMap, "leftBack"));
        MotorGroup right = new MotorGroup(new Motor(hwMap, "rightFront"), new Motor(hwMap, "rightBack"));

        drive = new DifferentialDrive(left, right);
    }

    public void drive(double forward, double turn)
    {
        drive.arcadeDrive(forward, turn);
    }

    public void setMaxSpeed(double newMaxSpeed)
    {
        drive.setMaxSpeed(newMaxSpeed);
        maxSpeed = newMaxSpeed;
    }
    public double getMaxSpeed()
    {
        return maxSpeed;
    }
}
