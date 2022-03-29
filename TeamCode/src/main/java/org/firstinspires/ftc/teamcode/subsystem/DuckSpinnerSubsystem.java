package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckSpinnerSubsystem extends SubsystemBase
{
    private final MotorEx duckSpinner;
    private double duckSpinnerSpeed = 1;

    public DuckSpinnerSubsystem(HardwareMap hwMap)
    {
        duckSpinner = new MotorEx(hwMap, "duckSpinner");
    }

    public void setDuckSpinnerSpeed(double speed)
    {
        duckSpinnerSpeed = speed;
    }

    public void runDuckSpinner(double speed)
    {
        duckSpinner.set(duckSpinnerSpeed * speed);
    }
}
