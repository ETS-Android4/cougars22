package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase
{
    private final MotorGroup arm;
    private final Motor intake;

    private double armSpeed = 1;
    private double intakeSpeed = 1;

    public ArmSubsystem(final HardwareMap hwMap)
    {
        Motor arm1 = new Motor(hwMap, "arm1");
        Motor arm2 = new Motor(hwMap, "arm2");
        arm2.setInverted(true);
        arm = new MotorGroup(arm1, arm2);
        intake = new Motor(hwMap, "intake");
    }

    public void setIntakeSpeed(double newIntakeSpeed)
    {
        intakeSpeed = newIntakeSpeed;
    }

    public void setArmSpeed(double newArmSpeed)
    {
        armSpeed = newArmSpeed;
    }

    public void runIntake(double speed)
    {
        intake.set(intakeSpeed * speed);
    }

    public void runArm(double speed)
    {
        arm.set(armSpeed * speed);
    }


}
