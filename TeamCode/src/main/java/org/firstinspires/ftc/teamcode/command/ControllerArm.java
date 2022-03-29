package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DuckSpinnerSubsystem;

import java.util.function.DoubleSupplier;

public class ControllerArm extends CommandBase
{
    private final ArmSubsystem arm;
    private final DuckSpinnerSubsystem duckSpinner;
    private final GamepadEx gamepad;

    public ControllerArm(ArmSubsystem arm, DuckSpinnerSubsystem duckSpinner, GamepadEx gamepad)
    {
        this.arm = arm;
        this.duckSpinner = duckSpinner;
        this.gamepad = gamepad;
        addRequirements(arm, duckSpinner);
    }

    @Override
    public void execute()
    {
        arm.runArm(gamepad.getLeftY());
        arm.runIntake(gamepad.getRightY());
        duckSpinner.runDuckSpinner(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }
}
