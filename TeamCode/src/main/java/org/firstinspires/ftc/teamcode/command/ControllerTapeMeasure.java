package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.TapeMeasureSubsystem;

import java.util.function.DoubleSupplier;

public class ControllerTapeMeasure extends CommandBase
{
    private final TapeMeasureSubsystem tapeMeasure;
    private final GamepadEx gamepad;

    public ControllerTapeMeasure(TapeMeasureSubsystem tapeMeasure, GamepadEx gamepad)
    {
        this.tapeMeasure = tapeMeasure;
        this.gamepad = gamepad;
        addRequirements(tapeMeasure);
    }

    @Override
    public void execute()
    {
        tapeMeasure.runRotate(gamepad.getRightX());
        tapeMeasure.runUpDown(gamepad.getLeftY());
        tapeMeasure.runExtend(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }
}
