package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TapeMeasureSubsystem;

import java.util.function.DoubleSupplier;

public class ControllerDrive extends CommandBase
{
    private final DriveSubsystem drive;
    private final TapeMeasureSubsystem tapeMeasure;
    private final GamepadEx gamepad;

    private boolean tapeMeasureMode;

    public ControllerDrive(DriveSubsystem drive, TapeMeasureSubsystem tapeMeasure, GamepadEx gamepad)
    {
        this.drive = drive;
        this.tapeMeasure = tapeMeasure;
        this.gamepad = gamepad;
        addRequirements(drive, tapeMeasure);
    }

    @Override
    public void execute()
    {
        drive.drive(gamepad.getLeftY(), gamepad.getRightX());
        tapeMeasure.runExtend(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }
}
