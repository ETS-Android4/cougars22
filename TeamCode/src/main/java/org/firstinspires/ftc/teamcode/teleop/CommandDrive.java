package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.ControllerArm;
import org.firstinspires.ftc.teamcode.command.ControllerDrive;
import org.firstinspires.ftc.teamcode.command.ControllerTapeMeasure;
import org.firstinspires.ftc.teamcode.command.OnceCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DuckSpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TapeMeasureSubsystem;

@TeleOp()
public class CommandDrive extends CommandOpMode
{
    private final double FAST_DRIVE_POWER = 0.8;
    private final double SLOW_DRIVE_POWER = 0.5;
    private final double ARM_POWER = 0.4;
    private final double INTAKE_POWER = 0.8;
    private final double DUCK_SPINNER_POWER = 0.5;
    private final double TAPE_MEASURE_ROTATE_POWER = 0.7;
    private final double TAPE_MEASURE_UP_DOWN_POWER = 0.9;
    private final double TAPE_MEASURE_EXTEND_POWER = 1;

    @Override
    public void initialize()
    {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        drive.setMaxSpeed(FAST_DRIVE_POWER);

        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        arm.setArmSpeed(ARM_POWER);
        arm.setIntakeSpeed(INTAKE_POWER);

        TapeMeasureSubsystem tapeMeasure = new TapeMeasureSubsystem(hardwareMap);
        tapeMeasure.setRotateSpeed(TAPE_MEASURE_ROTATE_POWER);
        tapeMeasure.setUpDownSpeed(TAPE_MEASURE_UP_DOWN_POWER);
        tapeMeasure.setExtendSpeed(TAPE_MEASURE_EXTEND_POWER);

        DuckSpinnerSubsystem duckSpinner = new DuckSpinnerSubsystem(hardwareMap);
        duckSpinner.setDuckSpinnerSpeed(DUCK_SPINNER_POWER);

        Servo armHold = hardwareMap.get(Servo.class, "armHold");

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx armGamepad = new GamepadEx(gamepad2);

        ControllerDrive driveCommand = new ControllerDrive(drive, tapeMeasure, driverGamepad);
        ControllerTapeMeasure tapeMeasureCommand = new ControllerTapeMeasure(tapeMeasure, driverGamepad);
        ControllerArm armCommand = new ControllerArm(arm, duckSpinner, armGamepad);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> drive.setMaxSpeed(SLOW_DRIVE_POWER)));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> drive.setMaxSpeed(FAST_DRIVE_POWER)));
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(tapeMeasureCommand);
        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(driveCommand);

        armGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> armHold.setPosition(1)));
        armGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> armHold.setPosition(0)));

        schedule(driveCommand);
        schedule(armCommand);

        schedule(new OnceCommand(() -> armHold.setPosition(0))); // Pretty sure this will run once on run and never again

        schedule(new RunCommand(() -> {
            telemetry.addData("Power", drive.getMaxSpeed());
            telemetry.addData("Mode", tapeMeasureCommand.isScheduled() ? "Tape Measure" : "Drive");
            telemetry.update();
        }));
    }
}
