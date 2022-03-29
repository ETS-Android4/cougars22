package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class OnceCommand extends CommandBase
{
    private final Runnable toRun;

    public OnceCommand(Runnable toRun, Subsystem... requirements)
    {
        this.toRun = toRun;
        addRequirements(requirements);
    }

    public OnceCommand()
    {
        this.toRun = () -> {};
    }

    @Override
    public void execute()
    {
        toRun.run();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
