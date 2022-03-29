package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TapeMeasureSubsystem extends SubsystemBase
{
    private final CRServo rotate;
    private final CRServo upDown;
    private final CRServo extend;
    private double rotateSpeed = 1;
    private double upDownSpeed = 1;
    private double extendSpeed = 1;

    public TapeMeasureSubsystem(HardwareMap hwMap)
    {
        rotate = new CRServo(hwMap, "tapeMeasureRotate");
        rotate.setInverted(true);
        upDown = new CRServo(hwMap, "tapeMeasureUpDown");
        upDown.setInverted(true);
        extend = new CRServo(hwMap, "tapeMeasureExtend");
    }

    public void setRotateSpeed(double rotateSpeed)
    {
        this.rotateSpeed = rotateSpeed;
    }

    public void setUpDownSpeed(double upDownSpeed)
    {
        this.upDownSpeed = upDownSpeed;
    }

    public void setExtendSpeed(double extendSpeed)
    {
        this.extendSpeed = extendSpeed;
    }

    public void runRotate(double speed)
    {
        rotate.set(rotateSpeed * speed);
    }

    public void runUpDown(double speed)
    {
        upDown.set(upDownSpeed * speed);
    }

    public void runExtend(double speed)
    {
        extend.set(extendSpeed * speed);
    }
}
