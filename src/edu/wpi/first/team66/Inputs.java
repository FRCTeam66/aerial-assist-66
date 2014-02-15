package edu.wpi.first.team66;

import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Inputs implements IOParams {
    
    private final Joystick driveStickL;
    private final Joystick driveStickR;
    private final Joystick armStick;
    
    private final DigitalInput autoModeSelectorBit0;
    private final DigitalInput autoModeSelectorBit1;
    private final DigitalInput autoModeSelectorBit2;
    
    public Inputs(
            Joystick driveStickL,
            Joystick driveStickR,
            Joystick armStick,
            DigitalInput autoModeSelectorBit0,
            DigitalInput autoModeSelectorBit1,
            DigitalInput autoModeSelectorBit2)
    {
        this.driveStickL = driveStickL;
        this.driveStickR = driveStickR;
        this.armStick = armStick;
        
        this.autoModeSelectorBit0 = autoModeSelectorBit0;
        this.autoModeSelectorBit1 = autoModeSelectorBit1;
        this.autoModeSelectorBit2 = autoModeSelectorBit2;
    }
    
    public int getAutonomousMode()
    {
        return ((autoModeSelectorBit2.get()) ? 4 : 0) |
               ((autoModeSelectorBit1.get()) ? 2 : 0) |
               ((autoModeSelectorBit0.get()) ? 1 : 0);
    }
    
    public double getRightDriveSpeed()
    {
        double v = driveStickR.getY();
        return v > JOYSTICK_DEADBAND ? v : 0.0;
    }
    
    public double getLeftDriveSpeed()
    {
        double v = driveStickL.getY();
        return v > JOYSTICK_DEADBAND ? v : 0.0;
    }
    
    public int getShiftMode()
    {
        if (driveStickL.getRawButton(DRIVER_SHIFTER_TRIGGER))
        {
            return SHIFT_MODE_LOW;
        }
        else if (driveStickR.getRawButton(DRIVER_SHIFTER_TRIGGER))
        {
            return SHIFT_MODE_HIGH;
        }
        else
        {
            return SHIFT_MODE_AUTO;
        }
    }
    
    public boolean getShootButton()
    {
        return armStick.getRawButton (JOYSTICK_ARM_SHOOT_BUTTON);
    }
    
    public boolean getExtendButton()
    {
        return armStick.getRawButton(JOYSTICK_ARM_EXTEND_BUTTON);
    }
    
    public boolean getRetractButton()
    {
        return armStick.getRawButton(JOYSTIC_ARM_RETRACT_BUTTON);
    }
    
    public boolean getEjectButton()
    {
        return armStick.getRawButton (JOYSTICK_ARM_EJECT_BUTTON);
    }
}
