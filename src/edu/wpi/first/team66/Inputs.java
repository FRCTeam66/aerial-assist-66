package edu.wpi.first.team66;

import edu.wpi.first.team66.params.ControllerParams;
import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Inputs implements IOParams, ControllerParams {
    
    private final Joystick driverController;
    private final Joystick shooterController;
    
    private final DigitalInput autoModeSelectorBit0;
    private final DigitalInput autoModeSelectorBit1;
    private final DigitalInput autoModeSelectorBit2;
    
    public Inputs(
            Joystick driverController,
            Joystick shooterController,
            DigitalInput autoModeSelectorBit0,
            DigitalInput autoModeSelectorBit1,
            DigitalInput autoModeSelectorBit2)
    {
        this.driverController = driverController;
        this.shooterController = shooterController;
        
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
        double v = driverController.getAxis(RIGHT_DRIVE_AXIS_ID);
        return Math.abs(v) > JOYSTICK_DEADBAND ? v : 0.0;
    }
    
    public double getLeftDriveSpeed()
    {
        double v = driverController.getAxis(LEFT_DRIVE_AXIS_ID);
        return Math.abs(v) > JOYSTICK_DEADBAND ? v : 0.0;
    }
    
    public int getShiftMode()
    {
        // if either of these two buttons are held
        if (driverController.getRawButton(HIGH_GEAR_BUTTON_1_ID) || 
            driverController.getRawButton(HIGH_GEAR_BUTTON_2_ID))
        {
            return SHIFT_MODE_HIGH;
        }
        else
        {
            return SHIFT_MODE_LOW;
        }
    }
    
    public boolean getShootButton()
    {
        return shooterController.getRawButton(STANDARD_SHOT_BUTTON_1_ID) && 
               shooterController.getRawButton(STANDARD_SHOT_BUTTON_2_ID);
    }
    
    public boolean getTrussTossButton()
    {
        return shooterController.getRawButton(TRUSS_TOSS_BUTTON_1_ID) && 
               shooterController.getRawButton(TRUSS_TOSS_BUTTON_2_ID);
    }
    
    public boolean getOverrideLoaderExtendedCheckButton()
    {
        return shooterController.getRawButton(OVERRIDE_LOADER_EXTENDED_CHECK_BUTTON_ID);
    }
     
    
    public boolean getExtendButton()
    {
        return shooterController.getRawButton(EXTEND_LOADER_BUTTON_ID);
    }
    
    public boolean getRetractButton()
    {
        return shooterController.getRawButton(RETRACT_LOADER_BUTTON_ID);
    }
    
    public boolean getRollOutButton()
    {
        return shooterController.getRawButton(ROLL_OUT_LOADER_BUTTON_ID);
    }
    
    public boolean getRollInButton()
    {
        return shooterController.getRawButton(ROLL_IN_LOADER_BUTTON_ID);
    }
}
