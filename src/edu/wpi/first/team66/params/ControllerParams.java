package edu.wpi.first.team66.params;

import edu.wpi.first.wpilibj.Joystick;

public interface ControllerParams {
    /**
     * Driver Controller
     */
    public static final int DRIVER_CONTROLLER_ID  = 1;
    public static final int LEFT_DRIVE_AXIS_ID    = 4; // Left vertical axis
    public static final int RIGHT_DRIVE_AXIS_ID   = 2; // Right vertical axis
    public static final int HIGH_GEAR_BUTTON_1_ID = 5; // L1 button (trigger)
    public static final int HIGH_GEAR_BUTTON_2_ID = 6; // R1 button (trigger)
    
    /**
     * Shooter/Loader Controller
     */
    public static final int SHOOTER_CONTROLLER_ID                       = 2;
    public static final int EXTEND_LOADER_BUTTON_ID                     = 4; // Triangle button
    public static final int RETRACT_LOADER_BUTTON_ID                    = 2; // X button
    public static final int ROLL_IN_LOADER_BUTTON_ID                    = 6; // R1 button (trigger)
    public static final int ROLL_OUT_LOADER_BUTTON_ID                   = 5; // L1 button (trigger)
    
    public static final int STANDARD_SHOT_BUTTON_1_ID                   = 7; // R2 button (trigger)
    public static final int STANDARD_SHOT_BUTTON_2_ID                   = 8; // L2 button (trigger)
    public static final int TRUSS_TOSS_BUTTON_1_ID                      = 11; // R3 button (thumbstick button)
    public static final int TRUSS_TOSS_BUTTON_2_ID                      = 12; // L3 button (thumbstick button)
    public static final int MANUAL_SHOOTER_MOVE_BUTTON_ID               = 10; // Start button
    public static final int MANUAL_SHOOTER_MOVE_AXIS_ID                 = 4; // Left vertical axis
    public static final int OVERRIDE_LOADER_EXTENDED_CHECK_BUTTON_ID    = 9; // Select button
    public static final int RESET_SHOOTER_HOME_BUTTON_ID                = 13;// Home button

}
