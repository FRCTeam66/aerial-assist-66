/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66.params;

/**
 *
 * @author pdehaan
 */
public interface IOParams {
    
    //final int kAIOSolt = 0;                       // Slot number of the Analog IO module
    static final int DIO_SLOT = 1;                         // Slot number of the Digital IO module
    static final int SIO_SLOT = 2;                         // Slot number of the Solenoid IO module

    // PWM IO Channels
    static final int LEFT_MOTOR_PWM_CHANNEL             = 1; // PWM channel numbers
    static final int RIGHT_MOTOR_PWM_CHANNEL            = 2;
    static final int SHOOTER_MOTOR_PWM_CHANNEL          = 3;
    static final int BALL_ROLLER_MOTOR_PWM_CHANNEL       = 4;

    // Relay IO Channels
    static final int CAMERA_LIGHTS_RELAY_RIO_CHANNEL     = 1;
    static final int AIR_COMPRESSOR_RELAY_RIO_CHANNEL    = 8;
    
    // Solenoid IO Channels
    
    static final int LOW_SHIFTER_SOLENOID_SIO_CHANNEL               = 1;   // Shift to low  gear
    static final int HIGH_SHIFTER_SOLENOID_SHIFTER_SIO_CHANNEL      = 2;   // Shift to high gear
    static final int ARM_RETRACT_SOLENOID                           = 3;   // Arm retrace Solenoid
    static final int ARM_EXTEND_SOLENOID                            = 4;   // Arm extend solenoid
    

    // Digital IO Channels
    static final int LEFT_WHEEL_ENCODER_BIT_0             = 1;
    static final int LEFT_WHEEL_ENCODER_BIT_1             = 2;
    static final int RIGHT_WHEEL_ENCODER_BIT_0            = 3;
    static final int RIGHT_WHEEL_ENCODER_BIT_1            = 4;
    static final int SHOOTER_ARM_ENCODER_BIT_0            = 5;
    static final int SHOOTER_ARM_ENCODER_BIT_1            = 6;
    static final int ARM_EXTEND_LIMIT_SWITCH_DI_CHANNEL    = 8;  // Digital IO channels for Arm Extend  Limit Switch.
    static final int ARM_RETRACT_LIMIT_SWITCH_DI_CHANNEL   = 7;  // Digital IO channels for Arm Retract Limit Switch.
    static final int SHOOTER_COCKED_LIMIT_SWITCH_DI_CHANNEL = 9;  // Digital IO channels for Shooter Cocked Limit Switch.
    static final int SHOOTER_SHOT_LIMIT_SWITCH_DI_CHANNEL   = 10; // Digital IO channels for Shooter Shot Limit Switch.
    static final int NASSON_PRESSURE_SWITCH_DI_CHANNEL    = 11; // Digital IO channels for the Compressor switch.
    static final int AUTONOMOUS_BIT_0_DI_CHANNEL          = 12; // Digital IO channels for autonomous Bit 0
    static final int AUTONOMOUS_BIT_1_DI_CHANNEL          = 13; // Digital IO channels for autonomous Bit 1
    static final int AUTONOMOUS_BIT_2_DI_CHANNEL          = 14; // Digital IO channels for autonomous Bit 2
    
    // Analog IO Channels
    static final int GYRO_ANALOG_AI_CHANNEL               = 1;  // Analog channel for Gyro.
    static final int BALL_LOADED_SENSOR_AI_CHANNEL        = 2;  // For Sharp infrared distance sensor to detect a ball.
    static final int ARM_POSITION_POT_AI_CHANNEL          = 3;  // Absolute arm position from an alalon potentiatometer.
    static final int DIAGNOSTIC_SELECTOR_AI_CHANNEL      = 7;  // Analog IO channels for diagnostics selector.
    static final int BATTERY_VOLTAGE_AI_CHANNEL          = 8;  // Channel allocated by FIRST for battery voltage.
    
    // Drive motor & encoders.
    static final double JOYSTICK_DEADBAND = 0.08;           // Joystick deadband for the motors.

    static final double STEERING_MOTOR_DEADBAND = 0.15;     // This amount +/- in the X-axis to drive straight.
    
    static final boolean IS_LEFT_DRIVE_REVERSED = true;
    static final boolean IS_RIGHT_DRIVE_REVERSED = false;
    
    // IMU
    static final double TODO_GYRO_DEADBAND = 1.5;               // Deadband degrees.
    
    // Ball Roller Motor Declarations and Constants.
    static final double BALL_ROLLER_MOTOR_LOAD_ON  = -1.00;   // Speed that the rollers will turn Picking-up a ball.
    static final double BALL_ROLLER_MOTOR_EJECT_ON = 1.00;    // Speed that the rollers will turn Ejecting a ball.
    static final double BALL_ROLLER_MOTOR_OFF     = 0.00;    // Speed that the rollers will turn Off.
    
    static final boolean ARM_RETRACT_SOLENOID_EXTEND = true;
    static final boolean ARM_EXTEND_SOLENOID_EXTEND = false;
        
    static final boolean ARM_RETRACT_SOLENOID_RETRACT = true;
    static final boolean ARM_EXTEND_SOLENOID_RETRACT = false;
    
    final int ARM_POSITION_MIN = 100;                // Arm in cocked position.
    final int ARM_POSITION_MAX = 600;                // Arm can exceed this angle!
    
    // Shooter stuff...
    static final double SHOOT_AND_COCK_MOTOR_ON  = 0.1;       // This will be controlled in a PID loop.
    static final double SHOOT_AND_COCK_MOTOR_OFF = 0.0;
    static final double SHOOT_AND_COCK_MOTOR_ON_SLOW_UP = 0.1;  // For moving the shooter to home position in Autonomous.
    static final double SHOOT_AND_COCK_MOTOR_ON_COCK   = -0.2; // Shooter cocking speed.
    
    static final boolean IS_SHOOTER_MOTOR_REVERSED = true;
    
    // TODO - Find the correct shooter travel distance. This is a SWAG.
    static final double SHOOTER_DISTANCE = 100.0d;         // Number of encoder clicks to move the shooter to fire.
    static final double PAUSE_AFTER_SHOOTING_TIME = 500.0d;   // Pause 500 milli-seconds (0.5 sec.) after shooting 
                                                    // to let the shooter stop moving.
    static final double ENCODER_SHOOTER_DISTANCE_PER_PULSE = 4.64d;// pulses per deg
    
    static final int SHIFT_MODE_LOW = 0;
    static final int SHIFT_MODE_HIGH = 1;
    static final int SHIFT_MODE_AUTO = 2;
}
