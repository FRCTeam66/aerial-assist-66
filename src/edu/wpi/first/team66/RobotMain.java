// * Class T66Robot
// *
// *  TTTTT      6          6
// *    T       6          6
// *    T      6          6
// *    T     6          6
// *    T    6 6666     6 6666
// *    T    6     6    6     6
// *    T     66666      66666
// *
// * Edit Date: February 13, 2014
// *

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// IO list
// -------
// PWM1 Left drive motor to Victor 884
// PWM2 Right drive motor to Victor 884
// PWM3 Shooter motor to Victor 884
// PWM4 Ball collector motor to Victor 884
// PWM5 
// PWM6 
// PWM7
// PWM8
// PWM9
// PWM10
//
// Relays
// R1 Camera lights to Spike Relay
// R2 
// R3 
// R4 
// R5 
// R6
// R7
// R8 Air Compressor
//
// Solenoids
// S1 Shift to High gear
// S2 Shift to Low  gear
// S3 Arm extend
// S4 Arm retract 
// S5 
// S6
// S7
// S8
//
// Digital IO
// DIO1 Left     wheel encoder bit 0 \
// DIO2 Left     wheel encoder bit 1 /
// DIO3 Right    wheel encoder bit 0 \
// DIO4 Right    wheel encoder bit 1 /
// DIO5 Shooter arm    encoder bit 0 \
// DIO6 Shooter arm    encoder bit 1 /
// DIO7 Arm extended  limit switch \
// DIO8 Arm retracted limit switch /
// DIO9 Shooter cocked limit switch
// DIO10 Shooter shot limit switch
// DIO11 Nasson pressure switch for air compresssor
// DIO12 Autonomous bit 0
// DIO13 Autonomous bit 1
// DIO14 Autonomous bit 2
//
// Analog IO
// AIO1 Gyro
// AIO2 Ball loaded distance sensor
// AIO3 Arm Position POT
// AIO4 
// AIO5 
// AIO6 
// AIO7 Diagnostics Pot 
// AIO8 Battery voltage
//
// I2C IO - Unused
//
// CAN Bus
// Board ID 1 
// Board ID 2 
// Board ID 3 
//
//
// Operator interface
// USB port1 Left driver joystick
// USB port2 Right driver joystick
// USB port3 Shooter joystick
// USB port4 Stop button
//
// Joystick1 Driver
// X-axis Unused in tank drive
// Y-axis Left motor forward of backward
// Z-axis Unused
// Button 1 or Trigger to shift the robot from high to low gear
// Button 2
// Button 3 
// Button 4
// Button 5
// Button 6
// Button 7
// Button 8
// Button 9
// Button 10 Pause Clear
// Button 11 Pause
//
// Joystick 2 Unused
// X-axis Unused in tank drive
// Y-axis Right motor forward of backward
// Z-axis Unused
// Button 1 or Trigger to shift the robot from high to low gear
// Button 2 
// Button 3 
// Button 4 
// Button 5 
// Button 6 
// Button 7 
// Button 8 
// Button 9 
// Button 10 Pause Clear
// Button 11 Pause
//
// Joystick 3 Shooter Control
// X-axis Unused
// Y-axis Unused
// Button 1 or Trigger  Shoot
// Button 2 Arm Extend
// Button 3 Arm Retract
// Button 4 Ball extract
// Button 5 
// Button 6 Manual Lights Toggle
// Button 7 Eject Ball From Shooter
// Button 8 Retract Arm
// Button 9 Extend Arm
// Button 10 Pause Clear
// Button 11 Pause
//

package edu.wpi.first.team66;

import edu.wpi.first.team66.autonomous.DoNothingState;
import edu.wpi.first.team66.autonomous.StateMachine;
import edu.wpi.first.team66.autonomous.mode1.Mode1InitState;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;

import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.team66.params.StateParams;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends IterativeRobot implements IOParams, StateParams {

    private DiagnosticDisplay diagnostics = null;
    
    private DeltaTimer periodicTimer = null;
    
    private TankDrive tankDrive = null;
    
    private Loader loader = null;
    
    private Shooter shooter = null;
    
    private StateMachine autoStateMachine = null;
    
    
    Hand hand;                                      // Required for the trigger function
    
    // Declare variables for the two joysticks being used
    Joystick driveStickL;                           // Joystick Left  (tank drive)
    Joystick driveStickR;                           // Joystick Right (tank drive)
    Joystick armStick;                              // Joystick for the arm

    // Gyro objects and constants. The Gyro object takes care of everything for us.
    Gyro gyro;                                      // Gyro for autonomous steering.

    // Time of autonomous or telop modes
    Timer timer = new Timer();

    private int autonomousMode = 0;     // Mode is a number between 0 and 7.
    double autonomousStopTime = 0.0;    // Used as safety time in autonomous.

    DigitalInput autonomousBit0;
    DigitalInput autonomousBit1;
    DigitalInput autonomousBit2;

    AnalogChannel armPosition;

    final double DRIVE_STOP = 0.0;                  // Motor stop

    boolean diagOff = false;                        // Flag to display diagnostics-off screen once.

    boolean cameraLightsButtonPressed;              // ?
    boolean cameraLightsButtonWasPressed;           // Status of camera lightbutton from previous message.

    Compressor airCompressor;                       // Define the air compressor object.
    
    final boolean lowShifterHigh    = false; // To shift high gear
    final boolean higherShifterhigh = true;

    final boolean lowShifterLow     = true; // To shift to low gear.
    final boolean highShifterLow    = false;

    double leftWheelPower = 0.0d;                   // Used in diagnostics.
    double rightWheelPower = 0.0d;                  // Used in diagnostics.
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        periodicTimer = new DeltaTimer();
        
        // Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
        driveStickL = new Joystick(1);
        driveStickR = new Joystick(2);
        armStick    = new Joystick(3);

        // Setup tank drive
        Encoder leftMotorEncoder = new Encoder(
                DIO_SLOT,
                LEFT_WHEEL_ENCODER_BIT_0,
                DIO_SLOT,
                LEFT_WHEEL_ENCODER_BIT_1);
        
        
        Encoder rightMotorEncoder = new Encoder(
                DIO_SLOT,
                RIGHT_WHEEL_ENCODER_BIT_0,
                DIO_SLOT,
                RIGHT_WHEEL_ENCODER_BIT_1);
        
        tankDrive = new TankDrive(
                new Victor(DIO_SLOT, LEFT_MOTOR_PWM_CHANNEL),
                new Victor(DIO_SLOT, RIGHT_MOTOR_PWM_CHANNEL),
                new Solenoid(HIGH_SHIFTER_SOLENOID_SHIFTER_SIO_CHANNEL),
                new Solenoid(LOW_SHIFTER_SOLENOID_SIO_CHANNEL),
                leftMotorEncoder,
                rightMotorEncoder);
        
        // Setup loader system
        
        loader = new Loader(
                new Solenoid(ARM_RETRACT_SOLENOID),
                new Solenoid(ARM_EXTEND_SOLENOID),
                new Victor(DIO_SLOT, BALL_ROLLER_MOTOR_PWM_CHANNEL),
                new DigitalInput(ARM_EXTEND_LIMIT_SWITCH_DI_CHANNEL),
                new DigitalInput(ARM_RETRACT_LIMIT_SWITCH_DI_CHANNEL));
        
        // Setup shooter system
        
        SpeedController shooterMotor = new Victor(DIO_SLOT, SHOOTER_MOTOR_PWM_CHANNEL);

        Encoder shooterMotorEncoder  = new Encoder(DIO_SLOT, SHOOTER_ARM_ENCODER_BIT_0, 
                                           DIO_SLOT, SHOOTER_ARM_ENCODER_BIT_1);
        shooterMotorEncoder.setDistancePerPulse(ENCODER_SHOOTER_DISTANCE_PER_PULSE);
        shooterMotorEncoder.start();
        
        DigitalInput shooterCockedLimitSwitch = new DigitalInput(SHOOTER_COCKED_LIMIT_SWITCH_DI_CHANNEL);
        DigitalInput shooterShotLimitSwitch   = new DigitalInput(SHOOTER_SHOT_LIMIT_SWITCH_DI_CHANNEL);
        
        AnalogChannel ballLoadedSensor = new AnalogChannel(BALL_LOADED_SENSOR_AI_CHANNEL);
        
        shooter = new Shooter(
                shooterMotor,
                shooterMotorEncoder,
                ballLoadedSensor,
                shooterCockedLimitSwitch,
                shooterShotLimitSwitch,
                loader);

        gyro = new Gyro(GYRO_ANALOG_AI_CHANNEL);       // Gyro object
        gyro.reset();
        
        // Autonomous stuff...
        autonomousBit0 = new DigitalInput(AUTONOMOUS_BIT_0_DI_CHANNEL);
        autonomousBit1 = new DigitalInput(AUTONOMOUS_BIT_1_DI_CHANNEL);
        autonomousBit2 = new DigitalInput(AUTONOMOUS_BIT_2_DI_CHANNEL);

        autonomousMode = ((autonomousBit2.get()) ? 4 : 0) |
                         ((autonomousBit1.get()) ? 2 : 0) |
                         ((autonomousBit0.get()) ? 1 : 0);
        System.out.println("Autonomous mode: " + StringUtils.format(autonomousMode, 2));
        
        autoStateMachine = new StateMachine();
        
        // Switches behave just like a limit switch on a Digital IO.

        airCompressor = new Compressor(NASSON_PRESSURE_SWITCH_DI_CHANNEL, AIR_COMPRESSOR_RELAY_RIO_CHANNEL);
        airCompressor.start();                      // Start the air compressor.

        cameraLightsButtonPressed = JOYSTICK_BUTTON_NOT_PRESSED;
        cameraLightsButtonWasPressed = false;

        timer = new Timer ();                       // Instantiate the match timer shootDiag6
        timer.reset();

        diagnostics = new DiagnosticDisplay(
                DriverStation.getInstance(),
                DriverStationLCD.getInstance(),
                new AnalogChannel(DIAGNOSTIC_SELECTOR_AI_CHANNEL),
                driveStickL,
                driveStickR,
                armStick,
                tankDrive,
                shooter,
                loader);
    }

    public void autonomousInit () {
        periodicTimer.reset();
        timer.start();
        airCompressor.start();
        
        switch (autonomousMode) {
            case AMODE_DO_NOTHING:
                autoStateMachine.setState(new DoNothingState());
                break;
            case AMODE_DRIVE_FORWARD:
                autoStateMachine.setState(new Mode1InitState());
                break;
            default:
                autoStateMachine.setState(new DoNothingState());
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        diagnostics.displayDiagnostics();

        autoStateMachine.update(tankDrive, loader, shooter);
        
        double deltaTime = periodicTimer.getDeltaTime();
        shooter.update(deltaTime);
        tankDrive.update(deltaTime);

    }


    public void teleopInit () {
        periodicTimer.reset();
        airCompressor.start();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        double yl, yr;
        int i;
        boolean b;

        // Call the diagnostic display functions.
        // This function will decide what to do.
        diagnostics.displayDiagnostics();

        // +T66
        // Driver joysticks.
        // If either trigger is pulled then shift to low speed.
        i = (driveStickL.getRawButton(DRIVER_SHIFTER_TRIGGER) ? 2 : 0) +
            (driveStickR.getRawButton(DRIVER_SHIFTER_TRIGGER) ? 1 : 0);
        if (i == 0) {                               // Neither trigger button is pressed,
            tankDrive.shiftHighGear();
        } else {                                    // A trigger button is pressed,
            tankDrive.shiftLowGear();
        }

        // Set the motors speed.
        yl = driveStickL.getY();
        yl = (Math.abs(yl) < JOYSTICK_DEADBAND) ? 0.0 :yl; // If within in deadband make it 0.

        yr = driveStickR.getY();
        yr = (Math.abs(yr) < JOYSTICK_DEADBAND) ? 0.0 :yr;
        
        tankDrive.setTargetSpeed(yl, yr);

        // If the Joystick Arm Extend button is pressed, then extend the arm...
        // just the arm extend Solenoid, and set the ball roller motor to Load.
        if (armStick.getRawButton(JOYSTICK_ARM_EXTEND_BUTTON) == JOYSTICK_BUTTON_PRESSED) {
            loader.extend();
        } // if (armStick.getRawButton(kJoystickArmExtendButton) == kJoystickButtonPressed)

        // If the Joystick Arm Retract button is pressed, then retract the arm...
        //   just turn on the motor to retract, and turn off the ball roller motor
        //   (otherwise it might eject the ball).
        // This is using an "else if" to handle the situation if both buttons
        //   are pressed at once.
        else if (armStick.getRawButton (JOYSTIC_ARM_RETRACT_BUTTON) == JOYSTICK_BUTTON_PRESSED) {
            loader.retract();
        } // else if (armStick.getRaw (kJoyStickArmRetractButton) == kJoystickButtonPressed)

        // If the Joystick ball eject button is pressed
        //   then turn on the motor to eject the ball and retract the arm.
        if (armStick.getRawButton (JOYSTICK_ARM_EJECT_BUTTON) == JOYSTICK_BUTTON_PRESSED) {
            loader.eject();
        }
        else if (loader.isEjecting()) {
            loader.stopEjecting();
        }

        // Is the shooting trigger button pressed?
        if (armStick.getRawButton (JOYSTICK_ARM_SHOOT_BUTTON) == JOYSTICK_BUTTON_PRESSED) {
            shooter.shoot();
        }

        double deltaTime = periodicTimer.getDeltaTime();
        shooter.update(deltaTime);
        tankDrive.update(deltaTime);
    }


    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
}
