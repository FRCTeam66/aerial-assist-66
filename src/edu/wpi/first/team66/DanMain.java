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

import java.lang.String;
import java.util.*;

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
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.team66.params.IOParams;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class DanMain extends IterativeRobot implements IOParams {

    private DeltaTimer periodicTimer = null;
    
    private TankDrive tankDrive = null;
    
    private Loader loader = null;
    
    
    // Driver Station and Diagnostics Display
    DriverStation ds;                               // Define the drivers station object
    DriverStationLCD dslcd;
    Hand hand;                                      // Required for the trigger function

    SpeedController shooterMotor;
    Encoder shooterMotorEncoder;
    
    //double driverStickXAxis = 0.0;                // Save this for robot diagnostics
    //double driverStickYAxis = 0.0;


    // Declare variables for the two joysticks being used
    Joystick driveStickL;                           // Joystick Left  (tank drive)
    Joystick driveStickR;                           // Joystick Right (tank drive)
    Joystick armStick;                              // Joystick for the arm

    // Gyro objects and constants. The Gyro object takes care of everything for us.
    Gyro gyro;                                      // Gyro for autonomous steering.

    boolean emergencyStop = false;

    // Diagnostic selector is a linear variable Pot
    // If not connected an analog input port returns a vlalue of < 0
    // Take the value from .get() and divide it by 107.
    //   will be 0 - 9 as the diagnostic modes. 0 is no diagnostics.
    AnalogChannel diagnosticSelector;

    // Time of autonomous or telop modes
    Timer timer = new Timer();

    int autonomousMode = 0;                         // Mode is a number between 0 and 7.
    double autonomousStopTime = 0.0;                // Used as safety time in autonomous.

    DigitalInput autonomousBit0;                    // Digital input objects
    DigitalInput autonomousBit1;
    DigitalInput autonomousBit2;

    // Ball Roller Motor Declarations and Constants.
    
    AnalogChannel ballLoadedSensor;                 // Used to detect if a ball is in the shooter mechanism.
     


    DigitalInput armExtendLimitSwitch;              // Limit Switches.
    DigitalInput armRetractLimitSwitch;

    AnalogChannel armPosition;

    
    // Shooter stuff...
    DigitalInput shooterCockedLimitSwitch;          // Limit Switches.
    DigitalInput shooterShotLimitSwitch;            // Shooter iss moved all the way.
//    final double kShootAndCockMotorOn  = 1.0;       // This will be controlled in a PID loop.



// TODO CALIBRATE if needed
    
    int shootPastState = 0;                         // Shoot Method State Machine state variables.
    int shootState = 0;
    int shootNextState = 0;
    boolean shootButtonPressed = false;

    // Shooter diagnostics.  The shoot diagnostics code will always run in the shoot function.
    String shootDiag6;                              // Text to be displayed about the shooter state machine.
    Timer shootTime;                                // Time of shooting cycle.
    Timer shootTimer;                               // Misc. timer for shooting routines.

    // Autonomous state machine controls.
    int aState = 1;                                 // State executing
    int aStateNext = 1;                             // Next state to be executed
    int aStatePast = 1;                             // Previous state executed
    String autoDiag5;                                // Used for diagnostics.
    
    //final double kDriveFast = -0.6;               // Drive speed in Autonomous mode for drive motors.
    //final double kDriveSlow = -0.3;               // Turning speed for drive motors. Speeds are negated.
    //final double kDriveTurn = 0.2;                // Steering speed + turn right, - turn left
    final double DRIVE_STOP = 0.0;                  // Motor stop

    boolean diagOff = false;                        // Flag to display diagnostics-off screen once.


    boolean cameraLightsButtonPressed;              // ?
    boolean cameraLightsButtonWasPressed;           // Status of camera lightbutton from previous message.

    Compressor airCompressor;                       // Define the air compressor object.

    Solenoid highShifterSolenoid;
    Solenoid  lowShifterSolenoid;

    final boolean lowShifterHigh    = false; // To shift high gear
    final boolean higherShifterhigh = true;

    final boolean lowShifterLow     = true; // To shift to low gear.
    final boolean highShifterLow    = false;

    Relay cameraLights;                             // Camera lights Declarations and Constants.
    Relay.Value cameraLightsOn;                     // Value to Spike Relay to turn the motor On to Extend.
    Relay.Value cameraLightsOff;                    // Value to Spike Relay to turn the motor Off.
    boolean cameraLightToggle = false;              // false = off,true = on.
    boolean cameraLightsButton =  false;            // Joy stick button status.

    double leftWheelPower = 0.0d;                   // Used in diagnostics.
    double rightWheelPower = 0.0d;                  // Used in diagnostics.
    
    // Move code variables and constants.  ----------------------
    long speed;                                     // ??Track robot speed every 26.2 ms.

// TODO dan get rid of this!
    boolean offenseDefense;                         // Which end of the robot is the front.

    double degrees;                                 // Yaw rate sensor value in degrees from gyro.getAngele ().
    double degreesStart = 0.0d;                     // Starting angle OF THE MOVE SET IN MOVEsTART.

    double distance = 0.0d;
    double ramp = 0.0d;

    int moveState     = 0;                          // Move state machine states.
    int moveNextState = 0;
    int movePastState = 0;

    double  maxPower = 1.0;                         // This value is initial power setting.

    Timer moveTimer;
    double startTime;                               // Starting time for the move.
    double tm = -1;                                 // Time of the last message from the Field Control System.
    
    double endDistanceLeft = 0;                     // Ending position in clicks.
    double endDistanceRight = 0;

    double leftPowerMax  = 1.0;                     // These values are power setting
    double rightPowerMax = 1.0;                     // values of -1.0 to +1.0.

    // Variable that change depending on Offence or Defence switch.
    // Recall that the robot has two front ends depending on O/D switch.
    final double BRAKE_FORCE = -0.1;                // 
    double brakeForce = 0.0;

    final double DEGREES_MARGIN = 1.0;              // Allow +/- N degrees variation in the direction.
    
    double compensation = 0;                        // Amount of wheel power correction is computed 
    // ----------------------                       // based upon power setting.
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        periodicTimer = new DeltaTimer();
        
        // get the driver station instance to read the digital I/O pins
        ds = DriverStation.getInstance();
        dslcd = DriverStationLCD.getInstance();

        dslcd.println(DriverStationLCD.Line.kUser1, 1, StringUtils.TODO_SPACES_21);
        dslcd.println(DriverStationLCD.Line.kUser2, 1, StringUtils.TODO_SPACES_21);
        dslcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
        dslcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
        dslcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);
        dslcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
        dslcd.updateLCD();
        
        shootDiag6 = StringUtils.TODO_SPACES_21;
        shootTime = new Timer();                    // Used for shooting diagnostics.
        shootTime.reset();
        shootTimer = new Timer();                   // Used for various things while shooting.
        shootTimer.reset();

        //enhancedIO = DriverStation.getInstance().getEnhancedIO(); // Just in case the Cypress is used.

        emergencyStop = false;

        // Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
        driveStickL = new Joystick(1);
        driveStickR = new Joystick(2);
        armStick    = new Joystick(3);

        // Setup tank drive
        Encoder leftMotorEncoder     = new Encoder(DIO_SLOT, LEFT_WHEEL_ENCODER_BIT_0,
                                           DIO_SLOT, LEFT_WHEEL_ENCODER_BIT_1);
        
        
        Encoder rightMotorEncoder    = new Encoder(DIO_SLOT, RIGHT_WHEEL_ENCODER_BIT_0,
                                           DIO_SLOT, RIGHT_WHEEL_ENCODER_BIT_1);
        
        tankDrive = new TankDrive(
            new Victor(DIO_SLOT, LEFT_MOTOR_PWM_CHANNEL),
            new Victor(DIO_SLOT, RIGHT_MOTOR_PWM_CHANNEL),
            leftMotorEncoder,
            rightMotorEncoder);
        
      //shooterMotor = new Jaguar(kDIOSlot, kShooterMotorPWMChannel);
        shooterMotor = new Victor(DIO_SLOT, SHOOTER_MOTOR_PWM_CHANNEL);

        shooterMotorEncoder  = new Encoder(DIO_SLOT, SHOOTER_ARM_ENCODER_BIT_0, 
                                           DIO_SLOT, SHOOTER_ARM_ENCODER_BIT_1);
        shooterMotorEncoder.setDistancePerPulse(ENCODER_SHOOTER_DISTANCE_PER_PULSE);
        shooterMotorEncoder.start();

        // Note "boolean reverseDirection" parameter may be needed to set output of encoders, See JavaDocs.
        //myEncoder = new Encoder(aSlot, aChannel, bSlot, bChannel, myBackwards, CounterBase.EncodingType.k4X);
        //steeringEncoder = new Encoder(kDIOSlot, kSteeringInputAChannel,
        //                              kDIOSlot, kSteeringInputBChannel,
        //                              false, CounterBase.EncodingType.k2X); // also EncodingType.k4X_val

        gyro = new Gyro(GYRO_ANALOG_AI_CHANNEL);       // Gyro object
        gyro.reset();
        
        // Setup the Ball Roller Motor.
        loader = new Loader(
                new Solenoid(ARM_RETRACT_SOLENOID),
                new Solenoid(ARM_EXTEND_SOLENOID),
                new Victor(DIO_SLOT, BALL_ROLLER_MOTOR_PWM_CHANNEL));
        
        armExtendLimitSwitch  = new DigitalInput(ARM_EXTEND_LIMIT_SWITCH_DI_CHANNEL);
        armRetractLimitSwitch = new DigitalInput(ARM_RETRACT_LIMIT_SWITCH_DI_CHANNEL);
        
        armPosition = new AnalogChannel(ARM_POSITION_POT_AI_CHANNEL);
        
        // Shooter stuff.
        shooterCockedLimitSwitch = new DigitalInput(SHOOTER_COCKED_LIMIT_SWITCH_DI_CHANNEL);
        shooterShotLimitSwitch   = new DigitalInput(SHOOTER_SHOT_LIMIT_SWITCH_DI_CHANNEL);
        
        // Ball loaded sensor (Sharp infrared distance sensor)
        //ballLoadedSensor = new AnalogChannel(kAIOSolt, kBallLoadedSensorAIChannel);
        ballLoadedSensor = new AnalogChannel(BALL_LOADED_SENSOR_AI_CHANNEL);

        // Autonomous stuff...
        autonomousBit0 = new DigitalInput(AUTONOMOUS_BIT_0_DI_CHANNEL);
        autonomousBit1 = new DigitalInput(AUTONOMOUS_BIT_1_DI_CHANNEL);
        autonomousBit2 = new DigitalInput(AUTONOMOUS_BIT_2_DI_CHANNEL);

        autonomousMode = ((autonomousBit2.get() == LIMIT_SWITCH_PRESSED) ? 4 : 0) |
                         ((autonomousBit1.get() == LIMIT_SWITCH_PRESSED) ? 2 : 0) |
                         ((autonomousBit0.get() == LIMIT_SWITCH_PRESSED) ? 1 : 0);
        System.out.println("Autonomous mode: " + StringUtils.format(autonomousMode, 2));
        // Switches behave just like a limit switch on a Digital IO.

        diagnosticSelector = new AnalogChannel(DIAGNOSTIC_SELECTOR_AI_CHANNEL);
        //System.out.println("diagnosticSelector: " + format(diagnosticSelector.getAverageValue (),6, 3));

        shootState = shootNextState = shootPastState = 0; // Shooting state machine controls.
        shootButtonPressed = false;                 // Assume the shoot button is not pressure

        aState = aStateNext = aStatePast = 0;       // Autonomous state machine controls.    

        airCompressor = new Compressor(NASSON_PRESSURE_SWITCH_DI_CHANNEL, AIR_COMPRESSOR_RELAY_RIO_CHANNEL);
        airCompressor.start();                      // Start the air compressor.

        cameraLights = new Relay (DIO_SLOT, CAMERA_LIGHTS_RELAY_RIO_CHANNEL, Relay.Direction.kBoth);
        cameraLightsOn   = Relay.Value.kForward;
        //cameraLightsOn = Relay.Value.kReverse;
        cameraLightsOff  = Relay.Value.kOff;
        cameraLights.set (cameraLightsOff);         // Set Default value to off.
        cameraLightToggle = false;

        cameraLightsButtonPressed = JOYSTICK_BUTTON_NOT_PRESSED;
        cameraLightsButtonWasPressed = false;

        highShifterSolenoid = new Solenoid (1);     // Instantiate the Solenoid objects to control the valves.
        lowShifterSolenoid  = new Solenoid (2);

        lowShifterSolenoid.set  (lowShifterHigh);    // Initially set it to high gear.
        highShifterSolenoid.set (higherShifterhigh); // Initially set it to high gear.

        

        // Do not mess with the pneumatics here. Leave it alone. 
        //armExtendSolenoid.set  (false);           // Initially set it to arm retracted.
        //armExtendSolenoid.set (true);             // Initially set it to arm retracted.

        timer = new Timer ();                       // Instantiate the match timer shootDiag6
        timer.reset();

        moveTimer = new Timer ();
        moveTimer.reset ();

        autoDiag5 = StringUtils.TODO_SPACES_21;
    } // public void robotInit()



    public void autonomousInit () {
        periodicTimer.reset();
        timer.start();                              // Start the timer
        airCompressor.start();                      // Startthe air compressor.
        
    } // public void autonomousInit ()

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        // Call the diagnostic display functions.
        // This function will decide what to do.
        displayDiagnostics ();

        // Emergency shut down mode works with DRIVER joystick buttons 10 and 11.
        // This works like a latch.  Once stopped it must be cleared.
        // If in emergence mode, is the emergency clear button pressed?
        // Did someone press the E-stop Clear button?
        if (emergencyStop == true) {
            if ((driveStickL.getRawButton(DRIVE_SWITCH_ESTOP_CLEAR) == JOYSTICK_BUTTON_PRESSED) ||
                (driveStickR.getRawButton(DRIVE_SWITCH_ESTOP_CLEAR) == JOYSTICK_BUTTON_PRESSED) ||   
                (armStick.getRawButton   (DRIVE_SWITCH_ESTOP_CLEAR) == JOYSTICK_BUTTON_PRESSED))   {
                emergencyStop = false;              // Clear the emergency stop.
            } else {
                return;                             // Emergency Stopped. Remain stopped.
            }
        }
        // Did someone press the soft E-stop button?
        if ((driveStickL.getRawButton(DRIVE_SWITCH_ESTOP_SET) == JOYSTICK_BUTTON_PRESSED) ||
            (driveStickR.getRawButton(DRIVE_SWITCH_ESTOP_SET) == JOYSTICK_BUTTON_PRESSED) ||   
            (armStick.getRawButton   (DRIVE_SWITCH_ESTOP_SET) == JOYSTICK_BUTTON_PRESSED))   {
            emergencyStop = true;                   // Set the emergency stop

            tankDrive.stop();
            shooterMotor.set(DRIVE_STOP);
            return;
        }

        aStatePast = aState;                        // Save the past state.
        aState = aStateNext;                        // Set the new state.

        // Do the appropriate autonomous reotine base upon the switch settings.
        switch (autonomousMode) {

            // Autonomous 0 - Do nothing.
            case 0:
                autonomous0();
                break;

            // Autonomous 1 - Drive forward.
            case 1:
                autonomous1();
                break;

            // Autonomous 2 - Shoot the ball at the goal that you are aimed at, then move ahead.
            case 2:
                autonomous2();
                break;

            // Autonomous 3 - Look for the illuminated goal with the camera, turn, shoot, turn, drive forward.
            case 3:
                autonomous3();
                break;

            // TBD
            case 4:
                autonomous4();
                break;

            // TBD
            case 5:
                autonomous5();
                break;

            // TBD
            case 6:
                autonomous6();
                break;

            // TBD
            case 7:
                autonomous7();
                break;

            default:
                System.out.println("Unknown autonomous mode:" + StringUtils.format(autonomousMode, 3));
                autonomousMode = 0;                 // Pretend there is no autonomous mode.

        } // switch (autonomousMode)

        shoot ();    // Call this eveny message loop to run the shooting state machine.
        
        tankDrive.update(periodicTimer.getDeltaTime());

    } // public void autonomousPeriodic()


    public void teleopInit () {
        periodicTimer.reset();
        airCompressor.start();                      // Start the air compressor.
    } // public void teleopInit ()



    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        double yl, yr;
        int i;
        boolean b;

        // Call the diagnostic display functions.
        // This function will decide what to do.
        displayDiagnostics ();

        // Emergency shut down mode works with DRIVER joystick buttons 10 and 11.
        // This works like a latch.  Once stopped it must be cleared.
        // If in emergence mode, is the emergency clear button pressed?
        if (emergencyStop == true) {
            // Is the clear button pressed?
            if ((driveStickL.getRawButton(DRIVE_SWITCH_ESTOP_CLEAR) == JOYSTICK_BUTTON_PRESSED) ||
                (driveStickR.getRawButton(DRIVE_SWITCH_ESTOP_CLEAR) == JOYSTICK_BUTTON_PRESSED) ||   
                (armStick.getRawButton   (DRIVE_SWITCH_ESTOP_CLEAR) == JOYSTICK_BUTTON_PRESSED))   {
                emergencyStop = false;              // Yes, clear the emergency stop.
            } else {
                return;                             // No, emergency Stopped.
            }
        }
        // Is the emergence stop joystick button pressed?

        if ((driveStickL.getRawButton(DRIVE_SWITCH_ESTOP_SET) == JOYSTICK_BUTTON_PRESSED) ||
            (driveStickR.getRawButton(DRIVE_SWITCH_ESTOP_SET) == JOYSTICK_BUTTON_PRESSED) ||   
            (armStick.getRawButton   (DRIVE_SWITCH_ESTOP_SET) == JOYSTICK_BUTTON_PRESSED))   {
            emergencyStop = true;                  // Yes, set the emergency stop

            tankDrive.stop();
            shooterMotor.set(DRIVE_STOP);
            return;
        }

        // +T66
        // Driver joysticks.
        // If either trigger is pulled then shift to low speed.
        i = (driveStickL.getRawButton(DRIVER_SHIFTER_TRIGGER) ? 2 : 0) +
            (driveStickR.getRawButton(DRIVER_SHIFTER_TRIGGER) ? 1 : 0);
        if (i == 0) {                               // Neither trigger button is pressed,
            lowShifterSolenoid.set  (lowShifterHigh); // shift to high speed.
            highShifterSolenoid.set (higherShifterhigh);
        } else {                                    // A trigger button is pressed,
            highShifterSolenoid.set (highShifterLow); // shift to low gear.
            lowShifterSolenoid.set  (lowShifterLow);
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
            // Leave the roller motor on as long as the Eject button is pressed.
        } // if (armStick.getRaw (joyStickArmEjectButton) == kJoystickButtonPressed)

        // Is the shotting trigger button pressed?
        if (armStick.getRawButton (JOYSTICK_ARM_SHOOT_BUTTON) == JOYSTICK_BUTTON_PRESSED) {
            if (shootButtonPressed == false) {      // Yes.  Was the shoot button just pressed?
                shootButtonPressed = true;          // Yes, set a flag.
                shootStart ();                      // Fire in the hole!
            }
        }
        // Else the shooting trigger button released?
        else {              
            if (shootButtonPressed == true) {       // Was the shoot button just released?
                shootButtonPressed = false;         // Yes, set a flag.
                shootStop ();                       // Reset the Shoot function state machine.
            }
        }

        // Manualy toggle the camera lights.
        b = armStick.getRawButton (JOYSTICK_ARM_LIGHT_TOGGLE);
        if (b != JOYSTICK_BUTTON_PRESSED) {          // Is the button pressed?
            if (cameraLightsButtonWasPressed == false) {
                cameraLightsButtonWasPressed = true; // Remember that the button is now pressed in.
                if (cameraLightToggle) {            // Yes.  Was the shoot button just pressed?
                    cameraLights.set (cameraLightsOff); // Set Default value to off.
                    cameraLightToggle = false;
                } else {        
                    cameraLights.set (cameraLightsOn);
                    cameraLightToggle = true;
                }
            }                                       // The button was alread pressedin.
        } else {
            cameraLightsButtonWasPressed = false;   // Remember that the button is no longer pressed pressed.
        }
        // -T66

       shoot ();    // Call this every message loop to run the shooting state machine.

       tankDrive.update(periodicTimer.getDeltaTime());
    } // public void teleopPeriodic()


    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }


    // **********  Other Non-FIRST functions  **********

    // shootStart
    // If shootPastState = shootState = shootNextState = 0 then the shooting is not in process. 
    void shootStart () {
        if (shootState != 0) {
            // Do nothing.  Something is already in process.
        } else {
            // Start the shooting process!!!
            shootNextState = 1;                     // This gets the process started.
            shootTime.reset ();                     // Get the time that the shooting
            shootTime.start ();                     // process was started for diagnostics.
        }
    } // void ShootStart ()

    // shootStop
    // Stop the shooting process.
    // If the shooter is cocking let it continue.
    // If the ball has not yet been shot then stop the process and reset things.
    // The reset depends on how far the the shoot process has progressed.
    void shootStop () {
        if (shootState == 0) {
            // Shoot has not started, do nothing.
        }
        else if (shootState == 1) {
            // The shooting process has begun to check that all is right to shoot.
            // This can be stopped because this state changes nothing.
            shootTime.stop();                       // For diagnostics.
            shootTime.reset();

            shootTimer.stop();                      // for shootermechanism.
            shootTimer.reset();
            shootNextState = 0;                     // Reset to State 0.
        }
        else if (shootState >= 2) {
            // The shooting is under way - there is no way to safely stop.
            // Do nothing and let it reset it self.
        } else {
            // Do nothing and let it reset it self.
        }
    } // void shootStop ()
    
    // Shoot the ball!
    // This method is called in every message loop pass in both autonomous and teleop modes.
    // There are a number of things to check before shooting.
    void shoot () {
        
        shootPastState = shootState;
        shootState = shootNextState;

            //shootDiag6 = "State: 0 1abcde 2 3 4 "; // Full state string.


        switch (shootState) {
            // Shooting not in process when in this state.
            case 0:
                // Do nothing so that ShootStart () can set shootNextState to start the shooting process.

                           // 1234567890123456789012
                shootDiag6 = "State: 0              "; // Print this line if the Shooter diagnostics is shown.
                shootNextState = 0;                 // Just stay in this state until some starts us.
                break;

            // Check if the robot is in the right condition to shoot.
            case 1:
                           // 1234567890123456789012
                shootDiag6 = "State: 0 1a           ";

                // Is the arm cocked and ready to shoot?
                if (shooterCockedLimitSwitch.get () == ARM_NOT_COCKED) {
                    shootNextState = 1;             // No. Arm is not ready, try again later.
                    break;
                }

                           // 1234567890123456789012
                shootDiag6 = "State: 0 1ab          ";

                // Is there a ball in the shooter?
                if (isBallInShooter () == BALL_NOT_IN_SHOOTER) {
                    shootNextState = 1;             // No, a ball is NOT in the shooter.
                    break;
                }

                           // 1234567890123456789012
                shootDiag6 = "State: 0 1abc         ";

                // Is the arm in the Extend position?
                if (armExtendLimitSwitch.get () == LIMIT_SWITCH_NOT_PRESSED) {
                    loader.eject();
                    shootNextState = 1;             // Limit Switch is off! Wait.
                    break;
                }

                           // 1234567890123456789012
                shootDiag6 = "State: 0 1abcd        ";

                loader.stopEjecting();

                // Things look good. Fire!
                shootNextState = 2;
                break;
                
            // Start the shooting sequence.
            case 2:
                           // 1234567890123456789012
                shootDiag6 = "State: 0 1abcd 2      ";

// TODO - TURN ON the motors based upon the distance sensed by the camera, and set the power to the PID loops.

                // Zero the encoder counter before shooting.
                shooterMotorEncoder.reset();

                // Turn on the shooting motor and keep it on.
                shooterMotor.set (SHOOT_AND_COCK_MOTOR_ON); // For now use a fist shooting power.

                shootNextState = 3;                      // Go complete the re-cocking process.
                break;

                
                
            // Stop the shootere motor when either the encoder has travened the desired
            // distance or the shooterMotorLimitSwith is pressed by thhe arm.
            case 3:
                           // 1234567890123456789012
                shootDiag6 = "State: 0 1abcd 2 3    ";

// TODO Put in angle and power settings based on distance calculated from the camera.
                if ((shooterMotorEncoder.get() >= SHOOTER_DISTANCE) ||
                    (shooterShotLimitSwitch.get() == LIMIT_SWITCH_PRESSED)) {
                    shooterMotor.set(SHOOT_AND_COCK_MOTOR_OFF); // Turn the Motor OFF!
//    AnalogChannel armPosition;
//    final int kArmPositionMin = 100;                // Arm in cocked position.
//    final int kArmPositionMax = 600;                // Arm can exceed this angle!

                    shootTimer.reset();
                    shootTimer.start();             // Start a timer.
                    shootNextState = 4;
                    break;
                }

                shootNextState = 3;                     // Keep waiting for the arm to compleet shooting.
                break;
                
            // Wait for a bit for the shooting arm to stop moving before reversing direction.
            case 4:
                           // 1234567890123456789012
                shootDiag6 = "State: 0 1abcd 2 3 4  ";

                if(shootTimer.get() < PAUSE_AFTER_SHOOTING_TIME) { // Check the time.
                    shootNextState = 4;                 // Keep waiting.
                    break;
                }

                shooterMotor.set (SHOOT_AND_COCK_MOTOR_ON_COCK); // Turn the Motor On to Cock the shooter.

                shootNextState = 5;                     // Start cocking t5he arm.
                break;

            // Complete the re-cocking process.
            // Keep the motor on until the cocked limit switch is on again.

            case 5:
                           // 1234567890123456789012
                shootDiag6 = "State: 0 1abcd 2 3 4 5";
// TODO - Check logic!
                if ((shooterMotorEncoder.get () > 0.0d) ||
                    (shooterCockedLimitSwitch.get() == LIMIT_SWITCH_PRESSED)) {
                    shooterMotor.set (SHOOT_AND_COCK_MOTOR_OFF); // Turn the Motor OFF!

                    shootTimer.stop();              // No need for the shoot internal timer for a while.
                    shootTimer.reset();

                    shootTime.stop();               // No need for the shooting process diagnostic timer.
                    shootTime.reset(); 
                    shootNextState = 0;             // Shooter reset is complete.
                    break;                          // Arm is ready to go.
                }

                shootNextState = 5;                 // Arm not ready.
                break;

            default:
                System.out.print ("Shoot unknown state " + StringUtils.format(shootState, 2) + 
                    " from past state " + StringUtils.format(shootPastState, 2) + StringUtils.TODO_CRLF);
                shootNextState = 0;
// TODO - consider calling shootStop.
                break;            

        } // switch (shootState)  

    } // void shoot()
    
    
    // isBallInShooter
    // Returns true  if there is a ball load and ready to go in the shooter.
    // Returns false if there is not a ball in the shooter.
    boolean isBallInShooter() {
        return(ballLoadedSensor.getAverageValue() > 110);
        //Calibration table:
        // Inches      Reading
        //  00            5    infinity
        //  12           50
        //  10          110
        //   8          130
        //   6          150
        //   4          180
        //Sharp 2D120XF 5X Infrared DIstance sensor.
        // See diagnostic 4
    } // boolean isBallInShooter()



    // Autonomous 0 - Do nothing.
    void autonomous0() {
        // Your autonomous code here.
    } // autonomous0()


    // Autonomous 1
    void autonomous1() {
        aState = aStateNext;
        
        switch(aState) {
        
        case 0:
            tankDrive.moveDistance(72, 0.8);
            // Move 72.0 inches, at 0.8 of 1.0 power, and ramp up time is 1 second.
            aStateNext = 1;
            break;

        case 1:
            if (tankDrive.moveDistanceCompleted()) {  // Is the move complete?
                aStateNext = 2;                     // Yes.
                break;
            } else {
                aStateNext = 1;                     // No. Loop.
                break;
            }
    
        case 2:
            aStateNext = 2;                         // Autonomous is completed. Stay here.
            break;
    
        default:
            // error message.
            break;
        
        } // Switch(aState) {
        
    } // autonomous1()


    // autonomous 2
    // Don't used the camera in this autonomous program.
    // Pick up a ball, move forward, shoot and move forward some more.
    void autonomous2() {
        double d;
        
        aState = aStateNext;

//        autoDiag5 = "Auto " + String.format("%2d", autonomousMode)
//            + " St " + String.format("%2d", aState)
//            + " Ps " + String.format("%2d", aStatePast) + "  ";

        switch(aState) {

        // Extend arm and turn on roller motor.   
        case 0:

            loader.extend();

            shooterMotor.set(SHOOT_AND_COCK_MOTOR_ON_SLOW_UP); // Turn on the shooter motor very slowly.
            shootTimer.reset();                     // We are not shooting. So Steal the shoot timer for a second.
            aStateNext = 1;
            break;

        // Lift shooter arm very low power to get it just above the cocked limit switch.
        case 1:
            if (shootTimer.get() < 200) {           // Wait for 200 milli-seconds (0.2 seconds).
                shooterMotor.set (SHOOT_AND_COCK_MOTOR_ON_SLOW_UP);
                aStateNext = 1;                     // Wait a bit longer.
                break;
            }

             // Turn the motor off, time is up.
            shooterMotor.set(SHOOT_AND_COCK_MOTOR_OFF);// Turn the motor off.
            shootTimer.reset();                     // Reset the time.
            shootNextState = 2;                     // Next!
            
            shooterMotor.set(SHOOT_AND_COCK_MOTOR_ON_COCK); // Turn the motor on to cocking speed.
            
            break;
 
        // Wait until the shooting arm slowly drops to the cocked limit switch.
        case 2:
            if(shooterCockedLimitSwitch.get() == ARM_NOT_COCKED) { // Wait for the limit switch to be pressed.
                shooterMotor.set(SHOOT_AND_COCK_MOTOR_ON_COCK);
                aStateNext = 2;                     // Wait a bit longer.
                break;
            }

             // Turn the motor off, the shooting arm is in the Home position.
            shooterMotor.set(SHOOT_AND_COCK_MOTOR_OFF);// Turn the motor off.
            aStateNext = 3;                         // Next.
            break;

        // Wait for the arm to be extended all of the way.
        case 3:
            if(armExtendLimitSwitch.get() == LIMIT_SWITCH_NOT_PRESSED) { // Wait for limit switch press.
	        //armRetractSolenoid.set(false);   // Keep extending the arm...
                //armExtendSolenoid.set (true);    // These do not have to be done again.
                //ballRollerMotor.set(kBallRollerMotorLoadOn);  // Also, turn on the motor to pickup the ball.
                aStateNext = 3;                     // Wait a bit longer.
                break;
            }
            
            // The arm is externded all of the way and the roller is moving in the load direction.

            // Next start a move backward 4 feet to pickup a ball.
            tankDrive.moveDistance(-48 /* inches */, 0.8 /* % max speed */);
            // Move 72.0 inches, at 0.4 of 1.0 power, and ramp up time is 2 second.
            aStateNext = 4;                         // Next.
            break;

        // Move backward 4 feet or until a ball is sensed in the shooter.
        case 4:
            // Is there a ball loaded into the shooter?
            if (isBallInShooter() == BALL_IN_SHOOTER) {
                // Yes, we have a loaded ball!
                moveStop();                        // Stop moving, we have the ball
                loader.retract();

	        // Start moving forward 4 feet plus the distance moved backward.
                tankDrive.moveDistance(48 /* inches */, 0.8 /* % max speed */);
                
                shootNextState = 5;                 // Go wait for the move forward to complete.
                break;
            }
            // A ball is not sensed.

            // Is the move complete?
             if (!tankDrive.moveDistanceCompleted()) {                 // Is the move complete?
                // No, so keep on truckin.
                aStateNext = 4;                     // Keep on roll'in.
                break;
            } else {
                // Start moving forward 4 feet plus the distance moved backward 
                //plus another 4 feet into the scoring zone.
                tankDrive.moveDistance(2 * 48 /* inches */, 0.8 /* % max speed */);

                aStateNext = 8;                     // Move is ended and no ball.
                break;                              // No reason to shoot.
            }                                       // Jump ahead to move forward to the scoring zone

        // Wait for the move complete.
        case 5:
            // Is the move complete?
             if (!tankDrive.moveDistanceCompleted()) {                 // Is the move complete?
                // No, so keep on truckin.
                aStateNext = 5;                     // Keep on roll'in.
                break;
            } else {
                aStateNext = 6;                     // GO and shoot.
                break;
            }
            
        // Some day, put in code to use the camera to look at the target
        // to determine the height of the reflective zone and 
        // convert that to a distance or a power setting.
        // For now just use a fixed power assuming we are at a hot spot on the floor.
            
        // Shoot
        case 6:
            shootStart();                           // Start the independant shooting process.
            aStateNext = 7;                         // Move is ended.
            break;                                  // Wait till the deal'in's done.
            
        // Wait until the ball has been shot.
        // To determine if the shooting is done look at the shooter motor power.
        // If the power is positive the arm is throwing the ball.
        // If the power is negative the arm is being returned to the cocked 
        // position, and it is now OK to move the robot forward.
        case 7:
            if (shooterMotor.get() > 0.0d) {        // Wait till the deal'in's done.
                aStateNext = 7;                     // Keep waiting.
                break;                              
            }

            // Next start a move forward 4 feet into the scoring zone.
            tankDrive.moveDistance(48 /* inches */, 0.4 /* % max speed */);
            // Move 48.0 inches, at 0.4 of 1.0 power, and ramp up time is 2 second.

            aStateNext = 8;                         // Move is ended.
            break;                                  // Wait till the deal'ins done.

        // Drive forward into the scoring zone.
        // NOTE: You can get here from states 4 or 7.
        case 8:
            // Is the move complete?
             if(!tankDrive.moveDistanceCompleted()) {                  // Is the move complete?
                // No, so keep on truckin.
                aStateNext = 5;                     // Keep roll'in.
                break;
            } else {
                aStateNext = 6;                     // Move is ended.
                break;                              // Go shoot.
            }
        
	// End of Autonomous.
        case 9:
            aStateNext = 6;                         // Stay here forever...
            break;                                  // Or until Autonomous mode ends...
                                                    // Which ever comes first.

        default:
            // error message.
            break;
        
        } // Switch (aState) {

    } // autonomous2()   


    // autonomous 3
    void autonomous3() {
        // Your autonomous code here.
   
    } // autonomous3()    

    
    // autonomous 4
    void autonomous4() {
        // Your autonomous code here.
   
    } // autonomous4()    

    
    // autonomous 5
    void autonomous5() {
        // Your autonomous code here.
   
    } // autonomous5()    
    
    
    // autonomous 6
    void autonomous6() {
        // Your autonomous code here.
   
    } // autonomous6()    

    
    // autonomous 7
    void autonomous7() {
        // Your autonomous code here.
   
    } // autonomous()    



    // Stop the robot NOW!
    // Just tell the state machine to end the move.
    void moveStop () {
        // Start braking.
        // This is the fastest way to stop.
        moveNextState = 3;

        // Apply brakes.
        tankDrive.stop();

    } // Move_Stop ()


    void displayDiagnostics () {
        int d;                                     // Which diagnostic to run.
        String b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11;
        String s;
        final String sP = "P";                      // Note that often used string constants are
        final String sS = "_";                      // best made finals, else the compiler may make
                                                    // each one unique wasting memory.

        // Decide which diagnostic routine to display, if any.
        //d = diagnosticSelector.getAverageValue();   // Get the POT value.
        d = diagnosticSelector.getValue();          // Get the POT value.
        d = (d < 0) ? 0 : d;                        // Sometimes it returns a negative number.
        d = d / 107;                                // Compute the diagnostic to display, should be 1 to 9.
                                                    // My POT returns values from -3 to 963. (963/9=107)
        d = (d <= 9) ? d : 9;                       // Limit the maximum value.

        switch (d) {
            // No diagnostics to display.
            // Note never run diagnostics during a competition match, it can take excess CPU time.
            case 0:
                if (diagOff == false) {             // Do this once per entry into diagnostice Off.
                    diagOff = true;                              // 123456789012345678901
                    dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
				  + " " + StringUtils.format (diagnosticSelector.getValue(),5));
                    dslcd.println(DriverStationLCD.Line.kUser2, 1, "Diagnostics Off       ");
                    dslcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
                    dslcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
                    dslcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);
                    dslcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);
                } // if (diagOff != false;
                break;

            // Driver joystick left.
            case 1:
                                                             // 123456789012345678901
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Driver Joystick Left  ");

                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X"  + StringUtils.format(driveStickL.getX(), 2, 2) +
                    " Y" + StringUtils.format(driveStickL.getY(), 2, 2) +
                    " Z" + StringUtils.format(driveStickL.getZ(), 2, 2));

                b1 = ((driveStickL.getRawButton(1)) ? sP : sS);
                b2 = ((driveStickL.getRawButton(2)) ? sP : sS);
                b3 = ((driveStickL.getRawButton(3)) ? sP : sS);
                b4 = ((driveStickL.getRawButton(4)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T   " + b1 + " B2 " + b2 + " B3 " + b3 + " B4 " + b4);

                b5 = ((driveStickL.getRawButton(5)) ? sP : sS);
                b6 = ((driveStickL.getRawButton(6)) ? sP : sS);
                b7 = ((driveStickL.getRawButton(7)) ? sP : sS);
                b8 = ((driveStickL.getRawButton(8)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + " B6 " + b6 + " B7 " + b7 + " B8 " + b8);

                b9 = ((driveStickL.getRawButton(9)) ? sP : sS);
                b10 = ((driveStickL.getRawButton(10)) ? sP : sS);
                b11 = ((driveStickL.getRawButton(11)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + " B10 " + b10 +  " B11 " + b11);
                diagOff = false;
                break;

            // Driver joystick right.
            case 2:
                                                             // 123456789012345678901
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Driver Joystick Right ");

                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X"  + StringUtils.format(driveStickR.getX(), 2, 2) +
                    " Y" + StringUtils.format(driveStickR.getY(), 2, 2) +
                    " Z" + StringUtils.format(driveStickR.getZ(), 2, 2));

                b1 = ((driveStickR.getRawButton(1)) ? sP : sS);
                b2 = ((driveStickR.getRawButton(2)) ? sP : sS);
                b3 = ((driveStickR.getRawButton(3)) ? sP : sS);
                b4 = ((driveStickR.getRawButton(4)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T   " + b1 + " B2 " + b2 + " B3 " + b3 + " B4 " + b4);

                b5 = ((driveStickR.getRawButton(5)) ? sP : sS);
                b6 = ((driveStickR.getRawButton(6)) ? sP : sS);
                b7 = ((driveStickR.getRawButton(7)) ? sP : sS);
                b8 = ((driveStickR.getRawButton(8)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + " B6 " + b6 + " B7 " + b7 + " B8 " + b8 );

                b9 = ((driveStickR.getRawButton(9)) ? sP : sS);
                b10 = ((driveStickR.getRawButton(10)) ? sP : sS);
                b11 = ((driveStickR.getRawButton(11)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + " B10 " + b10 +  " B11 " + b11);
                diagOff = false;
                break;

            // Shooter joystick
            case 3:
                                                             // 123456789012345678901
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter Joystick      ");

                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X"  + StringUtils.format(armStick.getX(), 2, 2) +
                    " Y" + StringUtils.format(armStick.getY(), 2, 2) +
                    " Z" + StringUtils.format(armStick.getZ(), 2, 2));

                b1 = ((armStick.getRawButton(1)) ? sP : sS);
                b2 = ((armStick.getRawButton(2)) ? sP : sS);
                b3 = ((armStick.getRawButton(3)) ? sP : sS);
                b4 = ((armStick.getRawButton(4)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T   " + b1 + " B2 " + b2 + " B3 " + b3 + " B4 " + b4);

                b5 = ((armStick.getRawButton(5)) ? sP : sS);
                b6 = ((armStick.getRawButton(6)) ? sP : sS);
                b7 = ((armStick.getRawButton(7)) ? sP : sS);
                b8 = ((armStick.getRawButton(8)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + " B6 " + b6 + " B7 " + b7 + " B8 " + b8);

                b9 = ((armStick.getRawButton(9)) ? sP : sS);
                b10 = ((armStick.getRawButton(10)) ? sP : sS);
                b11 = ((armStick.getRawButton(11)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + " B10 " + b10 + " B11 " + b11);
                diagOff = false;
                break;

            // Shooter Diagnostics.
            case 4:
                                                             // 123456789012345678901
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                tm = shootTime.get ()/10.0d;                 // Make id 1/1000 to 1/100 of sec.
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter        " + StringUtils.format(tm, 3, 2));
                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "Ball Loaded " + ((isBallInShooter ()) ? "Y " : "N ") 
                    + StringUtils.format (ballLoadedSensor.getValue(), 6));
                s = "Ckd " + ((shooterCockedLimitSwitch.get () == ARM_COCKED) ? "Y " : "N ") +
                    "Mn/Mx " + StringUtils.format(ARM_POSITION_MIN, 4) + "/" + StringUtils.format(ARM_POSITION_MAX, 4);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, s);
                dslcd.println(DriverStationLCD.Line.kUser5, 1,
                   "Arm Position " + StringUtils.format(armPosition.getValue(), 3) + "     ");
                dslcd.println(DriverStationLCD.Line.kUser6, 1, shootDiag6); // Use text from shoot function.
                diagOff = false;
                break;

            // Shooter Diagnostics 2.
            case 5:
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter 2      " + StringUtils.format(tm, 3, 2));
                dslcd.println(DriverStationLCD.Line.kUser3, 1,
                    "Arm Ext LS " + ((armExtendLimitSwitch.get()==LIMIT_SWITCH_PRESSED) ? "P" : "_") + "         ");
                dslcd.println(DriverStationLCD.Line.kUser4, 1,
                    "Arm Ret LS " + ((armRetractLimitSwitch.get()==LIMIT_SWITCH_PRESSED) ? "P" : "_") + "         ");
                dslcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);        
                dslcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);
                diagOff = false;
                break;

            // Drive motor power
            case 6:
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Drive motor power    ");
                dslcd.println(DriverStationLCD.Line.kUser3, 1, "Left motor:   " + StringUtils.format ( leftWheelPower, 4, 3));
                dslcd.println(DriverStationLCD.Line.kUser4, 1, "Shoot: " + StringUtils.format (shooterMotorEncoder.getDistance(),6, 2));;
//                dslcd.println(DriverStationLCD.Line.kUser4, 1, "Right motor:  " + format (rightWheelPower, 4, 3));
                dslcd.println(DriverStationLCD.Line.kUser5, 1, "Left:  " + StringUtils.format(tankDrive.getLeftEncoderDistance(),6, 2));
                dslcd.println(DriverStationLCD.Line.kUser6, 1, "Right: " + StringUtils.format(tankDrive.getRightEncoderDistance(),6, 2));
                diagOff = false;
                break;

            // Miscellaneus...
            case 7:
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Miscellaneus...      ");
                //dslcd.println(DriverStationLCD.Line.kUser3, 1, kSpaces21);
//                s = "Gyro " + String.format("%f3.2", gyro.getAngle());
                s = "Gyro " + StringUtils.format(gyro.getAngle(),3,2);
                dslcd.println(DriverStationLCD.Line.kUser3, 1, s);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
                diagOff = false;
                break;

            // Ball pickup arm
            case 8:
                                                             // 123456789012345678901
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) + " " 
                    + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Ball pickup arm      ");
                s =      "Ext LS "  + ((armExtendLimitSwitch.get () == LIMIT_SWITCH_NOT_PRESSED) ? "P" : "_");
                s = s + " Ret LS " + ((armRetractLimitSwitch.get() == LIMIT_SWITCH_NOT_PRESSED) ? "P" : "_");
                dslcd.println(DriverStationLCD.Line.kUser3, 1, s);
                dslcd.println(DriverStationLCD.Line.kUser4, 1,
                    "Ext Sol " + (loader.isExtending() ? "N" : "F")
                 + " Ret Sol " + (loader.isRetracting() ? "N" : "F"));
                    // "N" = "oN",  "F" = "oFf".
                dslcd.println(DriverStationLCD.Line.kUser5, 1,
                     "Roller Motor " + StringUtils.format(loader.getRollorSpeed(), 3,1));
                dslcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
                diagOff = false;
                break;

            // TBD
            case 9:
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, autoDiag5);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, shootDiag6); // Use text from shoot function.
                diagOff = false;
                break;
                

                
/*            // TBD
            case 9:
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + format (d, 2) 
                    + " " + format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser2, 1, kSpaces21);
                dslcd.println(DriverStationLCD.Line.kUser3, 1, kSpaces21);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, kSpaces21);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, kSpaces21);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, kSpaces21);        
                diagOff = false;
                break;
*/                
            default:
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "Diag:" + StringUtils.format (d, 3) + " Bad Selection");
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                dslcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
                diagOff = false;
                break;
            }  //switch ()

        dslcd.updateLCD();

    } // void displayDiagnostics ()

} // public class RobotTemplate extends IterativeRobot
