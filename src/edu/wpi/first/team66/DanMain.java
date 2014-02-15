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

import edu.wpi.first.wpilibj.Relay;
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
public class DanMain extends IterativeRobot implements IOParams, StateParams {

    private DeltaTimer periodicTimer = null;
    
    private TankDrive tankDrive = null;
    
    private Loader loader = null;
    
    private Shooter shooter = null;
    
    // Driver Station and Diagnostics Display
    DriverStation ds;                               // Define the drivers station object
    DriverStationLCD dslcd;
    Hand hand;                                      // Required for the trigger function
    
    // Declare variables for the two joysticks being used
    Joystick driveStickL;                           // Joystick Left  (tank drive)
    Joystick driveStickR;                           // Joystick Right (tank drive)
    Joystick armStick;                              // Joystick for the arm

    // Gyro objects and constants. The Gyro object takes care of everything for us.
    Gyro gyro;                                      // Gyro for autonomous steering.

    // Diagnostic selector is a linear variable Pot
    // If not connected an analog input port returns a vlalue of < 0
    // Take the value from .get() and divide it by 107.
    //   will be 0 - 9 as the diagnostic modes. 0 is no diagnostics.
    AnalogChannel diagnosticSelector;

    // Time of autonomous or telop modes
    Timer timer = new Timer();

    private int autonomousMode = 0;     // Mode is a number between 0 and 7.
    double autonomousStopTime = 0.0;    // Used as safety time in autonomous.

    DigitalInput autonomousBit0;
    DigitalInput autonomousBit1;
    DigitalInput autonomousBit2;

    // Ball Roller Motor Declarations and Constants.

    DigitalInput armExtendLimitSwitch;
    DigitalInput armRetractLimitSwitch;

    AnalogChannel armPosition;

// TODO CALIBRATE if needed

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
        // Switches behave just like a limit switch on a Digital IO.

        diagnosticSelector = new AnalogChannel(DIAGNOSTIC_SELECTOR_AI_CHANNEL);

        shootButtonPressed = false;                 // Assume the shoot button is not pressure

        aState = aStateNext = aStatePast = 0;       // Autonomous state machine controls.    

        airCompressor = new Compressor(NASSON_PRESSURE_SWITCH_DI_CHANNEL, AIR_COMPRESSOR_RELAY_RIO_CHANNEL);
        airCompressor.start();                      // Start the air compressor.

        cameraLights = new Relay (DIO_SLOT, CAMERA_LIGHTS_RELAY_RIO_CHANNEL, Relay.Direction.kBoth);
        cameraLightsOn   = Relay.Value.kForward;
        cameraLightsOff  = Relay.Value.kOff;
        cameraLights.set (cameraLightsOff);         // Set Default value to off.
        cameraLightToggle = false;

        cameraLightsButtonPressed = JOYSTICK_BUTTON_NOT_PRESSED;
        cameraLightsButtonWasPressed = false;

        timer = new Timer ();                       // Instantiate the match timer shootDiag6
        timer.reset();

        autoDiag5 = StringUtils.TODO_SPACES_21;
    }

    public void autonomousInit () {
        periodicTimer.reset();
        timer.start();
        airCompressor.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        displayDiagnostics();

        aStatePast = aState;                        // Save the past state.
        aState = aStateNext;                        // Set the new state.

        // Do the appropriate autonomous reotine base upon the switch settings.
        switch (autonomousMode) {

            // Autonomous 0 - Do nothing.
            case AMODE_DO_NOTHING:
                autonomousDoNothing();
                break;

            // Autonomous 1 - Drive forward.
            case AMODE_DRIVE_FORWARD:
                autonomous1();
                break;

            // Autonomous 2 - Shoot the ball at the goal that you are aimed at, then move ahead.
            case AMODE_SHOOT_THEN_MOVE:
                autonomous2();
                break;

            // Autonomous 3 - Look for the illuminated goal with the camera, turn, shoot, turn, drive forward.
            case AMODE_AIM_SHOOT_DRIVE_FORWARD:
                autonomous3();
                break;

            default:
                System.out.println("Unknown autonomous mode:" + StringUtils.format(autonomousMode, 3));
                autonomousMode = 0;                 // Pretend there is no autonomous mode.
        }
        
        double deltaTime = periodicTimer.getDeltaTime();
        shooter.update(deltaTime);
        tankDrive.update(deltaTime);

    }


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
            // Leave the roller motor on as long as the Eject button is pressed.
        } // if (armStick.getRaw (joyStickArmEjectButton) == kJoystickButtonPressed)

        // Is the shotting trigger button pressed?
        if (armStick.getRawButton (JOYSTICK_ARM_SHOOT_BUTTON) == JOYSTICK_BUTTON_PRESSED) {
            if (shootButtonPressed == false) {      // Yes.  Was the shoot button just pressed?
                shootButtonPressed = true;          // Yes, set a flag.
                shooter.shoot();
            }
        }
        // Else the shooting trigger button released?
        else {              
            if (shootButtonPressed == true) {       // Was the shoot button just released?
                shootButtonPressed = false;         // Yes, set a flag.
                shooter.stop();                   // Reset the Shoot function state machine.
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

        double deltaTime = periodicTimer.getDeltaTime();
        shooter.update(deltaTime);
        tankDrive.update(deltaTime);
    }


    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }


    // **********  Other Non-FIRST functions  **********

    void autonomousDoNothing() {
    }

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
            throw new RuntimeException("invalid autonomous1 state");
        }
        
    }

    // Don't used the camera in this autonomous program.
    // Pick up a ball, move forward, shoot and move forward some more.
    void autonomous2() {
        
    }

    void autonomous3() {
    }

    void moveStop () {
        tankDrive.stop();
    }

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
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter        " + StringUtils.format(0, 3, 2));
                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "Ball Loaded " + (shooter.isBallInShooter() ? "Y " : "N "));
                s = "Ckd " + (shooter.isCocked() ? "Y " : "N ") +
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
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter 2      " + StringUtils.format(0, 3, 2));
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
                dslcd.println(DriverStationLCD.Line.kUser4, 1, "Shoot: " + StringUtils.format (shooter.getShooterRate(),6, 2));;
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

}
