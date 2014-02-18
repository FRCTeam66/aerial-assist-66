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
import edu.wpi.first.team66.params.ControllerParams;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;


import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.PIDController;


import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.team66.params.StateParams;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends IterativeRobot implements IOParams, StateParams, ControllerParams {

    private DiagnosticDisplay diagnostics = null;
    
    private Inputs inputs = null;
    
    private DeltaTimer periodicTimer = null;
    
    private TankDrive tankDrive = null;
    
    private Loader loader = null;
    
    private Shooter shooter = null;
    
    private StateMachine autoStateMachine = null;
    
    private Gyro gyro;
    
    private Compressor airCompressor;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        periodicTimer = new DeltaTimer();
        
        Joystick driverController = new Joystick(DRIVER_CONTROLLER_ID);
        Joystick shooterController = new Joystick(SHOOTER_CONTROLLER_ID);
        
        inputs = new Inputs(
                driverController,
                shooterController,
                new DigitalInput(AUTONOMOUS_BIT_0_DI_CHANNEL),
                new DigitalInput(AUTONOMOUS_BIT_1_DI_CHANNEL),
                new DigitalInput(AUTONOMOUS_BIT_2_DI_CHANNEL));
        
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
        shooterMotorEncoder.setReverseDirection(IS_SHOOTER_MOTOR_REVERSED);
        shooterMotorEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
        shooterMotorEncoder.start();
        
        DigitalInput shooterCockedLimitSwitch = new DigitalInput(SHOOTER_COCKED_LIMIT_SWITCH_DI_CHANNEL);
        DigitalInput shooterShotLimitSwitch   = new DigitalInput(SHOOTER_SHOT_LIMIT_SWITCH_DI_CHANNEL);
        
        AnalogChannel ballLoadedSensor = new AnalogChannel(BALL_LOADED_SENSOR_AI_CHANNEL);
        
        AnalogChannel shooterAbsoluteAngle = new AnalogChannel(ARM_POSITION_POT_AI_CHANNEL);
        
        PIDController shooterPositionPID = new PIDController(0.02d,0d,0d,shooterAbsoluteAngle,shooterMotor);
        PIDController shooterSpeedPID = new PIDController(0.8d,0d,0d,0.5d,shooterMotorEncoder,shooterMotor);
        
        airCompressor = new Compressor(NASSON_PRESSURE_SWITCH_DI_CHANNEL, AIR_COMPRESSOR_RELAY_RIO_CHANNEL);
        airCompressor.start();
        
        shooter = new Shooter(
                shooterMotor,
                shooterMotorEncoder,
                ballLoadedSensor,
                shooterAbsoluteAngle,
                shooterCockedLimitSwitch,
                shooterShotLimitSwitch,
                shooterPositionPID,
                shooterSpeedPID,
                airCompressor,
                loader);

        // Misc.
        gyro = new Gyro(GYRO_ANALOG_AI_CHANNEL);
        gyro.reset();

        autoStateMachine = new StateMachine();
        
        diagnostics = new DiagnosticDisplay(
                DriverStation.getInstance(),
                DriverStationLCD.getInstance(),
                new AnalogChannel(DIAGNOSTIC_SELECTOR_AI_CHANNEL),
                driverController,
                shooterController,
                null,
                tankDrive,
                shooter,
                loader,
                inputs);
    }

    public void autonomousInit () {
        periodicTimer.reset();
        airCompressor.start();
        
        switch (inputs.getAutonomousMode()) {
            case AMODE_DO_NOTHING:
                autoStateMachine.setState(DoNothingState.instance);
                break;
            case AMODE_DRIVE_FORWARD:
                autoStateMachine.setState(Mode1InitState.instance);
                break;
            default:
                autoStateMachine.setState(DoNothingState.instance);
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        diagnostics.displayDiagnostics();

        autoStateMachine.update(tankDrive, loader, shooter);
        
        double deltaTime = periodicTimer.getDeltaTime();
        shooter.update(deltaTime, false /* Do not override loader limit switch state */);
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
        diagnostics.displayDiagnostics();
        
        if (inputs.getShiftMode() == SHIFT_MODE_HIGH) {
            tankDrive.shiftHighGear();
        } else {
            tankDrive.shiftLowGear();
        }
        
        tankDrive.setTargetSpeed(
                inputs.getLeftDriveSpeed(),
                inputs.getRightDriveSpeed());

        if (inputs.getExtendButton())
        {
            loader.extend();
        }
        else if (inputs.getRetractButton())
        {
            loader.retract();
        }
        
        if (inputs.getRollOutButton())
        {
            loader.setRollerState(Loader.ROLLER_OUT);
        }
        else if (inputs.getRollInButton())
        {
            loader.setRollerState(Loader.ROLLER_IN);
        }
        else
        {
            loader.setRollerState(Loader.ROLLER_OFF);
        }
        
        if (inputs.getShootButton()) {
            // TODO
            shooter.shoot(800, 145);
        }
        else if(inputs.getTrussTossButton()){
            shooter.trussToss();
        }
        else{
            shooter.stop();
        }
        
        if (inputs.getShootHomeButton()){
            shooter.resetShooter();
        }
            
        
        double deltaTime = periodicTimer.getDeltaTime();
        boolean checkExtended = !inputs.getOverrideLoaderExtendedCheckButton();
        if(inputs.getManualShooterControlButton())
        {
            double speed = inputs.getManualShooterControlSpeed();
            shooter.manualControl(speed, checkExtended);
        }
        else
        {
            shooter.update(deltaTime, checkExtended);
        }
        tankDrive.update(deltaTime);
    }


    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
}
