package edu.wpi.first.team66;

import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PIDController;

public class Shooter implements IOParams {

    private static final int STATE_INIT = -1;
    private static final int STATE_IDLE = 0;
    private static final int STATE_BALL_LOADED = 1;
    private static final int STATE_PREFIRE_CHECKS = 2;
    private static final int STATE_SHOOTING = 3;
    private static final int STATE_STOPPING = 4;
    private static final int STATE_RESETTING = 5;
    private static final int STATE_UNKNOWN = 6;
    
    private static final int NO_COMMAND     = 0;
    private static final int HIGH_GOAL      = 1;
    private static final int TRUSS_TOSS     = 2;
    private static final int BOUNCE_PASS    = 3;
    private static final int RESET          = 4;
    
    private static final double VOLT_TO_ANGLE_SLOPE = -148.76d;
    private static final double VOLT_TO_ANGLE_INTERCEPT = 747.52d;
    
    private static final double HOME_POSITION = 25.0d;
    private static final double HOME_POSITION_MIN = 20.0d;
    private static final double HOME_POSITION_MAX = 30.0d;
    
    private static final double SHOOTER_STOPPED_RATE = 1.0d;
    
    //DIO pins pulled high, and switches N.O, so define switch press in 
    //terms of T & F
    private static final boolean SHOOTER_SW_PRESSED = false;
    
    private final SpeedController shooterMotor;
    private final Encoder shooterMotorEncoder;
    
    private final AnalogChannel ballLoadedSensor;
    public final AnalogChannel shooterAbsoluteAngle;

    private final DigitalInput shooterCockedLimitSwitch;
    private final DigitalInput shooterShotLimitSwitch;
    
    //PIDs
    private final PIDController shooterPositionPID;
    private final PIDController shooterSpeedPID;

    private final Loader loader;
    
    public int currentState = STATE_INIT;
    
    private int shootCommand = NO_COMMAND;
    
    private double shootSpeed;
    private double shootAngle;
    private boolean shooterReturnFlag;

    public Shooter(
            SpeedController shooterMotor,
            Encoder shooterMotorEncoder,
            AnalogChannel ballLoadedSensor,
            AnalogChannel shooterAbsoluteAngle,
            DigitalInput shooterCockedLimitSwitch,
            DigitalInput shooterShotLimitSwitch,
            PIDController shooterPositionPID,
            PIDController shooterSpeedPID,
            Loader loader)
    {
        this.shooterMotor = shooterMotor;
        this.shooterMotorEncoder = shooterMotorEncoder;
        this.ballLoadedSensor = ballLoadedSensor;
        this.shooterAbsoluteAngle = shooterAbsoluteAngle;
        this.shooterCockedLimitSwitch = shooterCockedLimitSwitch;
        this.shooterShotLimitSwitch = shooterShotLimitSwitch;
        this.shooterPositionPID = shooterPositionPID;
        this.shooterSpeedPID = shooterSpeedPID;
        this.loader = loader;
        
        shooterMotor.set(0);
    }
    
    public boolean isBallInShooter()
    {
        return ballLoadedSensor.getAverageValue() > 100;
    }
    
    public void shoot(double speed, double releaseAngle)
    {   
        shootCommand = HIGH_GOAL;
        shootSpeed = speed;
        shootAngle = releaseAngle;
        
    }
    
    public boolean trussToss()
    {
        if (currentState == STATE_IDLE)
        {
            // think about being able to shoot
        }
        return false;
    }
    
    public boolean stop()
    {
        // TODO try to stop if we aren't committed
        return false;
    }
    
    public void manualControl(double speed, boolean checkLoaderExtended)
    {
        // The autmotaic shooting state machine
        // no longer knows the shooter's state.
        currentState = STATE_UNKNOWN;
        
        if(loader.isExtended() || !checkLoaderExtended)
        {
            // TODO: Check shooter arm limit switches to prevent moving
            // in a bad direction.
            setShooterSpeed(speed * 0.3);
        }
        else
        {
            setShooterSpeed(0);
        }
    }
    
    public boolean resetShooter()
    {
        shootCommand = RESET;
        return false;    
    }
    
    public void update(double deltaTime, boolean checkLoaderExtended)
    {       
        switch (currentState) {

            case STATE_INIT:
                //Setup position control PID to go to home position 
                setPositionPID(25d,4d);
                shooterPositionPID.enable();
                currentState = STATE_IDLE;              
                break;
                
            case STATE_IDLE:
                //Shooter in home position, no ball in robot
                //Home position maintained by PID control?
                if (isBallInShooter())
                {
                    currentState = STATE_BALL_LOADED;
                }
                else
                {
                    if (shootCommand != NO_COMMAND)
                    {
                        //Reset shoot command since nothing to shoot
                        shootCommand = NO_COMMAND;
                    }
                }     
                break;
                      
            case STATE_BALL_LOADED:
                //Shooter in home position, ball in robot, wait for shoot command
                //Home position maintained by PID control?
                if (!isBallInShooter())
                {
                    //If ball ejected/lost return to STATE_IDLE
                    currentState = STATE_IDLE;
                }
                else if (shootCommand != NO_COMMAND)           
                {
                    currentState = STATE_PREFIRE_CHECKS;
                }
                else
                {
                    //Do nothing
                }                 
                break;
                
            case STATE_PREFIRE_CHECKS:
                //Shooter in home position, ball in robot, shoot command sent
                //Check if ball can be launched
                
                // Is the arm in the Extend position?
                if (!loader.isExtended()) 
                {
                    loader.extend();
                }
                else
                {
                    shooterPositionPID.disable();
                    //setSpeedPID(shootSpeed,4d);
                    //shooterSpeedPID.enable();
                    setShooterSpeed(-1.0);
                    shootCommand = NO_COMMAND;
                    currentState = STATE_SHOOTING;
                    // TODO transition acshooterSpeedPIDtions   
                }
                break;

            case STATE_SHOOTING:
                //If we get here all conditions satisfied to launch so fire away!
                //PID will control output until target angle reached 

                if (shooterAbsoluteAngle.getValue()>= shootAngle)
                {
                    //Stop the motor                    
                    shooterSpeedPID.disable();
                    setShooterSpeed(0);
                    currentState = STATE_STOPPING;
                }
                else
                {
                    //Do nothing
                }
                                    
                break;
                
            case STATE_STOPPING:
                
                if (isStopped())
                {
                    //Stop velocity PID
                    shooterSpeedPID.disable();
                    setShooterSpeed(0.2);
                    currentState = STATE_RESETTING;
                }
                else
                {
                    //Do Nothing
                } 
                break;
                
            case STATE_RESETTING:
                
                if (isInHomePosition())
                {
                    setShooterSpeed(0);
                    setPositionPID(25d,4d);
                    shooterPositionPID.enable();
                    currentState = STATE_IDLE;   
                }
                else
                {
                    //Do Nothing
                }
                break;
                
            case STATE_UNKNOWN:
                //Disable PID control
                shooterPositionPID.disable();
                shooterSpeedPID.disable();
                
                //if(shootCommand == RESET)
                //{
                //}
        }
    }
    
    public double getShooterAngle()
    {
        //y=mx+b conversion of position sensor voltage to angle in degrees
        
        return((VOLT_TO_ANGLE_SLOPE * shooterAbsoluteAngle.getVoltage()) + VOLT_TO_ANGLE_INTERCEPT);
    }
    
    public double getShooterRate()
    {
        return shooterMotorEncoder.getRate();
    }
    
    public double getShooterDistance()
    {
        return shooterMotorEncoder.getDistance();
    }
    
    public boolean isInHomePosition()
    {
        return ((shooterAbsoluteAngle.getValue() <= HOME_POSITION_MAX));
    }
    
    public boolean isInInitPosition()
    {
        return shooterCockedLimitSwitch.get() == SHOOTER_SW_PRESSED ? true:false;
    }
    
    private boolean isStopped()
    {
        //TODO: What is the rate?
        return shooterMotorEncoder.getRate() < SHOOTER_STOPPED_RATE;
    }
    
    private void setShooterSpeed(double speed)
    {
        shooterMotor.set((IS_SHOOTER_MOTOR_REVERSED ? -1:1)* speed);
    }
    
    private void setPositionPID(double position, double tolerance)
    {
        shooterPositionPID.setSetpoint(position);
        shooterPositionPID.setAbsoluteTolerance(tolerance);
    }
    
    private void setSpeedPID(double speed, double tolerance)
    {
        shooterSpeedPID.setSetpoint(speed);
        shooterSpeedPID.setAbsoluteTolerance(tolerance);     
    }
    
}
