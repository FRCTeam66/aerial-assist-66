package edu.wpi.first.team66;

import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class Shooter implements IOParams {

    private static final int STATE_INIT = -1;
    private static final int STATE_COCKED_AND_IDLE = 0;
    private static final int STATE_PREFIRE_CHECKS = 1;
    private static final int STATE_CHAMBER_BALL = 2;
    private static final int STATE_SHOOTING = 3;
    private static final int STATE_STOPPING = 4;
    private static final int STATE_RESETTING = 5;
    
    private final SpeedController shooterMotor;
    private final Encoder shooterMotorEncoder;
    
    private final AnalogChannel ballLoadedSensor;

    private final DigitalInput shooterCockedLimitSwitch;
    private final DigitalInput shooterShotLimitSwitch;

    private final Loader loader;
    
    private int currentState = STATE_INIT;

    public Shooter(
            SpeedController shooterMotor,
            Encoder shooterMotorEncoder,
            AnalogChannel ballLoadedSensor,
            DigitalInput shooterCockedLimitSwitch,
            DigitalInput shooterShotLimitSwitch,
            Loader loader)
    {
        this.shooterMotor = shooterMotor;
        this.shooterMotorEncoder = shooterMotorEncoder;
        this.ballLoadedSensor = ballLoadedSensor;
        this.shooterCockedLimitSwitch = shooterCockedLimitSwitch;
        this.shooterShotLimitSwitch = shooterShotLimitSwitch;
        this.loader = loader;
        
        shooterMotor.set(0);
    }
    
    public boolean isBallInShooter()
    {
        return ballLoadedSensor.getAverageValue() > 110;
    }
    
    public boolean shoot()
    {
        if (currentState == STATE_COCKED_AND_IDLE)
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
    
    public void update(double deltaTime)
    {
        shooterMotor.set(0);
//        
//        switch (currentState) {
//            // Shooting not in process when in this state.
//            case STATE_COCKED_AND_IDLE:
//                break;
//
//            case STATE_PREFIRE_CHECKS:
//
//                // Is the arm cocked and ready to shoot?
//                if (shooterCockedLimitSwitch.get () == ARM_NOT_COCKED) {
//                    break;
//                }
//
//                // Is there a ball in the shooter?
//                if (isBallInShooter () == BALL_NOT_IN_SHOOTER) {
//                    break;
//                }
//
//                // Is the arm in the Extend position?
//                if (!loader.isExtended()) {
//                    loader.extend();
//                    break;
//                }
//                
//                currentState = STATE_SHOOTING;
//                // TODO transition actions
//                
//                break;
//            case STATE_CHAMBER_BALL:
//                if (isBallChambered())
//                {
//                    currentState = STATE_SHOOTING;
//                    break;
//                }
//                break;
//            case STATE_SHOOTING:
//
//                if (isTagetSpeedReached())
//                {
//                    currentState = STATE_STOPPING;
//                }
//                
//                // TODO tune PID controller
//                
//                break;
//                
//            // Stop the shootere motor when either the encoder has travened the desired
//            // distance or the shooterMotorLimitSwith is pressed by thhe arm.
//            case STATE_STOPPING:
//                
//                if (isStopped())
//                {
//                    currentState = STATE_RESETTING;
//                    break;
//                }
//            
//                break;
//                
//            case STATE_RESETTING:
//                
//                if (isInCockedPosition())
//                {
//                    currentState = STATE_COCKED_AND_IDLE;
//                    break;
//                }
//                
//                break;
//        }
    }
    
    public double getShooterRate()
    {
        return shooterMotorEncoder.getRate();
    }
    
    public boolean isCocked()
    {
        return currentState == STATE_COCKED_AND_IDLE && isInCockedPosition();
    }
    
    private boolean isBallChambered()
    {
        return false;
    }
    
    private boolean isTagetSpeedReached()
    {
        return false;
    }
    
    private boolean isInCockedPosition()
    {
        return false;
    }
    
    /**
     * Arm has been halted
     * @return 
     */
    private boolean isStopped()
    {
        return false;
    }
    
}
