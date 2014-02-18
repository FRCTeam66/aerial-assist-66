package edu.wpi.first.team66;

import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class Loader implements IOParams {
    
    public static final int ROLLER_IN = -1;
    public static final int ROLLER_OFF = 0;
    public static final int ROLLER_OUT = 1;
    
    //DIO pins pulled high, and switches N.O, so define switch press in 
    //terms of T & F
    private static final boolean LOADER_SW_PRESSED = false;
    
    private final Solenoid armRetractSolenoid;
    private final Solenoid armExtendSolenoid;
    
    private final SpeedController ballRollerMotor;
    
    private final DigitalInput armExtendedSwitch;
    private final DigitalInput armRetractedSwitch;
    
    private int ballRollerState = ROLLER_OFF;
    
    public Loader(
            Solenoid armRetractSolenoid,
            Solenoid armExtendSolenoid,
            SpeedController ballRollerMotor,
            DigitalInput armExtendedSwitch,
            DigitalInput armRetractedSwitch)
    {
        this.armExtendSolenoid = armExtendSolenoid;
        this.armRetractSolenoid = armRetractSolenoid;
        this.ballRollerMotor = ballRollerMotor;
        this.armExtendedSwitch = armExtendedSwitch;
        this.armRetractedSwitch = armRetractedSwitch;
        
        Loader.this.retract();
        Loader.this.setRollerState(ROLLER_OFF);
    }
    
    public boolean isRetracting() {
        return armRetractSolenoid.get();
    }
    
    public boolean isRetracted() {
        return armRetractedSwitch.get() == LOADER_SW_PRESSED ? true:false;
    }
    
    public boolean isExtending() {
        return armExtendSolenoid.get();
    }
    
    public boolean isExtended() {
        return armExtendedSwitch.get() == LOADER_SW_PRESSED ? true:false;
    }
    
    public double getRollorSpeed() {
        return ballRollerMotor.get();
    }
    
    public int getRollerState() {
        return ballRollerState;
    }
    
    public void extend() {
        armRetractSolenoid.set(false);
        armExtendSolenoid.set(true);
    }
    
    public void retract() {
        armExtendSolenoid.set(false);
        armRetractSolenoid.set(true);
    }
    
    public void setRollerState(int state) {
        ballRollerState = state;
        switch(state){
            case ROLLER_IN:
                ballRollerMotor.set(BALL_ROLLER_MOTOR_LOAD_ON);
                break;
            case ROLLER_OFF:
                ballRollerMotor.set(BALL_ROLLER_MOTOR_OFF);
                break;
            case ROLLER_OUT:
                ballRollerMotor.set(BALL_ROLLER_MOTOR_EJECT_ON);
                break;
            default:
                throw new IllegalArgumentException("Illegal Roller State Argument:" + state);
        }

    }
}
