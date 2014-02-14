package edu.wpi.first.team66;

import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class Loader implements IOParams {
    
    private final Solenoid armRetractSolenoid;
    
    private final Solenoid armExtendSolenoid;
    
    private final SpeedController ballRollerMotor;
    
    private final DigitalInput armExtendedSwitch;
    
    private final DigitalInput armRetractedSwitch;
    
    private boolean ejecting = false;
    
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
        
        retract();
    }
    
    public boolean isEjecting() {
        return ejecting;
    }
    
    public boolean isRetracting() {
        return armRetractSolenoid.get();
    }
    
    public boolean isRetracted() {
        return armRetractedSwitch.get();
    }
    
    public boolean isExtending() {
        return armExtendSolenoid.get();
    }
    
    public boolean isExtended() {
        return armExtendedSwitch.get();
    }
    
    public double getRollorSpeed() {
        return ballRollerMotor.get();
    }
    
    public void extend() {
        armRetractSolenoid.set(false);
        armExtendSolenoid.set(true);
        ballRollerMotor.set (BALL_ROLLER_MOTOR_LOAD_ON);     // Also, turn on the motor to pickup the ball.
        ejecting = false;
    }
    
    public void retract() {
        armExtendSolenoid.set(false);
        armRetractSolenoid.set(true);
        ballRollerMotor.set (BALL_ROLLER_MOTOR_OFF);
        ejecting = false;
    }
    
    public void eject() {
        ballRollerMotor.set (BALL_ROLLER_MOTOR_EJECT_ON);
        armExtendSolenoid.set  (armRetractSolenoidRetract); // Retract the arm to get it to touch
        armRetractSolenoid.set (armExtendSolenoidRetract);  // the ball in the shooter to eject it.
        ejecting = true;
    }
    
    public void stopEjecting() {
        ballRollerMotor.set (BALL_ROLLER_MOTOR_OFF);
        ejecting = false;
    }
}
