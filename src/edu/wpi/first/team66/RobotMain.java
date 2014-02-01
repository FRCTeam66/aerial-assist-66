// * Class T66Robot
// *
// *  TTTTT      6          6
// *    T       6          6   
// *    T      6          6    
// *    T     6          6      
// *    T    6   666    6   666 
// *    T    6     6    6     6
// *    T     66666      66666 
// *
// * Edit Date: January 24, 2014
// *

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.team66;


import edu.wpi.first.team66.params.RobotParams;
import edu.wpi.first.team66.params.InputParams;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends IterativeRobot implements RobotParams, InputParams {
    
    /**
     * Inputs
     */
    private final Joystick gamepad = new Joystick(gamepadPort);
    
    private final Button shootButton = new JoystickButton(gamepad, shootButtonId);
    
    private final Button preciseModeButton = new JoystickButton(gamepad, preciseModeButtonId);
    
    /**
     * Drive system
     */
    private final SpeedController leftDriveMotor = new Victor(leftDriveMotorChannel);
    
    private final SpeedController rightDriveMotor = new Victor(rightDriveMotorChannel);
    
    private final TankDrive tankDrive = new TankDrive(leftDriveMotor, rightDriveMotor);
    
    /**
     * Shooter system
     */
    private final Encoder shooterEncoder = new Encoder(shooterEncoderChannelA, shooterEncoderChannelB);;
    
    private final SpeedController shooterMotor = new Victor(shooterMotorChannel);
    
    private final Shooter shooter = new Shooter(shooterMotor, shooterEncoder);
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        // TODO
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        
        updateTankDriveFromInput();
        
        updateShooterFromInput();
    }
    
    public void updateTankDriveFromInput() {
        double leftSpeed = gamepad.getRawAxis(leftDriveAxisId);
        double rightSpeed = gamepad.getRawAxis(rightDriveAxisId);
        double scale = preciseModeButton.get() ? 0.25 : 1.0;
        
        tankDrive.set(leftSpeed, rightSpeed, scale);
    }
    
    public void updateShooterFromInput() {
        if (shootButton.get() && shooter.canShoot()) {
            // TODO compute target relative position (vision task or should
            // we have a default/manual value?)
            double distance = 5;
            double height = 2;
            shooter.shoot(distance, height);
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
