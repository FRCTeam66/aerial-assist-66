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


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends IterativeRobot {
    
    // TODO get channels
    private final int leftDriveStickChannel = 1;
    private final int rightDriveStickChannel = 2;
    
    private final int leftDriveMotorChannel = 3;
    private final int rightDriveMotorChannel = 4;
    
    private final int shooterEncoderChannelA = 5;
    private final int shooterEncoderChannelB = 6;
    private final int shooterMotorChannel = 5;
    
    // TODO get appropriate axis mapping
    private final AxisType leftDriveAxis = AxisType.kY;
    private final AxisType rightDriveAxis = AxisType.kY;
    
    private Joystick leftDriveStick;
    private Joystick rightDriveStick;
    
    private Button shootButton;
    
    private SpeedController leftDriveMotor;
    private SpeedController rightDriveMotor;
    
    private PIDController shooterPID;
    private Encoder shooterEncoder;
    private SpeedController shooterMotor;
    
    private Shooter shooter;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        leftDriveStick = new Joystick(leftDriveStickChannel);
        rightDriveStick = new Joystick(rightDriveStickChannel);
        
        shooterEncoder = new Encoder(shooterEncoderChannelA, shooterEncoderChannelB);
        
         // TODO verify we're using Victors
        leftDriveMotor = new Victor(leftDriveMotorChannel);
        rightDriveMotor = new Victor(rightDriveMotorChannel);
        shooterMotor = new Victor(shooterMotorChannel);
        
        shooter = new Shooter(shooterMotor, shooterEncoder);
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
        
        // naive tank drive - replace this
        leftDriveMotor.set(leftDriveStick.getAxis(leftDriveAxis));
        rightDriveMotor.set(rightDriveStick.getAxis(rightDriveAxis));
        
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
