/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.team66;


import edu.wpi.first.team66.math.Pose;
import edu.wpi.first.team66.params.RobotParams;
import edu.wpi.first.team66.params.InputParams;
import edu.wpi.first.team66.vision.VisionSystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.camera.AxisCamera;

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
     * Vision system
     */
    private final AxisCamera shooterCamera = AxisCamera.getInstance();
    private final VisionSystem visionSystem = null;
    
    /**
     * Timing system
     */
    private final DeltaTimer continuousTimer = new DeltaTimer();
    private final DeltaTimer periodicTimer = new DeltaTimer();
    
    private double continuousDeltaTime = 0.0;
    private double periodicDeltaTime = 0.0;
    
    /**
     * Called once when the robot is started.
     */
    public void robotInit() {
        continuousTimer.reset();
        periodicTimer.reset();
    }
    
    /**
     * Called every cycle of the main loop (as fast as it can spin), during
     * any mode.
     */
    public void robotContinuous() {
        continuousDeltaTime = continuousTimer.getDeltaTime();
        tankDrive.update(continuousDeltaTime);
    }
    
    public void robotPeriodic() {
        periodicDeltaTime = periodicTimer.getDeltaTime();
        // TODO
    }

    /**
     * Called whenever autonomous mode is entered, before autonomousPeriodic
     * or autonomousContinuous.
     */
    public void autonomousInit() {
        // TODO
    }
    
    /**
     * Called only when there is new data from the Driver Station (~50Hz).
     */
    public void autonomousPeriodic() {
        robotPeriodic();
        // TODO
    }

    /**
     * Called every cycle of the main loop (as fast as it can spin).
     */
    public void autonomousContinuous() {
        robotContinuous();
        // 
    }
    
    /**
     * Called whenever teleop mode is entered, before teleopPeriodic
     * or teleopContinuous.
     */
    public void teleopInit() {
        // TODO
    }
    
    /**
     * Called only when there is new data from the Driver Station (~50Hz).
     */
    public void teleopPeriodic() {
        robotPeriodic();
        
        updateTankDriveFromInput();
        
        updateShooterFromInput();
    }
    
    /**
     * Called every cycle of the main loop (as fast as it can spin).
     */
    public void teleopContinuous() {
        robotContinuous();
        // TODO
    }
    
    /**
     * Use input from user to update drive system.
     */
    private void updateTankDriveFromInput() {
        double leftSpeed = gamepad.getRawAxis(leftDriveAxisId);
        double rightSpeed = gamepad.getRawAxis(rightDriveAxisId);
        double scale = preciseModeButton.get() ? 0.25 : 1.0;
        
        tankDrive.setTarget(leftSpeed, rightSpeed, scale);
    }
    
    /**
     * Use input from user to update shooter system.
     */
    private void updateShooterFromInput() {
        if (shootButton.get() && shooter.canShoot()) {
            
            Pose targetPose = visionSystem.findTeleopTarget();
            
            double distance = targetPose.position.x;
            double height = targetPose.position.y;
            
            shooter.shoot(distance, height);
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
