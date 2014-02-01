/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66;

import edu.wpi.first.team66.math.Math2;
import edu.wpi.first.wpilibj.SpeedController;

public class TankDrive {
    
    // TODO parameterize or at least put in a more convenient location
    private static final double MAX_ACCELERATION = 1; // [units]/s
    
    private final SpeedController leftMotor;
    private final SpeedController rightMotor;
    
    private double targetLeftSpeed = 0.0;
    private double targetRightSpeed = 0.0;
    
    public TankDrive(SpeedController leftMotor, SpeedController rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }
    
    public void setTarget(double leftSpeed, double rightSpeed) {
        setTarget(leftSpeed, rightSpeed, 1.0);
    }
    
    public void setTarget(double leftSpeed, double rightSpeed, double scale) {
        leftMotor.set(leftSpeed * scale);
        rightMotor.set(rightSpeed * scale);
    }
    
    public void update(double deltaTime) {
        double maxDelta = MAX_ACCELERATION * deltaTime;
        double currentLeftSpeed = leftMotor.get();
        double currentRightSpeed = rightMotor.get();
        
        leftMotor.set(Math2.clamp(
                currentLeftSpeed - maxDelta,
                currentLeftSpeed + maxDelta,
                targetLeftSpeed));
        
        rightMotor.set(Math2.clamp(
                currentRightSpeed - maxDelta,
                currentRightSpeed + maxDelta,
                targetRightSpeed));
    }
}
