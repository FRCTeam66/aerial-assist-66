/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66;

import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author pdehaan
 */
public class TankDrive {
    
    private final SpeedController leftMotor;
    private final SpeedController rightMotor;
    
    public TankDrive(SpeedController leftMotor, SpeedController rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }
    
    public void set(double leftSpeed, double rightSpeed) {
        set(leftSpeed, rightSpeed, 1.0);
    }
    
    public void set(double leftSpeed, double rightSpeed, double scale) {
        leftMotor.set(leftSpeed * scale);
        rightMotor.set(rightSpeed * scale);
    }
    
    public void arc(double radius, double speed) {
        
    }
}
