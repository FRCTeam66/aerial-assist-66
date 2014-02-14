/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66;

import edu.wpi.first.team66.math.Math2;
import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.team66.params.RobotParams;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class TankDrive implements IOParams, RobotParams {
    
    // TODO parameterize or at least put in a more convenient location
    // TODO reverse right motor?
    /**
     * Current value allows a motor to do a complete reversal (-1 -> 1)
     * in 400 milliseconds
     */
    private static final double MAX_ACCELERATION = 5; // [units]/s
    
    private static final double DISTANCE_ERROR_THRESHOLD = 12; // inches
    
    private final SpeedController leftMotor;
    
    private final SpeedController rightMotor;
    
    private final Encoder leftMotorEncoder;
    
    private final Encoder rightMotorEncoder;
    
    private double targetLeftSpeed = 0.0;
    private double targetRightSpeed = 0.0;
    
    // move distance state
    private boolean isMovingDistance = false;
    private double targetEncoderDistance = 0.0;
    private double distanceMaxSpeed = 0.0;
    
    
    public TankDrive(SpeedController leftMotor, SpeedController rightMotor, Encoder leftMotorEncoder, Encoder rightMotorEncoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.leftMotorEncoder = leftMotorEncoder;
        leftMotorEncoder.setDistancePerPulse(DRIVE_ENCODER_INCHES_PER_PULSE);
        leftMotorEncoder.start();
        
        this.rightMotorEncoder = rightMotorEncoder;
        rightMotorEncoder.start();
        rightMotorEncoder.setDistancePerPulse(DRIVE_ENCODER_INCHES_PER_PULSE);
    }
    
    public double getAverageEncoderDistance()
    {
        return 0.5 * (rightMotorEncoder.getDistance() + leftMotorEncoder.getDistance());
    }
    
    public double getLeftEncoderDistance()
    {
        return leftMotorEncoder.getDistance();
    }
    
    public double getRightEncoderDistance()
    {
        return rightMotorEncoder.getDistance();
    }
    
    /**
     * 
     * @param distance inches to travel
     * @param maxSpeed maximum speed [0,1]
     */
    public void moveDistance(double distance, double maxSpeed)
    {
        double totalEncoderDistance =
                getAverageEncoderDistance();
        
        isMovingDistance = true;
        targetEncoderDistance = totalEncoderDistance + distance;
        distanceMaxSpeed = maxSpeed;
    }
    
    public boolean moveDistanceCompleted()
    {
        return isMovingDistance;
    }
    
    public void setSpeed(double leftSpeed, double rightSpeed)
    {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        setTargetSpeed(leftSpeed, rightSpeed);
    }
    
    public void setTargetSpeed(double leftSpeed, double rightSpeed) {
        isMovingDistance = false;
        targetLeftSpeed = leftSpeed;
        targetRightSpeed = rightSpeed;
    }
    
    public void stop()
    {
        isMovingDistance = false;
        leftMotor.set(0.0);
        rightMotor.set(0.0);
        targetLeftSpeed = 0.0;
        targetRightSpeed = 0.0;
    }
    
    /**
     * Sets the motor controllers closer to the set target speeds,
     * constrained somewhat to avoid exceeding our maximum acceleration.
     * @param deltaTime 
     */
    public void update(double deltaTime) {
        
        if (isMovingDistance)
        {
            double currentDistance = getAverageEncoderDistance();
            double distanceError = targetEncoderDistance - currentDistance;
            if (distanceError <= DISTANCE_ERROR_THRESHOLD)
            {
                isMovingDistance = false;
                // TODO dynamic braking power
                setTargetSpeed(0, 0);
            }
        }
        
        /**
         * maximum change in speed allowed in this update call
         */
        double maxDelta = MAX_ACCELERATION * deltaTime;
        
        double currentLeftSpeed = leftMotor.get();
        double currentRightSpeed = rightMotor.get();
        
        /**
         * Move towards target speeds by clamping to
         * currentSpeed +/- maxDelta
         */
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
