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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class TankDrive implements IOParams, RobotParams {
    
    // TODO parameterize or at least put in a more convenient location
    /**
     * Current value allows a motor to do a complete reversal (-1 -> 1)
     * in 400 milliseconds
     */
    private static final double MAX_ACCELERATION = 5; // [% max speed] / s
    
    private static final double DISTANCE_ERROR_THRESHOLD = 12; // inches
    
    private final SpeedController leftMotor;
    
    private final SpeedController rightMotor;
    
    private final Solenoid highShifterSolenoid;
    
    private final Solenoid  lowShifterSolenoid;
    
    private final Encoder leftMotorEncoder;
    
    private final Encoder rightMotorEncoder;
    
    private double targetLeftSpeed = 0.0;
    private double targetRightSpeed = 0.0;
    
    // move distance state
    private boolean isMovingDistance = false;
    private double targetEncoderDistance = 0.0;
    private double distanceMaxSpeed = 0.0;
    
    
    public TankDrive(
            SpeedController leftMotor,
            SpeedController rightMotor,
            Solenoid highShifterSolenoid,
            Solenoid lowShifterSolenoid,
            Encoder leftMotorEncoder,
            Encoder rightMotorEncoder)
    {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.highShifterSolenoid = highShifterSolenoid;
        this.lowShifterSolenoid = lowShifterSolenoid;
        
        this.leftMotorEncoder = leftMotorEncoder;
        leftMotorEncoder.setDistancePerPulse(DRIVE_ENCODER_INCHES_PER_PULSE);
        leftMotorEncoder.setReverseDirection(IS_LEFT_DRIVE_REVERSED);
        leftMotorEncoder.start();
        
        this.rightMotorEncoder = rightMotorEncoder;
        rightMotorEncoder.start();
        rightMotorEncoder.setDistancePerPulse(DRIVE_ENCODER_INCHES_PER_PULSE);
        rightMotorEncoder.setReverseDirection(IS_RIGHT_DRIVE_REVERSED);
        
        TankDrive.this.shiftLowGear();
    }
    
    public void shiftLowGear()
    {
        highShifterSolenoid.set(false);
        lowShifterSolenoid.set(true);
    }
    
    public void shiftHighGear()
    {
        lowShifterSolenoid.set(false);
        highShifterSolenoid.set(true);
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
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
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
        setLeftSpeed(0.0);
        setRightSpeed(0.0);
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
        
        double currentLeftSpeed = getCommandedLeftSpeed();
        double currentRightSpeed = getCommandedRightSpeed();
        
        /**
         * Move towards target speeds by clamping to
         * currentSpeed +/- maxDelta
         */
        setLeftSpeed(Math2.clamp(
                currentLeftSpeed - maxDelta,
                currentLeftSpeed + maxDelta,
                targetLeftSpeed));
        
        setRightSpeed((IS_RIGHT_DRIVE_REVERSED ? -1 : 1) * Math2.clamp(
                currentRightSpeed - maxDelta,
                currentRightSpeed + maxDelta,
                targetRightSpeed));
    }
        
    public double getCommandedLeftSpeed()
    {
        return (IS_LEFT_DRIVE_REVERSED ? -1 : 1) * leftMotor.get();
    }
    
    public double getCommandedRightSpeed()
    {
        return (IS_RIGHT_DRIVE_REVERSED ? -1 : 1) * rightMotor.get();
    }
    
    private void setLeftSpeed(double speed)
    {
        leftMotor.set((IS_LEFT_DRIVE_REVERSED ? -1 : 1) * speed);
    }

    private void setRightSpeed(double speed)
    {
        rightMotor.set((IS_RIGHT_DRIVE_REVERSED ? -1 : 1) * speed);
    }
}
