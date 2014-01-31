/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Interface to the shooting mechanism, maintains state to keep
 * system internally consistent and automatically reset after a shot.
 */
public class Shooter {
    
    private final PIDController pidController;
    private static final int READY = 0, SHOOTING = 1, RESETTING = 2;
    private int currentState;
    
    public Shooter(SpeedController motor, Encoder encoder) {
        // TODO compute initial parameters
        this.currentState = RESETTING;
        this.pidController = new PIDController(0, 1, 1, encoder,
                new TargetReachedListenerProxy(motor));
        
    }
    
    public boolean canShoot() {
        return currentState == READY;
    }
    
    public void shoot(double distance, double peak) {
        if (currentState != READY) {
            return;
        }
        
        // TODO compute fire parameters
        /**
         * If a pidWrite call is made before we set state to
         * "SHOOTING" it shouldn't break (because we only do
         * this in the "READY" state, which has no other transition)
         */
        pidController.setPID(1, 0.5, -0.5);
        currentState = SHOOTING;
    }
    
    /**
     * Proxies the normal PIDOutput in order to check if the target position
     * is reached, if so change state and alter PID values.
     */
    private class TargetReachedListenerProxy implements PIDOutput {
        
        private final PIDOutput proxied;
        
        public TargetReachedListenerProxy(PIDOutput proxied) {
            this.proxied = proxied;
        }

        public void pidWrite(double output) {
            if (pidController.onTarget()) {
                
                /**
                 * This is depending on the fact that a PIDController
                 * is running a single thread, otherwise we could set the
                 * state and get another pidWrite call before we've changed
                 * the parameters (e.g. we say we're resetting, and the state
                 * gets checked and found on target before we've told the
                 * controller about a new target)
                 */
                switch (currentState) {
                    case RESETTING:
                        // reset, maintain position
                        currentState = READY;
                    case SHOOTING:
                        // shot fired, reset
                        currentState = RESETTING;
                        // TODO compute reset parameters
                        pidController.setPID(0, 0, 0);
                    default:
                }
            } else {
                proxied.pidWrite(output);
            }
        }
    }
}
