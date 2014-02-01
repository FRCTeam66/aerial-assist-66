/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author pdehaan
 */
public class DeltaTimer {
    
    double lastCallTime = -1.0;
    
    public DeltaTimer() {
    }
    
    public void reset() {
        lastCallTime = Timer.getFPGATimestamp();
    }
    
    /**
     * @return Number of seconds that have passed since the last call to
     * getDeltaTime() or reset().
     */
    public double getDeltaTime() {
        double currentCallTime = Timer.getFPGATimestamp();
        double deltaTime = currentCallTime - lastCallTime;
        lastCallTime = currentCallTime;
        return deltaTime;
    }
}
