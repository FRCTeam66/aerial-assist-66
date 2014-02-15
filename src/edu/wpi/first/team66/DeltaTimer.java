package edu.wpi.first.team66;

import edu.wpi.first.wpilibj.Timer;

/**
 * Helper class for measuring time deltas. Keeps track of when it's
 * updated so each call to getDeltaTime() reflects difference since last call.
 */
public class DeltaTimer {
    
    double lastCallTime = Timer.getFPGATimestamp();
    
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
