package edu.wpi.first.team66.autonomous;

import edu.wpi.first.team66.Loader;
import edu.wpi.first.team66.Shooter;
import edu.wpi.first.team66.TankDrive;

public class DoNothingState implements State {
    public State update(TankDrive tankDrive, Loader loader, Shooter shooter)
    {
        return this;
    }
}
