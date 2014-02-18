package edu.wpi.first.team66.autonomous.mode1;

import edu.wpi.first.team66.Loader;
import edu.wpi.first.team66.Shooter;
import edu.wpi.first.team66.TankDrive;
import edu.wpi.first.team66.autonomous.DoNothingState;
import edu.wpi.first.team66.autonomous.State;

public class MovingState implements State {
    
    public static final State instance = new MovingState();
    
    private MovingState(){}
    
    public State update(TankDrive tankDrive, Loader loader, Shooter shooter)
    {
        if (tankDrive.moveDistanceCompleted())
        {
            return DoNothingState.instance;
        }
        else
        {
            return this;
        }
    }
}
