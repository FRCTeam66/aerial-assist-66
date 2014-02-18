package edu.wpi.first.team66.autonomous;

import edu.wpi.first.team66.Loader;
import edu.wpi.first.team66.Shooter;
import edu.wpi.first.team66.TankDrive;

public class StateMachine {
    
    private State currentState;
    
    public StateMachine()
    {
        this.currentState = DoNothingState.instance;
    }
    
    public void setState(State state)
    {
        this.currentState = currentState;
    }
    
    public void update(TankDrive tankDrive, Loader loader, Shooter shooter)
    {
        currentState = currentState.update(tankDrive, loader, shooter);
    }
}
