package edu.wpi.first.team66.autonomous.mode1;

import edu.wpi.first.team66.Loader;
import edu.wpi.first.team66.Shooter;
import edu.wpi.first.team66.TankDrive;
import edu.wpi.first.team66.autonomous.State;

public class Mode1InitState implements State {
    
    public static final State instance = new Mode1InitState();
    
    private Mode1InitState(){}
    
    public State update(TankDrive tankDrive, Loader loader, Shooter shooter) {
        tankDrive.moveDistance(60 /* inches */, 0.5 /* % max speed */);
        return MovingState.instance;
    }
    
}
