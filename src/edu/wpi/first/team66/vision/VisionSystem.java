/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66.vision;

import edu.wpi.first.team66.math.Pose;

/**
 *
 * @author pdehaan
 */
public interface VisionSystem {

    /**
     * Find Pose of the target in autonomous mode, in robot-local space
     * @return
     */
    Pose findAutonomousTarget();

    /**
     * Find Pose of the target in teleop mode, in robot-local space
     * @return
     */
    Pose findTeleopTarget();
    
}
