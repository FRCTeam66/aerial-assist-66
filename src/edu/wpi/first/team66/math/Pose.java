/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.team66.math;

/**
 *
 * @author pdehaan
 */
public class Pose {
    public final Vector3 position;
    public final Quaternion rotation;
    
    public Pose(Vector3 position, Quaternion rotation) {
        this.position = position;
        this.rotation = rotation;
    }
}
