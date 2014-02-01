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
public class Quaternion {
    
    public final double x;
    public final double y;
    public final double z;
    public final double w;
    
    public Quaternion(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }
    
    public static Quaternion fromEulerAngles(double x, double y, double z) {
        return null;
    }
    
    public static Quaternion fromRotationAroundAxis(Vector3 axis, double rotation) {
        return null;
    }
}
