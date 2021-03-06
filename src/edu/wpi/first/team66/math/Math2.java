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
public class Math2{
    
    private Math2() {}
    
    public static double clamp(double min, double max, double v) {
        return Math.min(max, Math.max(min, v));
    }
    
    public static double hat(double center, double width, double height, double x)
    {
        return height * Math.max(0, width - Math.abs(center - x));
    }
}
