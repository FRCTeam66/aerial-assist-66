// * Class T66Robot
// *
// *  TTTTT      6          6
// *    T       6          6   
// *    T      6          6    
// *    T     6          6      
// *    T    6   666    6   666 
// *    T    6     6    6     6
// *    T     66666      66666 
// *
// * Edit Date: January 24, 2014
// *

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {

    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {





        // Driver joysticks.
        // If neither Joystick buttons 3 are pressed, then drive at full speed.
        // If either  Joystick buttons 3 are pressed, then drive at 3/4 speed.
        // If both    Joystick buttons 3 are pressed, then drive at 1/2 speed.
        i = (leftDriveStick.getRawButton(kSlowStickButton) ? 2 : 0) +
            (rightDriveStick.getRawButton(kSlowStickButton) ? 1 : 0);
        if (i == 0)
            d = 1.0D;
        else if ((i == 1) || (i == 2))
            d = 0.75D;
        else if (i == 3)
            d = 0.50;
        // Set the motors speed.
        // TODO TO curve the joystick either square or cube the joystick values.
        leftDriveMotor.set  (-d * leftDriveStick.getY());
        rightDriveMotor.set (d * rightDriveStick.getY());



	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
