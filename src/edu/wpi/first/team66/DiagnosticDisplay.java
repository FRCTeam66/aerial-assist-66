package edu.wpi.first.team66;

import edu.wpi.first.team66.params.IOParams;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;

public class DiagnosticDisplay implements IOParams {
    
    private final DriverStation ds;
    
    private final DriverStationLCD lcd;
    
    private final Joystick driveStickL;
    
    private final Joystick driveStickR;
    
    private final Joystick armStick;
    
    private final TankDrive tankDrive;
    
    private final Shooter shooter;
    
    private final Loader loader;
    
    private boolean diagOff = false;
    
    
    // Diagnostic selector is a linear variable Pot
    // If not connected an analog input port returns a vlalue of < 0
    // Take the value from .get() and divide it by 107.
    //   will be 0 - 9 as the diagnostic modes. 0 is no diagnostics.
    private final AnalogChannel diagnosticSelector;
    
    public DiagnosticDisplay(
            DriverStation driverStation,
            DriverStationLCD lcd,
            AnalogChannel diagnosticSelector,
            Joystick driveStickL,
            Joystick driveStickR,
            Joystick armStick,
            TankDrive tankDrive,
            Shooter shooter,
            Loader loader)
    {
        this.ds = driverStation;
        this.lcd = lcd;
        this.diagnosticSelector = diagnosticSelector;
        this.driveStickL = driveStickL;
        this.driveStickR = driveStickR;
        this.armStick = armStick;
        this.tankDrive = tankDrive;
        this.shooter = shooter;
        this.loader = loader;
        
        lcd.println(DriverStationLCD.Line.kUser1, 1, StringUtils.TODO_SPACES_21);
        lcd.println(DriverStationLCD.Line.kUser2, 1, StringUtils.TODO_SPACES_21);
        lcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
        lcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
        lcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);
        lcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
        lcd.updateLCD();
    }
    
    void displayDiagnostics () {
        int d;                                     // Which diagnostic to run.
        String b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11;
        String s;
        final String sP = "P";                      // Note that often used string constants are
        final String sS = "_";                      // best made finals, else the compiler may make
                                                    // each one unique wasting memory.

        // Decide which diagnostic routine to display, if any.
        //d = diagnosticSelector.getAverageValue();   // Get the POT value.
        d = diagnosticSelector.getValue();          // Get the POT value.
        d = (d < 0) ? 0 : d;                        // Sometimes it returns a negative number.
        d = d / 107;                                // Compute the diagnostic to display, should be 1 to 9.
                                                    // My POT returns values from -3 to 963. (963/9=107)
        d = (d <= 9) ? d : 9;                       // Limit the maximum value.

        switch (d) {
            // No diagnostics to display.
            // Note never run diagnostics during a competition match, it can take excess CPU time.
            case 0:
                if (diagOff == false) {             // Do this once per entry into diagnostice Off.
                    diagOff = true;                              // 123456789012345678901
                    lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
				  + " " + StringUtils.format (diagnosticSelector.getValue(),5));
                    lcd.println(DriverStationLCD.Line.kUser2, 1, "Diagnostics Off       ");
                    lcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
                    lcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
                    lcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);
                    lcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);
                } // if (diagOff != false;
                break;

            // Driver joystick left.
            case 1:
                                                             // 123456789012345678901
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Driver Joystick Left  ");

                lcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X"  + StringUtils.format(driveStickL.getX(), 2, 2) +
                    " Y" + StringUtils.format(driveStickL.getY(), 2, 2) +
                    " Z" + StringUtils.format(driveStickL.getZ(), 2, 2));

                b1 = ((driveStickL.getRawButton(1)) ? sP : sS);
                b2 = ((driveStickL.getRawButton(2)) ? sP : sS);
                b3 = ((driveStickL.getRawButton(3)) ? sP : sS);
                b4 = ((driveStickL.getRawButton(4)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T   " + b1 + " B2 " + b2 + " B3 " + b3 + " B4 " + b4);

                b5 = ((driveStickL.getRawButton(5)) ? sP : sS);
                b6 = ((driveStickL.getRawButton(6)) ? sP : sS);
                b7 = ((driveStickL.getRawButton(7)) ? sP : sS);
                b8 = ((driveStickL.getRawButton(8)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + " B6 " + b6 + " B7 " + b7 + " B8 " + b8);

                b9 = ((driveStickL.getRawButton(9)) ? sP : sS);
                b10 = ((driveStickL.getRawButton(10)) ? sP : sS);
                b11 = ((driveStickL.getRawButton(11)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + " B10 " + b10 +  " B11 " + b11);
                diagOff = false;
                break;

            // Driver joystick right.
            case 2:
                                                             // 123456789012345678901
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Driver Joystick Right ");

                lcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X"  + StringUtils.format(driveStickR.getX(), 2, 2) +
                    " Y" + StringUtils.format(driveStickR.getY(), 2, 2) +
                    " Z" + StringUtils.format(driveStickR.getZ(), 2, 2));

                b1 = ((driveStickR.getRawButton(1)) ? sP : sS);
                b2 = ((driveStickR.getRawButton(2)) ? sP : sS);
                b3 = ((driveStickR.getRawButton(3)) ? sP : sS);
                b4 = ((driveStickR.getRawButton(4)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T   " + b1 + " B2 " + b2 + " B3 " + b3 + " B4 " + b4);

                b5 = ((driveStickR.getRawButton(5)) ? sP : sS);
                b6 = ((driveStickR.getRawButton(6)) ? sP : sS);
                b7 = ((driveStickR.getRawButton(7)) ? sP : sS);
                b8 = ((driveStickR.getRawButton(8)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + " B6 " + b6 + " B7 " + b7 + " B8 " + b8 );

                b9 = ((driveStickR.getRawButton(9)) ? sP : sS);
                b10 = ((driveStickR.getRawButton(10)) ? sP : sS);
                b11 = ((driveStickR.getRawButton(11)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + " B10 " + b10 +  " B11 " + b11);
                diagOff = false;
                break;

            // Shooter joystick
            case 3:
                                                             // 123456789012345678901
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter Joystick      ");

                lcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X"  + StringUtils.format(armStick.getX(), 2, 2) +
                    " Y" + StringUtils.format(armStick.getY(), 2, 2) +
                    " Z" + StringUtils.format(armStick.getZ(), 2, 2));

                b1 = ((armStick.getRawButton(1)) ? sP : sS);
                b2 = ((armStick.getRawButton(2)) ? sP : sS);
                b3 = ((armStick.getRawButton(3)) ? sP : sS);
                b4 = ((armStick.getRawButton(4)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T   " + b1 + " B2 " + b2 + " B3 " + b3 + " B4 " + b4);

                b5 = ((armStick.getRawButton(5)) ? sP : sS);
                b6 = ((armStick.getRawButton(6)) ? sP : sS);
                b7 = ((armStick.getRawButton(7)) ? sP : sS);
                b8 = ((armStick.getRawButton(8)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + " B6 " + b6 + " B7 " + b7 + " B8 " + b8);

                b9 = ((armStick.getRawButton(9)) ? sP : sS);
                b10 = ((armStick.getRawButton(10)) ? sP : sS);
                b11 = ((armStick.getRawButton(11)) ? sP : sS);
                lcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + " B10 " + b10 + " B11 " + b11);
                diagOff = false;
                break;

            // Shooter Diagnostics.
            case 4:
                                                             // 123456789012345678901
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter        " + StringUtils.format(0, 3, 2));
                lcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "Ball Loaded " + (shooter.isBallInShooter() ? "Y " : "N "));
                s = "Ckd " + (shooter.isCocked() ? "Y " : "N ") +
                    "Mn/Mx " + StringUtils.format(ARM_POSITION_MIN, 4) + "/" + StringUtils.format(ARM_POSITION_MAX, 4);
                lcd.println(DriverStationLCD.Line.kUser4, 1, s);
                lcd.println(DriverStationLCD.Line.kUser5, 1,
                   "Arm Rate    " + StringUtils.format(1 /* todo */, 3) + "     ");
                lcd.println(DriverStationLCD.Line.kUser6, 1, ""); // Use text from shoot function.
                diagOff = false;
                break;

            // Shooter Diagnostics 2.
            case 5:
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter 2      " + StringUtils.format(0, 3, 2));
                lcd.println(DriverStationLCD.Line.kUser3, 1,
                    "Arm Ext LS " + (/* todo */ false ? "P" : "_") + "         ");
                lcd.println(DriverStationLCD.Line.kUser4, 1,
                    "Shooter Ret LS " + StringUtils.format(shooter.isInCockedPosition()));
                lcd.println(DriverStationLCD.Line.kUser5, 1, "Angle: " + StringUtils.format(shooter.getShooterAngle(),3,4));        
                lcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);
                diagOff = false;
                break;

            // Drive motor power
            case 6:
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Drive motor power    ");
                lcd.println(DriverStationLCD.Line.kUser3, 1, "Left motor:   " + StringUtils.format ( tankDrive.getCommandedLeftSpeed(), 4, 3));
                lcd.println(DriverStationLCD.Line.kUser4, 1, "Right motor:  " + StringUtils.format (tankDrive.getCommandedRightSpeed(), 4, 3));
                lcd.println(DriverStationLCD.Line.kUser5, 1, "Left:  " + StringUtils.format(tankDrive.getLeftEncoderDistance(),6, 2));
                lcd.println(DriverStationLCD.Line.kUser6, 1, "Right: " + StringUtils.format(tankDrive.getRightEncoderDistance(),6, 2));
                diagOff = false;
                break;

            // Miscellaneus...
            case 7:
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Miscellaneus...      ");
                s = "Gyro " + StringUtils.format(/* TODO */ 0, 3, 2);
                lcd.println(DriverStationLCD.Line.kUser3, 1, s);
                lcd.println(DriverStationLCD.Line.kUser4, 1, "Loader Out:  " + StringUtils.format(loader.isExtended()));
                lcd.println(DriverStationLCD.Line.kUser5, 1, "Loader In:   " + StringUtils.format(loader.isRetracted()));
                lcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
                diagOff = false;
                break;

            // Ball pickup arm
            case 8:
                                                             // 123456789012345678901
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) + " " 
                    + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, "Ball pickup arm      ");
                s =      "Ext LS "  + (loader.isExtended() ? "P" : "_");
                s = s + " Ret LS " + (loader.isRetracted() ? "P" : "_");
                lcd.println(DriverStationLCD.Line.kUser3, 1, s);
                lcd.println(DriverStationLCD.Line.kUser4, 1,
                    "Ext Sol " + (loader.isExtending() ? "N" : "F")
                 + " Ret Sol " + (loader.isRetracting() ? "N" : "F"));
                    // "N" = "oN",  "F" = "oFf".
                lcd.println(DriverStationLCD.Line.kUser5, 1,
                     "Roller Motor " + StringUtils.format(loader.getRollorSpeed(), 3,1));
                lcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
                diagOff = false;
                break;

            // TBD
            case 9:
                lcd.println(DriverStationLCD.Line.kUser1, 1, "T66 - Flyers  " + StringUtils.format (d, 2) 
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser2, 1, StringUtils.TODO_SPACES_21);
                lcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
                lcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
                lcd.println(DriverStationLCD.Line.kUser5, 1, "");
                lcd.println(DriverStationLCD.Line.kUser6, 1, "");
                diagOff = false;
                break;
                
            default:
                lcd.println(DriverStationLCD.Line.kUser1, 1, "Diag:" + StringUtils.format (d, 3) + " Bad Selection");
                lcd.println(DriverStationLCD.Line.kUser2, 1, "T66 - Flyers  " + StringUtils.format (d, 2)
                    + " " + StringUtils.format (diagnosticSelector.getValue (),5));
                lcd.println(DriverStationLCD.Line.kUser3, 1, StringUtils.TODO_SPACES_21);
                lcd.println(DriverStationLCD.Line.kUser4, 1, StringUtils.TODO_SPACES_21);
                lcd.println(DriverStationLCD.Line.kUser5, 1, StringUtils.TODO_SPACES_21);
                lcd.println(DriverStationLCD.Line.kUser6, 1, StringUtils.TODO_SPACES_21);        
                diagOff = false;
                break;
            }  //switch ()

        lcd.updateLCD();

    }
}
