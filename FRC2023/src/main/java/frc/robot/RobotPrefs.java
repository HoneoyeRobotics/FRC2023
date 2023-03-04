
package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.enums.DriveControlType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotPrefs {

    public static boolean getDebugMode(){
        if(!Preferences.containsKey("DebugMode")){
            Preferences.setBoolean("DebugMode", false);            
        }
        return Preferences.getBoolean("DebugMode", false);
    }

    public static boolean getEncoderAndNavXDisplayed(){
        if(!Preferences.containsKey("DisplayEncoderAndNavX")){
            Preferences.setBoolean("DisplayEncoderAndNavX", false);            
        }
        return Preferences.getBoolean("DebugMode", false);
    }

    public static double getBottomPickupSpeed(){
        if(!Preferences.containsKey("BottomPickupSpeed")){
            Preferences.setDouble("BottomPickupSpeed", 0.5);
        }
        return Preferences.getDouble("BottomPickupSpeed", 0.5);
    }

    public static double getRotateRobotSpeed(){
        if(!Preferences.containsKey("RotateRobotSpeed")){
            Preferences.setDouble("RotateRobotSpeed", 0.1);
        }
        return Preferences.getDouble("RotateRobotSpeed", 0.1);
    }
}
