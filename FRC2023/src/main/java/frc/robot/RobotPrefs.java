
package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

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

    public static double getArmLengthInSpeed(){
        if(!Preferences.containsKey("ArmLengthInSpeed")){
            Preferences.setDouble("ArmLengthInSpeed", -.25 );
        }
        return Preferences.getDouble("ArmLengthInSpeed", -.25);
    }
    public static double getArmLengthOutSpeed(){
        if(!Preferences.containsKey("ArmLengthOutSpeed")){
            Preferences.setDouble("ArmLengthOutSpeed", 0.25);
        }
        return Preferences.getDouble("ArmLengthOutSpeed", 0.25);
    }
    
    public static double getArmRotateUpSpeed(){
        if(!Preferences.containsKey("ArmRotateUpSpeed")){
            Preferences.setDouble("ArmRotateUpSpeed", .25 );
        }
        return Preferences.getDouble("ArmRotateUpSpeed", .25);
    }
    public static double getArmRotateDownSpeed(){
        if(!Preferences.containsKey("ArmRotateDownSpeed")){
            Preferences.setDouble("ArmRotateDownSpeed", -.25 );
        }
        return Preferences.getDouble("ArmRotateDownSpeed", -.25);
    }
    public static double getArmRotatePIDMovement(){
        if(!Preferences.containsKey("getArmRotatePIDMovement")){
            Preferences.setDouble("getArmRotatePIDMovement", 1);
        }
        return Preferences.getDouble("getArmRotatePIDMovement", 1);
    }

    public static double getRotateRobotSpeed(){
        if(!Preferences.containsKey("RotateRobotSpeed")){
            Preferences.setDouble("RotateRobotSpeed", 0.1);
        }
        return Preferences.getDouble("RotateRobotSpeed", 0.1);
    }
    public static double getRotateRobotDeadband(){
        if(!Preferences.containsKey("RotateRobotDeadband")){
            Preferences.setDouble("RotateRobotDeadband", 5);
        }
        return Preferences.getDouble("RotateRobotDeadband", 5);
    }

    public static boolean isBlue(){
        if(!Preferences.containsKey("Alliance")){
            Preferences.setBoolean("Alliance", true);            
        }
        return Preferences.getBoolean("Alliance", true);
    }
}
