
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

    public static DriveControlType getDriveControlType(){
        if(!Preferences.containsKey("DriveControlType")){
            Preferences.setString("DriveControlType", "SingleStick");            
        }
       String ControlType = Preferences.getString("DriveControlType", "SingleStick");

       switch (ControlType.toLowerCase()){
        case "triggersforward":
            return DriveControlType.TriggersForward;
        case "triggersturn":
            return DriveControlType.TriggersTurn;
        case "triggersturndoubleforward":        
            return DriveControlType.TriggersTurnDoubleForward;
        case "tank":
            return DriveControlType.Tank;
        default:
            return DriveControlType.SingleStick;
       }
    //SingleStick, TriggersForward, TriggersTurn
    }

    public static boolean getDebugMode(){
        if(!Preferences.containsKey("DebugMode")){
            Preferences.setBoolean("DebugMode", true);            
        }
        return Preferences.getBoolean("DebugMode", true);
    }

    public static double getBottomPickupSpeed(){
        if(!Preferences.containsKey("BottomPickupSpeed")){
            Preferences.setDouble("BottomPickupSpeed", 0.2);
        }
        return Preferences.getDouble("BottomPickupSpeed", 0.2);
    }
    public static double getArmLengthMax   (){
        if(!Preferences.containsKey("ArmLengthMax")){
            Preferences.setDouble("ArmLengthMax", 80);
        }
        return Preferences.getDouble("ArmLengthMax", 80);
    }
    public static double getArmLengthInSpeed(){
        if(!Preferences.containsKey("ArmLengthInSpeed")){
            Preferences.setDouble("ArmLengthInSpeed", 1 );
        }
        return Preferences.getDouble("ArmLengthInSpeed", 1);
    }

    public static double getArmLengthOutSpeed(){
        if(!Preferences.containsKey("ArmLengthOutSpeed")){
            Preferences.setDouble("ArmLengthOutSpeed", 0.5);
        }
        return Preferences.getDouble("ArmLengthOutSpeed", 0.5);
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

    public static double getReflectiveTagOffset(){
        if(!Preferences.containsKey("ReflectiveTagOffset")){
            Preferences.setDouble("ReflectiveTagOffset", -6 );
        }
        return Preferences.getDouble("ReflectiveTagOffset", -6);
    }
}
