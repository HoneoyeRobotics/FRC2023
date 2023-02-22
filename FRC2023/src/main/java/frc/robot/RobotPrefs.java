
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



    public static double getArmLengthGrabCube(){
        if(!Preferences.containsKey("ArmLengthGrabCube")){
            Preferences.setDouble("ArmLengthGrabCube", 10);
        }
        return Preferences.getDouble("ArmLengthGrabCube", 10);
    }

    public static double getArmLengthGrabConePointIn(){
        if(!Preferences.containsKey("ArmLengthConePointIn")){
            Preferences.setDouble("ArmLengthGrabConePointIn", 10);
        }
        return Preferences.getDouble("ArmLengthGrabConePointIn", 10);
    }

    public static double getArmLengthGrabConePointUp(){
        if(!Preferences.containsKey("ArmLengthConePointUp")){
            Preferences.setDouble("ArmLengthGrabConePointUp", 10);
        }
        return Preferences.getDouble("ArmLengthGrabConePointUp", 10);
    }

    public static double getArmLengthGrabConePointOut(){
        if(!Preferences.containsKey("ArmLengthConePointOut")){
            Preferences.setDouble("ArmLengthGrabConePointOut", 10);
        }
        return Preferences.getDouble("ArmLengthGrabConePointOut", 10);
    }
    public static double getArmRotateGrabCube(){
        if(!Preferences.containsKey("ArmRotateGrabCube")){
            Preferences.setDouble("ArmRotateGrabCube", 10);
        }
        return Preferences.getDouble("ArmRotateGrabCube", 10);
    }

    public static double getArmRotateGrabConePointIn(){
        if(!Preferences.containsKey("ArmRotateConePointIn")){
            Preferences.setDouble("ArmRotateGrabConePointIn", 10);
        }
        return Preferences.getDouble("ArmRotateGrabConePointIn", 10);
    }

    public static double getArmRotateGrabConePointUp(){
        if(!Preferences.containsKey("ArmRotateConePointUp")){
            Preferences.setDouble("ArmRotateGrabConePointUp", 10);
        }
        return Preferences.getDouble("ArmRotateGrabConePointUp", 10);
    }

    public static double getArmRotateGrabConePointOut(){
        if(!Preferences.containsKey("ArmRotateConePointOut")){
            Preferences.setDouble("ArmRotateGrabConePointOut", 10);
        }
        return Preferences.getDouble("ArmRotateGrabConePointOut", 10);
    }

    public static double getArmLengthScoreLow(){
        if(!Preferences.containsKey("ArmLengthScoreLow")){
            Preferences.setDouble("ArmLengthScoreLow", 80);
        }
        return Preferences.getDouble("ArmLengthScoreLow", 80);
    }
    public static double getArmRotateScoreLow(){
        if(!Preferences.containsKey("ArmRotateScoreLow")){
            Preferences.setDouble("ArmRotateScoreLow", 80);
        }
        return Preferences.getDouble("ArmRotateScoreLow", 80);
    }
    
    public static double getArmLengthScoreConeMed(){
        if(!Preferences.containsKey("ArmLengthScoreConeMed")){
            Preferences.setDouble("ArmLengthScoreConeMed", 80);
        }
        return Preferences.getDouble("ArmLengthScoreConeMed", 80);
    }
    public static double getArmRotateScoreConeMed(){
        if(!Preferences.containsKey("ArmRotateScoreConeMed")){
            Preferences.setDouble("ArmRotateScoreConeMed", 80);
        }
        return Preferences.getDouble("ArmRotateScoreConeMed", 80);
    }

    public static double getArmLengthScoreConeHigh(){
        if(!Preferences.containsKey("ArmLengthScoreConeHigh")){
            Preferences.setDouble("ArmLengthScoreConeHigh", 80);
        }
        return Preferences.getDouble("ArmLengthScoreConeHigh", 80);
    }
    public static double getArmRotateScoreConeHigh(){
        if(!Preferences.containsKey("ArmRotateScoreConeHigh")){
            Preferences.setDouble("ArmRotateScoreConeHigh", 80);
        }
        return Preferences.getDouble("ArmRotateScoreConeHigh", 80);
    }

    public static double getArmLengthScoreCubeMed(){
        if(!Preferences.containsKey("ArmLengthScoreCubeMed")){
            Preferences.setDouble("ArmLengthScoreCubeMed", 80);
        }
        return Preferences.getDouble("ArmLengthScoreCubeMed", 80);
    }
    public static double getArmRotateScoreCubeMed(){
        if(!Preferences.containsKey("ArmRotateScoreCubeMed")){
            Preferences.setDouble("ArmRotateScoreCubeMed", 80);
        }
        return Preferences.getDouble("ArmRotateScoreCubeMed", 80);
    }

    public static double getArmLengthScoreCubeHigh(){
        if(!Preferences.containsKey("ArmLengthScoreCubeHigh")){
            Preferences.setDouble("ArmLengthScoreCubeHigh", 80);
        }
        return Preferences.getDouble("ArmLengthScoreCubeHigh", 80);
    }
    public static double getArmRotateScoreCubeHigh(){
        if(!Preferences.containsKey("ArmRotateScoreCubeHigh")){
            Preferences.setDouble("ArmRotateScoreCubeHigh", 80);
        }
        return Preferences.getDouble("ArmRotateScoreCubeHigh", 80);
    }
    public static double getArmRotatePIDMovement(){
        if(!Preferences.containsKey("getArmRotatePIDMovement")){
            Preferences.setDouble("getArmRotatePIDMovement", 1);
        }
        return Preferences.getDouble("getArmRotatePIDMovement", 1);
    }

    public static double getReflectiveTagOffset(){
        if(!Preferences.containsKey("ReflectiveTagOffset")){
            Preferences.setDouble("ReflectiveTagOffset", 1.5 );
        }
        return Preferences.getDouble("ReflectiveTagOffset", 1.5);
    }


    //TODO: Add updated default value from dashboard
    public static double getBalanceP(){
        if(!Preferences.containsKey("BalanceP")){
            Preferences.setDouble("BalanceP", 0.2 );
        }
        return Preferences.getDouble("BalanceP", 0.2);
    }

    
    public static double getBalanceI(){
        if(!Preferences.containsKey("BalanceI")){
            Preferences.setDouble("BalanceI", 0.2 );
        }
        return Preferences.getDouble("BalanceI", 0.2);
    }

    
    public static double getBalanceD(){
        if(!Preferences.containsKey("BalanceD")){
            Preferences.setDouble("BalanceD", 0.2 );
        }
        return Preferences.getDouble("BalanceD", 0.2);
    }
}
