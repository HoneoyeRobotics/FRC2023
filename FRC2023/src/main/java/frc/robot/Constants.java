// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class CanIDs{
    public static final int LeftFrontDrive = 25;
    public static final int LeftRearDrive = 26;
    public static final int RightFrontDrive = 21;
    public static final int RightRearDrive = 22;
  }
  
//   public final class CanIDs{
//     public static final int LeftFrontDrive = 33;
//     public static final int LeftRearDrive = 34; 
//     public static final int RightFrontDrive = 31;
//     public static final int RightRearDrive = 32; 
//   }

    public static final class DriveConstants {
        public static final double kNeoCPR_none = (42);
        public static final double kNeoCPR_10_71 = (449.82);
        public static final double kNeoCPR_5_95 = (249.9);
        public static final double kNeoCPR_8_45= (354.9);
        public static final double kNeoCPR= (kNeoCPR_none);
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
          (kWheelDiameterMeters * Math.PI) / 10.71;
        
        public static final double kTrackwidthMeters = Units.inchesToMeters(22);
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        // public static final double ksVolts = 0.16478;
        // public static final double kvVoltSecondsPerMeter = 2.2025;
        // public static final double kaVoltSecondsSquaredPerMeter = 0.16441;
        //public static final double ksVolts = 0.21952;
        //public static final double kvVoltSecondsPerMeter = 1.0472;
        //public static final double kaVoltSecondsSquaredPerMeter = 0.090742;
        // public static final double ksVolts = 0.17012;
        // public static final double kvVoltSecondsPerMeter = 2.2082;
        // public static final double kaVoltSecondsSquaredPerMeter = 0.15066;
        public static final double ksVolts = 0.12646;
        public static final double kvVoltSecondsPerMeter = 2.8058;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15692;
    
        // Example value only - as above, this must be tuned for your drive!
        //TODO: tune this value
        //public static final double kPDriveVel = 8.5; .084125;
        //public static final double kPDriveVel = 0.70643;
        //public static final double kPDriveVel = 0.21557;
        public static final double kPDriveVel = 0.075011;
        //public static final double kPDriveVel = 0.32906;
    }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }

    public static final class AprilTags {
        public static final double Tag1X = 0;
        public static final double Tag1Y = 0;
        public static final double Tag2X = 0;
        public static final double Tag2Y = 0;
        public static final double Tag3X = 0;
        public static final double Tag3Y = 0;

    }
}
