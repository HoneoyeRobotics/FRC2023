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

    public static final class DriveConstants {

        public static final double kNeoCPR_10_71 = (449.82);
        public static final double kNeoCPR_5_95 = (249.9);
        public static final double kNeoCPR_8_45= (354.9);
        public static final double kNeoCPR= (kNeoCPR_10_71);
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
          (kWheelDiameterMeters * Math.PI) / kNeoCPR;
        
        public static final double kTrackwidthMeters = Units.inchesToMeters(22);
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.16478;
        public static final double kvVoltSecondsPerMeter = 2.2025;
        public static final double kaVoltSecondsSquaredPerMeter = 0.16441;
    
        // Example value only - as above, this must be tuned for your drive!
        //TODO: tune this value
        //public static final double kPDriveVel = 8.5;
        public static final double kPDriveVel = .0013871;
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }
}
