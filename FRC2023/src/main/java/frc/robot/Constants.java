// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    private static final double shift = Units.inchesToMeters(22);
    public static final int proxSensorID = 0;
    public static final double txdeadband = 1;
    public static final double yPosDeadband = .04;

    public static final double[] AprilTagYcoordinates = {0, 
        Units.inchesToMeters(174.19), Units.inchesToMeters(108.19),
        Units.inchesToMeters(41.19), Units.inchesToMeters(265.74), 
        Units.inchesToMeters(265.74), Units.inchesToMeters(174.19), 
        Units.inchesToMeters(108.19), Units.inchesToMeters(41.19)
    };

    public static final double[] BluetargetYcoordinates = {0,
        AprilTagYcoordinates[8] - shift, AprilTagYcoordinates[8], AprilTagYcoordinates[8] + shift, 
        AprilTagYcoordinates[7] - shift, AprilTagYcoordinates[7], AprilTagYcoordinates[7] + shift,
        AprilTagYcoordinates[6] - shift, AprilTagYcoordinates[6], AprilTagYcoordinates[6] + shift
    };

    public final class Drive {
        public static final double deadband = .05;
        public static final double ramprate = .05;
    }

    public final class CanIDs {
        public static final int LeftFrontDrive = 25;
        public static final int LeftRearDrive = 26;
        public static final int RightFrontDrive = 21;
        public static final int RightRearDrive = 22;

        public static final int leftFingerMotor = 10;
        public static final int rightFingerMotor = 11;

        public static final int BottomPickupMotor = 12;

        public static final int ArmRotateMotor = 24;
        public static final int ArmLengthMotor = 23;

        public static final int PCM = 10;
    }

}