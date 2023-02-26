// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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


        public static final int leftFingerMotor = 10;
        public static final int rightFingerMotor = 11;

        public static final int BottomPickupMotor = 12;
        
        public static final int ArmRotateMotor = 24;
        public static final int ArmLengthMotor = 23;

        public static final int PCM = 10;
        
    }

    public final class PCMIDs{
        public static final int Claw_Forward = 4;
        public static final int Claw_Reverse = 5;

        public static final int Arm_Rotate_Brake_On = 6;
        public static final int Arm_Rotate_Brake_Off = 7;
        public static final int Arm_Length_Brake_On = 0;
        public static final int Arm_Length_Brake_Off = 1;
    }
    //TODO: tune these values
    public final class ArmRotate{
        public static final double deadband = 2.0;
        
        public static final double maxPosition = 80.0;
        public static final double minPosition = 0.0;

        
        public static final double RotateKp = 0.08;

    }

    //TODO: tune these values
    public final class ArmLength{
        public static final int armLengthLimitSwitch = 0;
        public static final int armLenghtCurrentOverload = 30;
        public static final int armLengthTempOverload = 30;

        public static final double deadband = 2.0;
        
        public static final double maxPosition = 68.0;
        public static final double minPosition = 0.0;
    }

    public final class GrabPositions{
        public static final double cubeLength = 0;
        public static final double cubeHeight = 5;
        public static final double coneUpLength = 0;
        public static final double coneUpHeight = 5;
        public static final double coneOutLength = 0;
        public static final double coneOutHeight = 5;
        public static final double coneInLength = 0;
        public static final double coneInHeight = 5;
    }

    public final class ScorePositions{
        public static final double lowLength = 0;
        public static final double lowHeight = 18;
        public static final double cubeMedLength = 19;
        public static final double cubeMedHeight = 43;
        public static final double cubeHighLength = 67;
        public static final double cubeHighHeight = 53;
        public static final double coneMedLength = 8.5;
        public static final double coneMedHeight = 53;
        public static final double coneHighLength = 52;
        public static final double coneHighHeight = 61;
    }
}
