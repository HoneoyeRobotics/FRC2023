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
    //TODO: Fix these PCM IDs when they add them
        public static final int Arm_Length_Brake_On = 0;
        public static final int Arm_Length_Brake_Off = 1;
    }
    //TODO: tune these values
    public final class ArmRotate{
        public static final double deadband = 3.0;
        
        public static final double maxPosition = 60.0;
        public static final double minPosition = 0.0;
        public static final double midAPosition = 45;
        public static final double midBPosition = 30;

        
        public static final double RotateKp = 0.2;

    }

    //TODO: tune these values
    public final class ArmLength{
        public static final double deadband = 15.0;
        
        public static final double maxPosition = 400.0;
        public static final double minPosition = 0.0;
        public static final double midAPosition = 300;
        public static final double midBPosition = 200;
    }
}
