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

        public static final int BottomPickupMotor = 23;
        
        public static final int ArmRotateMotor = 0;
        public static final int ArmLiftMotor = 0;
        public static final int ArmLengthMotor = 0;

        public static final int PCM = 7;
        
    }

    public final class PCMIDs{
        public static final int Claw_Forward = 1;
        public static final int Claw_Reverse = 2;
    }
}
