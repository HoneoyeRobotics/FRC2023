// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

  public DriveTrain drivetrain;
  XboxController driverJoystick = new XboxController(0);

  public RobotContainer() {
    drivetrain = new DriveTrain();
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,
    () -> driverJoystick.getRightTriggerAxis(),
    () -> driverJoystick.getLeftTriggerAxis(),
    () -> driverJoystick.getLeftX()
    ));
    configureBindings();
    SmartDashboard.putString("CommandState", "Setting default command");
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    SmartDashboard.putString("CommandState", "Setting Auto");
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

     // An example trajectory to follow.  All units in meters.
     Trajectory exampleTrajectory =
     TrajectoryGenerator.generateTrajectory(
         // Start at the origin facing the +X direction
         new Pose2d(0, 0, new Rotation2d(0)),
         // Pass through these two interior waypoints, making an straight path
         List.of(new Translation2d(0, 1), new Translation2d(0, 2)),
         // End 3 meters straight ahead of where we started, facing forward
         new Pose2d(0, 3, new Rotation2d(0)),
         // Pass config
         config);
         
        RamseteCommand ramseteCommand = 
          new RamseteCommand(
            exampleTrajectory, 
            drivetrain::getPose, 
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
            new SimpleMotorFeedforward(
              DriveConstants.ksVolts,
              DriveConstants.kvVoltSecondsPerMeter,
              DriveConstants.kaVoltSecondsSquaredPerMeter), 
            DriveConstants.kDriveKinematics, 
            drivetrain::getWheelSpeeds, 
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            drivetrain::tankDriveVolts, 
            drivetrain);

    SmartDashboard.putString("Command Select", "Autonomus");
          // Reset odometry to the starting pose of the trajectory.
     drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
 
     // Run path following command, then stop at the end.
     return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
   }
}
