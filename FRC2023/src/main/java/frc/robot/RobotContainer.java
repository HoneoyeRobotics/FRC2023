// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  public DriveTrain drivetrain;
  CommandXboxController driverJoystick = new CommandXboxController(0);

  public RobotContainer() {
    drivetrain = new DriveTrain();
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,
      () -> driverJoystick.getRightTriggerAxis(),
      () -> driverJoystick.getLeftTriggerAxis(),
      () -> driverJoystick.getLeftX()
    ));
    
    configureBindings();
  }

  // public Command pathPlannerToRamsete(String filename) {
  //   Trajectory myPathPlannerTrajectory;
  //   try {
  //     Path myPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
  //     myPathPlannerTrajectory = TrajectoryUtil.fromPathweaverJson(myPath);

  //   } catch (IOException exception) {
  //     DriverStation.reportError("Error making the following file a path " + filename, exception.getStackTrace());
  //     System.out.println("Unable to use " + filename);
  //     return new InstantCommand();
  //   }
  //   RamseteCommand ramseteCommand = 
  //   new RamseteCommand(
  //     myPathPlannerTrajectory, 
  //     drivetrain::getPose, 
  //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
  //     new SimpleMotorFeedforward(
  //       DriveConstants.ksVolts,
  //       DriveConstants.kvVoltSecondsPerMeter,
  //       DriveConstants.kaVoltSecondsSquaredPerMeter), 
  //     DriveConstants.kDriveKinematics, 
  //     drivetrain::getWheelSpeeds, 
  //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //     drivetrain::tankDriveVolts, 
  //     drivetrain);

  //   return ramseteCommand;
  // }

  private void configureBindings() {
    SmartDashboard.putData("coastmode", new coastMode(drivetrain));
    SmartDashboard.putData("ResetEncoders", new ResetEncoders(drivetrain));
  }

  public Command getAutonomousCommand() {

    int AprilTagNumber = 1;
    Pose2d endpoint;

    double distanceToTag = 1;
    double angleToTag = 1;
    double currentXPosition = 1;
    double currentYPosition = 1;
    double currentAngle = 1;

    Trajectory myTrajectory;
    int trajectoryChoice;
    Path jsonPath;

    drivetrain.zeroHeading();
    drivetrain.resetEncoders();
    drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

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

        
    //Get the trajectory choice from the dashboard for the switch statement (If there is no key there it creates one)
    if(!Preferences.containsKey("TrajectoryChoice")){
        Preferences.setInt("TrajectoryChoice", 1);
    }
    trajectoryChoice = Preferences.getInt("TrajectoryChoice", 1);
  
    // Create a switch statement to switch between multiple trajectories
    switch (trajectoryChoice) {
      case 1:
        // An example trajectory to follow.  All units in meters. Goes Straight 3 meters
        myTrajectory =
        TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an straight path
          List.of(new Translation2d(1, 0), new Translation2d(2.9, 0)),
          // End 3 meters straight ahead of where we started, facing 0 degrees
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config);
        break;
      case 2:
        // A test trajectory that does some sort of curve
        myTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an curved path
            List.of(new Translation2d(1, 1), new Translation2d(2, 2)),
            // End 4.2 meters diagonally of where we started, facing 90 degrees
            new Pose2d(3, 3, new Rotation2d(90)),
            // Pass config
            config);
        break;
      case 3:
        // A test trajectory that does a "S" curve
        myTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an "S" curve
            List.of(new Translation2d(2, 1), new Translation2d(2, 3)),
            // End 5.7 meters diagonally of where we started, facing 0 degrees
            new Pose2d(4, 4, new Rotation2d(0)),
            // Pass config
            config);
        break;
      case 4:
        // A test trajectory that does a circular curve
        myTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an Circular curve
            List.of(new Translation2d(2.24, 1), new Translation2d(2.83, 2)),
            // End 5.7 meters diagonally of where we started, facing 0 degrees
            new Pose2d(3, 3, new Rotation2d(90)),
            // Pass config
            config);
        break;
      case 5:
        jsonPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/Test1.wpilib.json");
        //System.out.println("Path: " + jsonPath);
        try{
        myTrajectory = TrajectoryUtil.fromPathweaverJson(jsonPath);
        } catch (IOException exception) {
          DriverStation.reportError("Unable to use file" + jsonPath, exception.getStackTrace());
          System.out.println("Unable to use file");

          myTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(.33, 0), new Translation2d(.67, 0)),
            new Pose2d(1, 0, new Rotation2d(0)),
            config);
        }

        break;
      case 6:
        jsonPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/Test5.wpilib.json");
        //System.out.println("Path: " + jsonPath);
        try{
        myTrajectory = TrajectoryUtil.fromPathweaverJson(jsonPath);
        } catch (IOException exception) {
          DriverStation.reportError("Unable to use file" + jsonPath, exception.getStackTrace());
          System.out.println("Unable to use file");

          myTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(.33, 0), new Translation2d(.67, 0)),
            new Pose2d(1, 0, new Rotation2d(0)),
            config);
        }

        break;
      case 7:
        currentAngle = 180 - angleToTag;
        switch(AprilTagNumber) {
          case 1:
            if(angleToTag > 0) {
              currentXPosition = AprilTags.Tag1X + (Math.sin(angleToTag) * distanceToTag);
              currentYPosition = AprilTags.Tag1Y - (Math.cos(angleToTag) * distanceToTag);
            }
            else {
              currentXPosition = AprilTags.Tag1X + (Math.sin(Math.abs(angleToTag)) * distanceToTag);
              currentYPosition = AprilTags.Tag1Y + (Math.cos(Math.abs(angleToTag)) * distanceToTag);
            }
            endpoint = new Pose2d(AprilTags.Tag1X, AprilTags.Tag1Y, new Rotation2d(180));
            break;

          case 2:
            if(angleToTag > 0) {
              currentXPosition = AprilTags.Tag2X + (Math.sin(angleToTag) * distanceToTag);
              currentYPosition = AprilTags.Tag2Y - (Math.cos(angleToTag) * distanceToTag);
            }
            else {
              currentXPosition = AprilTags.Tag2X + (Math.sin(Math.abs(angleToTag)) * distanceToTag);
              currentYPosition = AprilTags.Tag2Y + (Math.cos(Math.abs(angleToTag)) * distanceToTag);
            }
            endpoint = new Pose2d(AprilTags.Tag2X, AprilTags.Tag2Y, new Rotation2d(180));
            break;

          case 3:
            if(angleToTag > 0) {
              currentXPosition = AprilTags.Tag3X + (Math.sin(angleToTag) * distanceToTag);
              currentYPosition = AprilTags.Tag3Y - (Math.cos(angleToTag) * distanceToTag);
            }
            else {
              currentXPosition = AprilTags.Tag3X + (Math.sin(Math.abs(angleToTag)) * distanceToTag);
              currentYPosition = AprilTags.Tag3Y + (Math.cos(Math.abs(angleToTag)) * distanceToTag);
            }
              endpoint = new Pose2d(AprilTags.Tag3X, AprilTags.Tag3Y, new Rotation2d(180));
            break;
          default:
            endpoint = new Pose2d(1,1, new Rotation2d(180));
            break;
              
        }

        myTrajectory = 
        TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(currentXPosition, currentYPosition, new Rotation2d(currentAngle)), endpoint), 
          config);
        break;

      case 8:
        myTrajectory = 
        TrajectoryGenerator.generateTrajectory(
          //A trajectory that only uses Pose2d objects and goes straight 3 meters
          List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(4.5, 0, new Rotation2d(0))), 
          config);
        break;

      default:
        // A safe trajectory of 1 meter straight ahead.
        myTrajectory =
        TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an straight path
          List.of(new Translation2d(.33, 0), new Translation2d(.67, 0)),
          // End 1 meters straight ahead of where we started, facing 0 degrees
          new Pose2d(1, 0, new Rotation2d(0)),
          // Pass config
          config);
        break;
    }

    RamseteCommand ramseteCommand = 
      new RamseteCommand(
        myTrajectory, 
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
    
    // Reset odometry to the starting pose of the trajectory.
    //drivetrain.resetOdometry(myTrajectory.getInitialPose());
 
     // Run path following command, then stop at the end.
     return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
   
  }
}
