// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ToggleVisionState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  public DriveTrain drivetrain;
  public Vision vision;
  CommandXboxController driverJoystick = new CommandXboxController(0);

  public RobotContainer() {
    //initialize subsystems
    drivetrain = new DriveTrain();
    vision = new Vision();


    //wire default commands
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,
      () -> driverJoystick.getLeftTriggerAxis(),
      () -> driverJoystick.getRightTriggerAxis(),
      () -> driverJoystick.getLeftX(),
      () -> driverJoystick.getLeftY(),
      () -> driverJoystick.getHID().getLeftBumper()
      ));
    configureBindings();

  }

  private void configureBindings() {
    
    driverJoystick.rightBumper().debounce(0.1).onTrue(new ToggleVisionState(vision));
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Test", new PathConstraints(1, 0.05));

    driverJoystick.start().debounce(0.1).onTrue(drivetrain.FollowPath(examplePath, true));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
