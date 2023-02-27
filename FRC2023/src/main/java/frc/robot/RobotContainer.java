// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveTrain;
public class RobotContainer {
  private DriveTrain driveTrain;
  CommandXboxController driverJoystick = new CommandXboxController(0);

  public RobotContainer() {
    //default Teleop command
    driveTrain.setDefaultCommand(new TeleopDrive(driveTrain, 
    () -> driverJoystick.getLeftTriggerAxis(), 
    () -> driverJoystick.getRightTriggerAxis(), 
    () -> driverJoystick.getLeftX(), 
    () -> driverJoystick.getLeftY(), 
    () -> driverJoystick.getRightY(), 
    () -> driverJoystick.getHID().getLeftBumper()));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
