// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

  public DriveTrain drivetrain;
  Joystick driverJoystick = new Joystick(0);

  public RobotContainer() {
    drivetrain = new DriveTrain();
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,
    () -> driverJoystick.getRawAxis(Constants.AXIS_RightTrigger),
    () -> driverJoystick.getRawAxis(Constants.AXIS_LeftTrigger),
    () -> driverJoystick.getRawAxis(Constants.AXIS_LeftStickX)
    ));
    configureBindings();


  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
