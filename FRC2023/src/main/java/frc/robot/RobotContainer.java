// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.RunBottomPickup;
import frc.robot.commands.ToggleVisionState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  public DriveTrain drivetrain;
  public Vision vision;
  public Pickup pickup;
  CommandXboxController driverJoystick = new CommandXboxController(0);

  public RobotContainer() {
    //initialize subsystems
    drivetrain = new DriveTrain();
    vision = new Vision();
    pickup = new Pickup();

    //wire default commands
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,
      () -> driverJoystick.getLeftTriggerAxis(),
      () -> driverJoystick.getRightTriggerAxis(),
      () -> driverJoystick.getLeftX(),
      () -> driverJoystick.getLeftY(),
      () -> driverJoystick.getHID().getLeftBumper(),
      () -> driverJoystick.getRightY()
      ));
    configureBindings();

  }

  private void configureBindings() {
    
    driverJoystick.rightBumper().debounce(0.1).onTrue(new ToggleVisionState(vision));
    driverJoystick.b().debounce(0.1).whileTrue(new RunBottomPickup(pickup));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
