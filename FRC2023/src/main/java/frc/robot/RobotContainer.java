// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BrakeMode;
import frc.robot.commands.DriveUntilPerpendicular;
import frc.robot.commands.MoveToScorePos;
import frc.robot.commands.ResetNavX;
import frc.robot.commands.RotateToDegree;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
public class RobotContainer {
  private DriveTrain driveTrain;
  private Arms arms;
  private Vision vision;
  CommandXboxController driverJoystick = new CommandXboxController(0);

  public RobotContainer() {
    driveTrain = new DriveTrain();
    arms = new Arms();
    vision = new Vision();
    //default Teleop command
    driveTrain.setDefaultCommand(new TeleopDrive(driveTrain, 
    () -> driverJoystick.getLeftTriggerAxis(), 
    () -> driverJoystick.getRightTriggerAxis(), 
    () -> driverJoystick.getLeftX(), 
    () -> driverJoystick.getHID().getLeftBumper()));

    configureBindings();
    SmartDashboard.putData("ResetNavX", new ResetNavX(driveTrain));
  }

  private void configureBindings() {
    driverJoystick.a().onTrue(new DriveUntilPerpendicular(driveTrain, vision));
    driverJoystick.rightBumper().whileTrue(new BrakeMode(driveTrain, false));
    driverJoystick.b().onTrue(new RotateToDegree(driveTrain, 0));
    driverJoystick.start().onTrue(new MoveToScorePos(driveTrain, vision));
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
