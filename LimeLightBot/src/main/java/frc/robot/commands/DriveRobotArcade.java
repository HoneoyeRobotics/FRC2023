// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveRobotArcade extends CommandBase {

  private DriveTrain drivetrain;
  private DoubleSupplier forwardSupplier;
  private DoubleSupplier backwardSupplier;
  private DoubleSupplier turnSupplier;

  /** Creates a new DriveRobot. */
  public DriveRobotArcade(DriveTrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier backwardSupplier,
      DoubleSupplier turnSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.forwardSupplier = forwardSupplier;
    this.backwardSupplier = backwardSupplier;
    this.turnSupplier = turnSupplier;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = 0;
    double zRotation = 0;
    double tempZ = turnSupplier.getAsDouble();

    xSpeed = forwardSupplier.getAsDouble() - backwardSupplier.getAsDouble();
    tempZ = tempZ * .8;
    zRotation = tempZ * tempZ * ((tempZ < 0) ? -1 : 1);
    
    // if(drivetrain.getReverse() == true)
    // xSpeed = xSpeed * -1;
    //not sure why it needs to be inverted
    drivetrain.drive(xSpeed, zRotation); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
