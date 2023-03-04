// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class BrakeMode extends CommandBase {
  private DriveTrain m_DriveTrain;
  private boolean instant;
  /** Creates a new BrakeMode. */
  public BrakeMode(DriveTrain driveTrain, boolean instant) {
    m_DriveTrain = driveTrain;
    this.instant = instant;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.setBreakMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.setCoastMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return instant;
  }
}
