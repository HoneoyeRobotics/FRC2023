// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.Pickup;

public class RunBottomPickup extends CommandBase {
  /** Creates a new RunBottomPickup. */
  private Pickup pickup;
  public RunBottomPickup(Pickup pickup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pickup = pickup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pickup.runBottomPickupMotor(RobotPrefs.getBottomPickupSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pickup.runBottomPickupMotor(01);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
