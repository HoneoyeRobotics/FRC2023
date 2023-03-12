// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
  private Drivetrain m_drivetrain;
  private double xspeed;
  private double zspeed;
  private double oldspeed;
  private double oldspeedz;
  private DoubleSupplier x1;
  private DoubleSupplier x2;
  private DoubleSupplier zrotation;
  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier x1, DoubleSupplier x2, DoubleSupplier z) {
    this.x1 = x1;
    this.x2 = x2;
    zrotation = z;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    oldspeed = xspeed;
    oldspeedz = zspeed;
    zspeed = zrotation.getAsDouble();
    xspeed = x1.getAsDouble() - x2.getAsDouble();

    if (xspeed > .1)
      xspeed = (xspeed > oldspeed + Constants.ramprate ? oldspeed + Constants.ramprate : xspeed);
    if (xspeed < -.1)
      xspeed = (xspeed < oldspeed - Constants.ramprate ? oldspeed - Constants.ramprate : xspeed);

    if (zspeed > .1)
      zspeed = (zspeed > oldspeedz + Constants.ramprate ? oldspeedz + Constants.ramprate : zspeed);
    if (zspeed < -.1)
      zspeed = (zspeed < oldspeedz - Constants.ramprate ? oldspeedz - Constants.ramprate : zspeed);


    m_drivetrain.arcadeDrive(xspeed, zspeed * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
