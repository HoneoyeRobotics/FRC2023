// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class BalanceOnPlatform extends CommandBase {
  private boolean endWhenBalanced = false;
  private DriveTrain driveTrain;
  private PIDController pidController;
  /** Creates a new BalanceOnPlatform. */
  public BalanceOnPlatform(DriveTrain driveTrain, boolean endWhenBalanced) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.endWhenBalanced = endWhenBalanced;
    addRequirements(driveTrain);  
    pidController = new PIDController(2, 0, 0);    
    pidController.setTolerance(5,10);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    driveTrain.arcadeDrive(0, pidController.calculate(driveTrain.getPitch(), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endWhenBalanced && pidController.atSetpoint();
  }
}
