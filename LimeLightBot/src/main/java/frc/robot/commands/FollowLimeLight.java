// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.tools.ForwardingFileObject;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FollowLimeLight extends CommandBase {
  /** Creates a new FollowLimeLight. */
  private DriveTrain driveTrain;
  public FollowLimeLight(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
  }

  PIDController sideController;
  PIDController forwaController;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sideController = new PIDController(0.2, 0, 0);
    forwaController = new PIDController(2, 0, 0);
    // sideController.setIntegratorRange(-0.4, 0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-suits");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  SmartDashboard.putNumber("LL X", tx.getDouble(0.0));
  double zRotation = 0;
  double place = tx.getDouble(0.0);

  if(place != 0){
   zRotation =  sideController.calculate(place, 0) * -1;
   if(zRotation > 0.5)
   zRotation = 0.5;
   else if (zRotation < -0.5)
   zRotation = -0.5;
    if(place < 3 && place > 0.3)
    zRotation = 0;
   SmartDashboard.putNumber("zRotation", zRotation);
  }
  // if(place <  3 && place )
  //   zRotation = 0.5;
  // else if (place < -3)
  //   zRotation = -0.5;
  

  double xspeed = 0;
  double area = ta.getDouble(0.5);

  double reversespot = 0.08;
  double forwardspot = 0.07;
  if(area > reversespot && area < forwardspot)
    xspeed = 0;
  else if (area > forwardspot)
    xspeed = -0.5;
  else if (area == 0)
    xspeed = 0;
    else{
      xspeed = forwaController.calculate(area, forwardspot);
      if(xspeed > 0.66)
      xspeed = 0.66;
    }  
    SmartDashboard.putNumber("xspeed", xspeed);
    driveTrain.drive(xspeed, zRotation);
SmartDashboard.putNumber("moving",zRotation);
    SmartDashboard.putNumber("LL Y", ty.getDouble(0.0));
    SmartDashboard.putNumber("LL A", ta.getDouble(0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
