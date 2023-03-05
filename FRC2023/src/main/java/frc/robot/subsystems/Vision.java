// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.LimeLightState;

public class Vision extends SubsystemBase {
  
  private NetworkTable side_limelight = NetworkTableInstance.getDefault().getTable("limelight-suitsx");
  private NetworkTableEntry side_tx = side_limelight.getEntry("tx");
  
  private NetworkTable main_limelight = NetworkTableInstance.getDefault().getTable("limelight-suits");
  private NetworkTableEntry main_tx = main_limelight.getEntry("tx");
  private NetworkTableEntry main_tz = main_limelight.getEntry("camerapose_targetspace");

  private NetworkTableEntry m_aprilTagID = side_limelight.getEntry("tid");
  
  /** Creates a new Vision. */
  public Vision() {
    setFrontLimelightState(LimeLightState.Drive);

  }

  public boolean isPerpendicular(int aprilTagID) {
    int currentAprilTagID = (int)(m_aprilTagID.getDouble(0));
    double target = -5;
    double current = side_tx.getDouble(10);
    if(currentAprilTagID == aprilTagID && 
    (current <= Constants.txdeadband + target && current >= (Constants.txdeadband * -1) + target))
      return true;
    else 
      return false;
  }

  public boolean closeToPerpendicular(int aprilTagID) {    
    int currentAprilTagID = (int)(m_aprilTagID.getDouble(0));
    double target = -15;
    double current = side_tx.getDouble(10);
    if(currentAprilTagID == aprilTagID && (current >= target))
      return true;
    else 
      return false;
    }

  public boolean correctDistance() {
    double target = -1.5;
    double[] coordinates = main_tz.getDoubleArray(new double[] {});
    if (coordinates.length < 3 || coordinates[2] == 0.0)
      return false;
    if(coordinates[2] >= target)
      return true;
    else 
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("AtSeven?", isPerpendicular(7));
    SmartDashboard.putBoolean("CorrectDistance", correctDistance());
  }

  public void setFrontLimelightState(LimeLightState state){
    switch (state){
      case Drive:    
        main_limelight.getEntry("pipeline").setDouble(1);
        main_limelight.getEntry("camMode").setDouble(1);
        break;
      case AprilTag:          
        main_limelight.getEntry("pipeline").setDouble(5);
        main_limelight.getEntry("camMode").setDouble(0);
        break;
      case Reflective:      
        main_limelight.getEntry("pipeline").setDouble(1);
        main_limelight.getEntry("camMode").setDouble(0);
        break;
    }
  }
}
