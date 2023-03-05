// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ScoringHeight;

public class Arms extends SubsystemBase {
  private AnalogInput proxSensor;
  private ScoringHeight scoringHeight = ScoringHeight.Low;
  private int scoringSlot = 1;    
  private String l_position;
  
  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private NetworkTable myTable = table.getTable("Shuffleboard/Tab 2");
  //private DigitalInput proxSensor;
  /** Creates a new Arm. */
  public Arms() {
    proxSensor = new AnalogInput(Constants.proxSensorID);
    //proxSensor = new DigitalInput(Constants.proxSensorID);
  }

  public void changeScoringHeight(boolean up) {
    switch(scoringHeight) {
      case Low: 
        if (up) 
          scoringHeight = ScoringHeight.Med;
        break;
      case Med:
        if(up) 
          scoringHeight = ScoringHeight.High;
        else 
          scoringHeight = ScoringHeight.Low;
        break;
      case High:
        if(!up) 
          scoringHeight = ScoringHeight.Med;
        break;
      default:
        scoringHeight = ScoringHeight.Med;
        break;
    }
    SmartDashboard.putString("ScoringHeight", scoringHeight.toString());
    smartDashboardScorePosition();
  }

  public ScoringHeight getScoringHeight() {
    return scoringHeight;
  }

  public void changeScoringSlot(boolean right) {
    if(right) {
      if(scoringSlot == 9)
        scoringSlot = 1;
      else 
        ++scoringSlot;
    }
    else {
      if(scoringSlot == 1) 
        scoringSlot = 9; 
      else
        --scoringSlot;
    }
    SmartDashboard.putNumber("Scoring Slot", scoringSlot);
    smartDashboardScorePosition();
  }

  public int getScoringSlot() {
    return scoringSlot;
  }

  public void smartDashboardScorePosition() {
    //NetworkTableEntry myEntry = myTable.getEntry("ScorePos1High");
    //myEntry.setBoolean(true);

    if (l_position != null) {
      myTable.getEntry(l_position).setBoolean(false);
    }
    l_position = String.format("ScorePos%d%s", scoringSlot, scoringHeight.toString());
      myTable.getEntry(l_position).setBoolean(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ProxSensorAnalog", proxSensor.getVoltage());
    //SmartDashboard.putBoolean("ProxSensor", proxSensor.get());
  }
}
