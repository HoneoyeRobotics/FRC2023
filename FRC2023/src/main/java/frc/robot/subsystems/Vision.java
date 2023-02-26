// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPrefs;
import frc.robot.enums.LimeLightState;

public class Vision extends SubsystemBase {


  private UsbCamera frontCamera;

  private AnalogInput proxSensor = new AnalogInput(0);

  private NetworkTable main_limelight = NetworkTableInstance.getDefault().getTable("limelight-suits");
  private NetworkTableEntry main_tx = main_limelight.getEntry("tx");
  private NetworkTableEntry main_ty = main_limelight.getEntry("ty");
  private NetworkTableEntry main_ta = main_limelight.getEntry("ta");

  private LimeLightState currentState = LimeLightState.ForwardDrive;
   /** Creates a new Vision. */
  public Vision() {
    setState(LimeLightState.ForwardRefletive);

    frontCamera = CameraServer.startAutomaticCapture("front", 0);
    frontCamera.setFPS(15);
  }

  public double getX(){
    return main_tx.getDouble(0);
  }

  public void toggleState(){
    switch(currentState){
      case ForwardDrive:
        setState(LimeLightState.ForwardRefletive);
        break; 
      case ForwardRefletive:
        setState(LimeLightState.ForwardApril);
        break;
      default:
      setState(LimeLightState.ForwardDrive);
      break;
    }
  }

  public void setState(LimeLightState state){
    currentState = state;
    switch(state){

      case ForwardApril:
        main_limelight.getEntry("pipeline").setNumber(2);
        
        break;
      default:
      case ForwardRefletive:
        main_limelight.getEntry("pipeline").setNumber(1 );
        break;
          }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //only show in debug mode set with the robot preferences.
    if(RobotPrefs.getDebugMode()){
      SmartDashboard.putNumber("Main LL: TX", main_tx.getDouble(0));      
      SmartDashboard.putNumber("Main LL: TY", main_ty.getDouble(0));      
      SmartDashboard.putNumber("Main LL: TA", main_ta.getDouble(0));
    }

    //SmartDashboard.putNumber("proxSensor", proxSensor.getValue());
  }
}
