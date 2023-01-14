// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Turret extends PIDSubsystem {

  private boolean enabled = false;
  private TalonSRX turretMotor;
  /** Creates a new Turret. */
  public Turret() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.2, 0, 0));

      turretMotor = new TalonSRX(Constants.TurretMotorPID);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("enabled", isEnabled());
    SmartDashboard.putNumber("Encoder", turretMotor.getSelectedSensorPosition() );
  
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-suits");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    SmartDashboard.putNumber("LL X", tx.getDouble(0.0));
    SmartDashboard.putNumber("LL Y", ty.getDouble(0.0));
    SmartDashboard.putNumber("LL A", ta.getDouble(0.0));
  }

  public void move(double speed){
    if(enabled == false)
      turretMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getEnabled(){
    return enabled;
  }
  public void toggleEnabled(){
    
    if(enabled == false){ 
      enable();
      enabled = true;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

    }
    else{
    disable();
    enabled = false;
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight-suits").getEntry("ledMode").setNumber(1);
    }
    
  }
  @Override
  public void useOutput(double output, double setpoint) {
    // change deadband
    if(enabled = false)
      output = 0;
    if(output > 0.5)
      output = 0.5;
    if (output < -0.5)
      output = -0.5;
    turretMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here      
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-suits");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    SmartDashboard.putNumber("LL X", tx.getDouble(0.0));
    SmartDashboard.putNumber("LL Y", ty.getDouble(0.0));
    SmartDashboard.putNumber("LL A", ta.getDouble(0.0));

    return tx.getDouble(0.0);
  }
}
