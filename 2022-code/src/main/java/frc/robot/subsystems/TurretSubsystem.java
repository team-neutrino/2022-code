// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  double initialAngle;
  double currentAngle;

  private TalonSRX m_turretMotor = new TalonSRX(Constants.CANIDConstants.TURRET_MOTOR_ID);
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    initialAngle = ((360.0/1024.0) * m_turretMotor.getSelectedSensorPosition()) / 3.88;
  }

  public void stop(){
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public void turnClockwise(){
    if (getCurrentAngle() >= 5){
      stop(); 
    }
    else {
    m_turretMotor.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public void turnCounterClockwise(){
    if (getCurrentAngle() <= -90){
      stop();
    }
    else {
    m_turretMotor.set(ControlMode.PercentOutput, -0.5);
    }
  }

  public double getCurrentAngle(){
    return currentAngle - initialAngle; 
  }
 
  @Override
  public void periodic() {                 
    currentAngle = ((360.0/1024.0) * m_turretMotor.getSelectedSensorPosition()) / 3.88;
     System.out.println(currentAngle); 
   // System.out.println(m_turretMotor.getSelectedSensorPosition());
   System.out.println(initialAngle);
  }
}
