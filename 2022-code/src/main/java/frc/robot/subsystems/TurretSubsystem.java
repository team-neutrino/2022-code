// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  double m_initialAngle;
  double m_currentAngle;

  private TalonSRX m_turretMotor = new TalonSRX(Constants.CANIDConstants.TURRET_MOTOR_ID);
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    // 3.88 is gear ratio conversion for 2021 robot. Probably delete once code is moved to new robot. 
    m_initialAngle = ((360.0/1024.0) * m_turretMotor.getSelectedSensorPosition()) / 3.88;
  }

  public void stop(){
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public void turnClockwise(){
    if (getCurrentAngle() >= 5){
      stop(); 
    }
    else {
    m_turretMotor.set(ControlMode.PercentOutput, Constants.TurretConstants.TURRET_MOTOR_OUTPUT);
    }
  }

  public void turnCounterClockwise(){
    if (getCurrentAngle() <= -90){
      stop();
    }
    else {
    m_turretMotor.set(ControlMode.PercentOutput, (Constants.TurretConstants.TURRET_MOTOR_OUTPUT) * -1);
    }
  }

  public double getCurrentAngle(){
    return m_currentAngle - m_initialAngle; 
  }
 
  @Override
  public void periodic() {                 
    m_currentAngle = ((360.0/1024.0) * m_turretMotor.getSelectedSensorPosition()) / 3.88;
  }
}
