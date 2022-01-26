// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private double m_initialAngle;
  private double m_currentAngle;
  private final double m_minAngle = 5;
  private final double m_maxAngle = -90;
  private double m_prevAngleError = 0;

  private TalonSRX m_turretMotor = new TalonSRX(Constants.CANIDConstants.TURRET_MOTOR_ID);
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    // 3.88 is gear ratio conversion for 2021 robot. Probably delete once code is moved to new robot. 
    m_initialAngle = angleConversion(m_turretMotor.getSelectedSensorPosition());
  }

  public double angleConversion(double sensorPosition){
    return (sensorPosition * (360.0/1024.0)) / 3.88;
  }

  public void stop(){
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public void turnClockwise(){
    if (getCurrentAngle() >= m_minAngle){
      stop(); 
    }
    else {
    m_turretMotor.set(ControlMode.PercentOutput, Constants.TurretConstants.TURRET_MOTOR_OUTPUT);
    }
  }

  public void turnCounterClockwise(){
    if (getCurrentAngle() <= m_maxAngle){
      stop();
    }
    else {
    m_turretMotor.set(ControlMode.PercentOutput, (Constants.TurretConstants.TURRET_MOTOR_OUTPUT) * -1);
    }
  }

  public double getCurrentAngle(){
    return m_currentAngle - m_initialAngle; 
  }

  private double PIDify(double error, double derivative, double integral) {
    double kP = Constants.TurretConstants.TURRET_KP;
    double kD = Constants.TurretConstants.TURRET_KD;
    double kI = Constants.TurretConstants.TURRET_KI;
    return kP * error + kD * derivative + kI * integral;
  }

  public void setSetpoint(double setpointAngle) {
    double currentAngleError = setpointAngle - getCurrentAngle(); 
    m_turretMotor.set(ControlMode.PercentOutput, PIDify(currentAngleError, 0, 0));  
  }
 
  @Override
  public void periodic() {                 
    m_currentAngle = angleConversion(m_turretMotor.getSelectedSensorPosition());
  }
}
