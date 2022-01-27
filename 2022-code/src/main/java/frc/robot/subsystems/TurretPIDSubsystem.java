// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretPIDSubsystem extends SubsystemBase {
  private TalonSRXConfiguration m_turretMotorConfig = new TalonSRXConfiguration();
  private TalonSRX m_turretMotor = new TalonSRX(7);
  private double m_currentAngle;

  public static final double ANGLES_PER_REVOLUTION = 360.0;
  private final double RADIANS_PER_REVOLUTION = 2 * Math.PI;
  public static final double ENCODER_UNITS_PER_REVOLUTION = 1024.0;
  
  

  /** Creates a new TurretPIDSubsystem. */
  public TurretPIDSubsystem() {
    m_turretMotorConfig.slot0.kP = 0.01;
    m_turretMotorConfig.slot0.kD = 0;
    m_turretMotorConfig.slot0.kI = 0;
    m_turretMotorConfig.slot0.kF = 0;
    m_turretMotor.configAllSettings(m_turretMotorConfig);
    m_turretMotor.setNeutralMode(NeutralMode.Coast);
  }

  public static double convertAnglesToEncoderUnits(double angles) {
    return angles * ENCODER_UNITS_PER_REVOLUTION / ANGLES_PER_REVOLUTION;
  }

  private double convertEncoderUnitsToAngles(double rawUnits) {
    return rawUnits * ANGLES_PER_REVOLUTION / ENCODER_UNITS_PER_REVOLUTION;
  }

  public void setTargetAngle(double targetAngle) {
    m_turretMotor.set(ControlMode.Position, targetAngle);
  }

  public double getCurrentAngle() {
    return m_currentAngle;
  }

  public void stop(){
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public void turnClockwise(){
    if (getCurrentAngle() >= 10000){
      stop(); 
    }
    else {
    m_turretMotor.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public void turnCounterClockwise(){
    if (getCurrentAngle() <= -10000){
      stop();
    }
    else {
      m_turretMotor.set(ControlMode.PercentOutput, -0.5);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_currentAngle = m_turretMotor.getSelectedSensorPosition(1);
    /*if(Math.abs(m_currentAngle) >= 360.0) {
      m_currentAngle = m_currentAngle % 360.0;
    }*/
  }
}
