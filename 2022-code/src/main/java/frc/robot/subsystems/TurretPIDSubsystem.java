// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretPIDSubsystem extends SubsystemBase {
  private TalonSRXConfiguration m_turretMotorConfig = new TalonSRXConfiguration();
  private TalonSRX m_turretMotor = new TalonSRX(Constants.CANIDConstants.TURRET_MOTOR_ID);
  private double m_currentAngle;
  private double FORWARD_SOFT_LIMIT_THRESHOLD = 600;
  private double REVERSE_SOFT_LIMIT_THRESHOLD = 200;
  private double TURRET_MOTOR_OUTPUT = 0.5;

  /** Creates a new TurretPIDSubsystem. */
  public TurretPIDSubsystem() {
    m_turretMotorConfig.slot0.kP = 1.5;
    m_turretMotorConfig.slot0.kD = 0;
    m_turretMotorConfig.slot0.kI = 0;
    m_turretMotorConfig.slot0.kF = 0;
    m_turretMotor.configAllSettings(m_turretMotorConfig);
    m_turretMotor.setNeutralMode(NeutralMode.Coast);
    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    // m_turretMotor.configFeedbackNotContinuous(false);
    m_turretMotor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT_THRESHOLD);
    m_turretMotor.configForwardSoftLimitEnable(true);
    m_turretMotor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT_THRESHOLD);
    m_turretMotor.configReverseSoftLimitEnable(true);
  }

  public void setTargetAngle(double targetAngle) {
    m_turretMotor.set(ControlMode.Position, targetAngle);
  }

  public double getCurrentAngle() {
    return m_currentAngle;
  }

  public void stop() {
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public void turnClockwise() {
    m_turretMotor.set(ControlMode.PercentOutput, TURRET_MOTOR_OUTPUT);
  }

  public void turnCounterClockwise() {
    m_turretMotor.set(ControlMode.PercentOutput, TURRET_MOTOR_OUTPUT * -1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getCurrentAngle());
    m_currentAngle = m_turretMotor.getSelectedSensorPosition(0);
  }

  public double getP() {
    return m_turretMotorConfig.slot0.kP;
  }

  public void setP(double P) {
    m_turretMotorConfig.slot0.kP = P;
  }

  public double getI() {
    return m_turretMotorConfig.slot0.kD;
  }

  public void setI(double I) {
    m_turretMotorConfig.slot0.kP = I;
  }

  public double getD() {
    return m_turretMotorConfig.slot0.kD;
  }

  public void setD(double D) {
    m_turretMotorConfig.slot0.kP = D;
  }
}
