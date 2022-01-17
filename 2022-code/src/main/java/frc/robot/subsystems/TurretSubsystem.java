// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private TalonSRX m_turretMotor = new TalonSRX(Constants.CANIDConstants.TURRET_MOTOR_ID);
  private double m_currentAngle;
  private double m_initialAngle;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_initialAngle = m_currentAngle;
  }

  public void turnClockwise(){
    m_turretMotor.set(ControlMode.PercentOutput, 1);
  }

  public void turnCounterClockwise(){
    m_turretMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stop(){
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public double turretLimit(double positiveAngleLimit, double angle) {
    double limitedAngle = angle;
    if (angle > positiveAngleLimit) {
      limitedAngle = positiveAngleLimit;
    }
    else if (angle < -positiveAngleLimit) {
      limitedAngle = -positiveAngleLimit;
    }
    return limitedAngle;
  }

  public double getCurrentAngle() {
    return m_currentAngle;
  }

  public void setAngle(double setpointAngle) {
    double limitedAngle = turretLimit(Constants.TurretConstants.TURRET_LIMIT_ANGLE, setpointAngle);
    double differenceAngle = getCurrentAngle() - limitedAngle;
    m_turretMotor.set(ControlMode.PercentOutput, Constants.TurretConstants.TURRET_KP * differenceAngle);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_currentAngle = m_turretMotor.getSelectedSensorPosition();
  }
}
