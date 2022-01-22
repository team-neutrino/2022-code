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
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
  }

  public void stop(){
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public void turnClockwise(){
    m_turretMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void turnCounterClockwise(){
    m_turretMotor.set(ControlMode.PercentOutput, -0.5);
  }
 
  @Override
  public void periodic() {                 
  
  }
}
