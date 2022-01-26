// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase
 {
  private TalonSRX m_indexMotor1 = new TalonSRX(CANIDConstants.INDEX_MOTOR_1_ID);
  private CANSparkMax m_indexMotor2 = new CANSparkMax(CANIDConstants.INDEX_MOTOR_2_ID, MotorType.kBrushless);
  private DigitalInput m_beamBreak = new DigitalInput(IndexConstants.INDEX_BEAMBREAK);
  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() 
  {
      m_indexMotor1.setInverted(true);
      m_indexMotor2.setInverted(false);
  }
 
  public void motorOneStart()
  {
    m_indexMotor1.set(ControlMode.PercentOutput, 0.5);
  }

  public void motorOneStop()
  {
    m_indexMotor1.set(ControlMode.PercentOutput,0);
  }
  public void MotorTwoStart()
  {
    m_indexMotor2.set(0.5);
  }
  public void MotorTwoStop()
  {
    m_indexMotor2.set(0);
  }

  public boolean getBeamBreak()
  {
    return m_beamBreak.get();
  }
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
