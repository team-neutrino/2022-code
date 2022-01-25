// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.Constants.SolenoidId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class IntakeSubSystem extends SubsystemBase {
  private TalonSRX m_IntakeFeedMotor = new TalonSRX(CANIDConstants.MOTOR_CONTROLLER_INTAKE_FEED);
  private DoubleSolenoid m_IntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SolenoidId.SOLENOID_INTAKE_FORWARD, SolenoidId.SOLENOID_INTAKE_REVERSE);
  
  public IntakeSubSystem() {

  }

  public void setDown() {
    m_IntakeSolenoid.set(Value.kForward);

  }

  public void setUp(){
    m_IntakeSolenoid.set(Value.kReverse);
  
  }

  public void setIntakeOn()
  {
      m_IntakeFeedMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
  }

  public void setIntakeReverse()
  {
      m_IntakeFeedMotor.set(ControlMode.PercentOutput, -Constants.IntakeConstants.INTAKE_MOTOR_POWER);
  }

  public void setIntakeOff()
  {
      m_IntakeFeedMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
