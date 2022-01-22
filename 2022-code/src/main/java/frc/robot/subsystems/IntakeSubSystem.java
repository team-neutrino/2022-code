// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.SolenoidId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase {
  private TalonSRX m_IntakeFeedMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_FEED);
  private Solenoid m_IntakePositionSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SolenoidId.SOLENOID_INTAKE_POSITION);
  public IntakeSubSystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
