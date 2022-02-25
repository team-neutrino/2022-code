// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class IntakeSubSystem extends SubsystemBase {
  /** Intake Constants */
  private final int SOLENOID_INTAKE_FORWARD = 4;

  private final int SOLENOID_INTAKE_REVERSE = 5;
  private final double INTAKE_MOTOR_POWER = .5;

  private CANSparkMax m_IntakeFeedMotor =
      new CANSparkMax(CANIDConstants.MOTOR_CONTROLLER_INTAKE_FEED, MotorType.kBrushless);
  private DoubleSolenoid m_IntakeSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, SOLENOID_INTAKE_FORWARD, SOLENOID_INTAKE_REVERSE);

  public IntakeSubSystem() {
    m_IntakeFeedMotor.setOpenLoopRampRate(.5);
  }

  public void setDown() {
    m_IntakeSolenoid.set(Value.kForward);
  }

  public void setUp() {
    m_IntakeSolenoid.set(Value.kReverse);
  }

  public void setIntakeOn() {
    m_IntakeFeedMotor.set(INTAKE_MOTOR_POWER);
  }

  public void setIntakeReverse() {
    m_IntakeFeedMotor.set(-INTAKE_MOTOR_POWER);
  }

  public void setIntakeOff() {
    m_IntakeFeedMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
