// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.DigitalConstants;

public class IntakeSubSystem extends SubsystemBase {
  private final int SOLENOID_INTAKE_FORWARD = 4;
  private final int SOLENOID_INTAKE_REVERSE = 5;
  private final double INTAKE_MOTOR_POWER = 1;
  private final double PRESSURE_SENSOR_INPUT_VOLTAGE = 4.94;

  private TalonSRX m_IntakeFeedMotor = new TalonSRX(CANIDConstants.MOTOR_CONTROLLER_INTAKE_FEED);
  private DoubleSolenoid m_IntakeSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, SOLENOID_INTAKE_FORWARD, SOLENOID_INTAKE_REVERSE);
  private AnalogInput m_PressureSensor = new AnalogInput(DigitalConstants.PRESSURE_SENSOR);

  public IntakeSubSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDown() {
    m_IntakeSolenoid.set(Value.kForward);
  }

  public void setUp() {
    m_IntakeSolenoid.set(Value.kReverse);
  }

  public void setIntakeOn() {
    m_IntakeFeedMotor.set(ControlMode.PercentOutput, -INTAKE_MOTOR_POWER);
  }

  public void setIntakeReverse() {
    m_IntakeFeedMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER);
  }

  public void setIntakeOff() {
    m_IntakeFeedMotor.set(ControlMode.PercentOutput, 0);
  }

  private double getPressureSensorOutputVoltage() {
    return m_PressureSensor.getVoltage();
  }

  public double getPressure() {
    return 250 * (getPressureSensorOutputVoltage() / PRESSURE_SENSOR_INPUT_VOLTAGE) - 25;
  }
}
