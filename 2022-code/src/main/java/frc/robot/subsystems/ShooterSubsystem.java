// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CalculateRPM;

public class ShooterSubsystem extends SubsystemBase {
  /** Shooter Constants */
  private final double WHEEL_P = 0.01;

  private final double WHEEL_I = 0;
  private final double WHEEL_D = 0;
  private final double WHEEL_FF = 0;

  private CANSparkMax m_wheelMotor;
  private CANSparkMax m_wheelMotor2;
  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;
  private SparkMaxPIDController m_pidController;
  private LimelightSubsystem m_limelight;

  private double m_targetRPM;

  private CalculateRPM RPMCalculator;

  public double m_defaultTargetRPM = 5000;

  public ShooterSubsystem(LimelightSubsystem p_limelight) {
    m_limelight = p_limelight;
    RPMCalculator = new CalculateRPM(m_limelight);

    m_wheelMotor =
        new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
    m_wheelMotor2 =
        new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
    m_wheelMotor.restoreFactoryDefaults();
    m_wheelMotor2.restoreFactoryDefaults();
    m_wheelMotor2.follow(m_wheelMotor);

    m_wheelMotor.setIdleMode(IdleMode.kCoast);
    m_wheelMotor2.setIdleMode(IdleMode.kCoast);

    m_encoder1 = m_wheelMotor.getEncoder();
    m_encoder2 = m_wheelMotor2.getEncoder();
    m_pidController = m_wheelMotor.getPIDController();
    m_pidController.setFeedbackDevice(m_encoder1);
    m_pidController.setP(WHEEL_P);
    m_pidController.setI(WHEEL_I);
    m_pidController.setD(WHEEL_D);
    m_pidController.setFF(WHEEL_FF);
    m_pidController.setOutputRange(.1, 1);
  }

  @Override
  public void periodic() {}

  public double CalculateRPM() {
    return RPMCalculator.InterpolateDistance();
  }

  public double getRPM1() {
    return m_encoder1.getVelocity();
  }

  public double getRPM2() {
    return m_encoder2.getVelocity();
  }

  public void setTargetRPM(double p_targetRPM) {
    m_targetRPM = p_targetRPM;
    m_pidController.setReference(m_targetRPM, ControlType.kVelocity);
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public double getDefaultRPM() {
      return m_defaultTargetRPM;
  }

  public void turnOff() {
    setPower(0);
  }

  public void setPower(double power) {
    m_wheelMotor.set(power);
  }

  public double getP() {
    return m_pidController.getP();
  }

  public double getFF() {
    return m_pidController.getFF();
  }

  public void setP(double P) {
    m_pidController.setP(P);
  }

  public double getI() {
    return m_pidController.getI();
  }

  public void setI(double I) {
    m_pidController.setI(I);
  }

  public double getD() {
    return m_pidController.getD();
  }

  public void setD(double D) {
    m_pidController.setD(D);
  }

  public void setFF(double FF) {
    m_pidController.setFF(FF);
  }
}
