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
  private final double WHEEL_P = 0.06;
  private final double WHEEL_I = 1.0;
  private final double WHEEL_D = 0;
  private final double WHEEL_FF = 0.07;

  private final double ROLLER_P = 0.3;
  private final double ROLLER_I = 0.0008;
  private final double ROLLER_D = 0;
  private final double ROLLER_FF = 0.2;

  private CANSparkMax m_wheelMotor;
  private CANSparkMax m_wheelMotor2;
  private CANSparkMax m_topRoller;
  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;
  private RelativeEncoder m_topRollerEncoder;
  private SparkMaxPIDController m_pidController;
  private SparkMaxPIDController m_TopRollerPidController;
  private LimelightSubsystem m_limelight;
  private CalculateRPM RPMCalculator;

  private double m_targetRPM;
private double m_topRollerShuffleboardRPM = 100;
  public double m_shuffleBoardRPM = 100;

  public ShooterSubsystem(LimelightSubsystem p_limelight) {
    m_limelight = p_limelight;
    RPMCalculator = new CalculateRPM(m_limelight);

    m_wheelMotor =
        new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
    m_wheelMotor2 =
        new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
    m_topRoller =
        new CANSparkMax(Constants.CANIDConstants.SHOOTER_ROLLER_MOTOR_ID, MotorType.kBrushless);
    m_wheelMotor.restoreFactoryDefaults();
    m_wheelMotor2.restoreFactoryDefaults();
    m_wheelMotor2.follow(m_wheelMotor);

    m_wheelMotor.setIdleMode(IdleMode.kCoast);
    m_wheelMotor2.setIdleMode(IdleMode.kCoast);
    m_topRoller.setInverted(true);
    m_topRoller.setSmartCurrentLimit(40);

    m_wheelMotor.setClosedLoopRampRate(1.5);

    m_encoder1 = m_wheelMotor.getEncoder();
    m_encoder2 = m_wheelMotor2.getEncoder();
    m_topRollerEncoder = m_topRoller.getEncoder();

    m_pidController = m_wheelMotor.getPIDController();
    m_pidController.setFeedbackDevice(m_encoder1);
    m_pidController.setP(WHEEL_P / 1000.0);
    m_pidController.setI(WHEEL_I / 1000.0);
    m_pidController.setD(WHEEL_D / 1000.0);
    m_pidController.setFF(WHEEL_FF / 1000.0);
    m_pidController.setIZone(100);
    m_pidController.setOutputRange(.1, 1);

    m_TopRollerPidController = m_topRoller.getPIDController();
    m_TopRollerPidController.setFeedbackDevice(m_topRollerEncoder);
    m_TopRollerPidController.setP(ROLLER_P / 1000.0);
    m_TopRollerPidController.setI(ROLLER_I / 1000.0);
    m_TopRollerPidController.setD(ROLLER_D / 1000.0);
    m_TopRollerPidController.setFF(ROLLER_FF / 1000.0);
    m_TopRollerPidController.setIZone(200);
    m_TopRollerPidController.setOutputRange(.1, 1);
    m_topRoller.burnFlash();
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

  public double getTopRollerRPM() {
    return m_topRollerEncoder.getVelocity();
  }

  public void setTopRollerRPM(double p_RPM) {
    m_TopRollerPidController.setReference(p_RPM * 1.2, ControlType.kVelocity);
  }

  public void setTargetRPM(double p_targetRPM) {
    m_targetRPM = p_targetRPM;
    m_pidController.setReference(m_targetRPM, ControlType.kVelocity);
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public double getShuffleboardRPM() {
    return m_shuffleBoardRPM;
  }

  public double getTopRollerShuffleboardRPM() {
    return m_topRollerShuffleboardRPM;
  }

  public void setShuffleboardRPM(double shuffleboardRPM) {
    m_shuffleBoardRPM = shuffleboardRPM;
  }

  public void setTopRollerShuffleboardRPM(double shuffleboardRPM) {
    m_topRollerShuffleboardRPM = shuffleboardRPM;
  }

  public void turnOff() {
    setPower(0);
  }

  public void setPower(double power) {
    m_wheelMotor.set(power);
    m_topRoller.set(power);
  }

  public double getP() {
    return m_TopRollerPidController.getP() * 1000.0;
  }

  public double getFF() {
    return m_TopRollerPidController.getFF() * 1000.0;
  }

  public void setP(double P) {
    m_TopRollerPidController.setP(P / 1000.0);
  }

  public double getI() {
    return m_TopRollerPidController.getI() * 1000.0;
  }

  public void setI(double I) {
    m_TopRollerPidController.setI(I / 1000.0);
  }

  public double getD() {
    return m_TopRollerPidController.getD() * 1000.0;
  }

  public void setD(double D) {
    m_TopRollerPidController.setD(D / 1000.0);
  }

  public void setFF(double FF) {
    m_TopRollerPidController.setFF(FF / 1000.0);
  }

  public boolean magicShooter(double RPM, double TRPM) {
    return Math.abs(RPM - TRPM) <= 10;
  }
}
