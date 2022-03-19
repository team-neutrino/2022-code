// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.util.CalculateRPM;

public class ShooterSubsystem extends PIDSubsystem {
  /** Shooter Constants */
  private final double WHEEL_P = 0.3;

  private final double WHEEL_I = 0;
  private final double WHEEL_D = 0;
  private final double WHEEL_FF = 0.2;

  private CANSparkMax m_wheelMotor;
  private CANSparkMax m_wheelMotor2;
  private Encoder m_encoder;
  private SimpleMotorFeedforward m_feedForward;
  private LimelightSubsystem m_limelight;
  private CalculateRPM RPMCalculator;

  private double m_targetRPM;
  public double m_shuffleBoardRPM = 100;

  public ShooterSubsystem(LimelightSubsystem p_limelight) {
    super(new PIDController(.03, 0, 0));
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

    m_wheelMotor.setClosedLoopRampRate(1.5);
    m_encoder = new Encoder(3, 4);
    // m_feedForward = new SimpleMotorFeedforward(ks, kv)
  }

  public double CalculateRPM() {
    return RPMCalculator.InterpolateDistance();
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public double getShuffleboardRPM() {
    return m_shuffleBoardRPM;
  }

  public void setShuffleboardRPM(double shuffleboardRPM) {
    m_shuffleBoardRPM = shuffleboardRPM;
  }

  public void turnOff() {
    m_wheelMotor.set(0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    if(output <= 0)
    {
      m_wheelMotor.setVoltage(.1);
    }else{
      m_wheelMotor.setVoltage(1);
    }
  }

  public double getP() {
    return getController().getP() * 1000.0;
  }

  // public double getFF() {
  //   return m_pidController.getFF() * 1000.0;
  // }

  public void setP(double P) {
    getController().setP(P / 1000.0);
  }

  public double getI() {
    return getController().getI() * 1000.0;
  }

  public void setI(double I) {
    getController().setI(I / 1000.0);
  }

  public double getD() {
    return getController().getD() * 1000.0;
  }

  public void setD(double D) {
    getController().setD(D / 1000.0);
  }

  // public void setFF(double FF) {
  //   m_pidController.setFF(FF / 1000.0);
  // }

  @Override
  public double getMeasurement() {
    return m_encoder.getRate();
  }

  public boolean atSetPoint() {
    return getController().atSetpoint();
  }

  public double getOutput() {
    return getController().calculate(getMeasurement(), 0.0);
  }
}
