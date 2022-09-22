// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretPIDSubsystem extends SubsystemBase {
  DriveTrainSubsystem m_driveTrain;
  public static double FORWARD_SOFT_LIMIT_THRESHOLD = 750;
  public static double REVERSE_SOFT_LIMIT_THRESHOLD = 100;
  double cycles = 0;
  double deltaA = 0;
  private TalonSRXConfiguration m_turretMotorConfig = new TalonSRXConfiguration();
  private TalonSRX m_turretMotor = new TalonSRX(Constants.CANIDConstants.TURRET_MOTOR_ID);
  private double m_currentAngle;
  private double m_initialAngle;
  private double TURRET_MOTOR_OUTPUT = 0.5;
  private double turretZero = 420;
  private double turretKs = 0;
  private double turretKv = 0;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(turretKs, turretKv);

  /** Creates a new TurretPIDSubsystem. */
  public TurretPIDSubsystem() {
    m_turretMotorConfig.slot0.kP = 5.0;
    m_turretMotorConfig.slot0.kD = 0;
    m_turretMotorConfig.slot0.kI = 0;
    m_turretMotorConfig.slot0.kF = 0;
    m_turretMotor.configAllSettings(m_turretMotorConfig);
    m_turretMotor.setNeutralMode(NeutralMode.Brake);
    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    m_turretMotor.configFeedbackNotContinuous(false, 0);
    m_turretMotor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT_THRESHOLD);
    m_turretMotor.configForwardSoftLimitEnable(true);
    m_turretMotor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT_THRESHOLD);
    m_turretMotor.configReverseSoftLimitEnable(true);
    m_initialAngle = getCurrentAngle();
  }
 /* 
   public void setTargetAngle(double currentTurretAngle, double tx, double distance) {

    double yVelocity = m_driveTrain.getXVelocity();
    double u = (currentTurretAngle + turretZero) * 0.4736;
    double theta = u + tx;
    double tangentialVelocity = distance * m_driveTrain.getZGyro();
   // double turretAngularVelocity = yVelocity * Math.sin(Math.toRadians(theta) / distance);
    double angularVelocity = -Math.toRadians(m_driveTrain.getZGyro());
    double targetAngle = feedforward.calculate(turretAngularVelocity + angularVelocity);
    m_turretMotor.set(ControlMode.Position, targetAngle);
  }
  */
  

  public void setTargetAngle(double targetAngle) {
    if (targetAngle > FORWARD_SOFT_LIMIT_THRESHOLD)
      m_turretMotor.set(ControlMode.Position, FORWARD_SOFT_LIMIT_THRESHOLD - 25.0);
    else if (targetAngle < REVERSE_SOFT_LIMIT_THRESHOLD)
      m_turretMotor.set(ControlMode.Position, REVERSE_SOFT_LIMIT_THRESHOLD + 25.0);
    else m_turretMotor.set(ControlMode.Position, targetAngle);
  }

  public double getInitialAngle() {
    return m_initialAngle;
  }

  public double getCurrentAngle() {
    return m_currentAngle;
  }

  public void stop() {
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public void turnClockwise() {
    m_turretMotor.set(ControlMode.PercentOutput, TURRET_MOTOR_OUTPUT);
  }

  public void turnCounterClockwise() {
    m_turretMotor.set(ControlMode.PercentOutput, TURRET_MOTOR_OUTPUT * -1);
  }

  public void setPower(double power) {
    m_turretMotor.set(ControlMode.PercentOutput, power);
  }

  public void deltaA(){
    double angle = getCurrentAngle();
    double deltaA = 0;
    double deltaATwo = 0;
    double deltaAThree = 0;
    double angleOne = 0;
    double angleTwo = 0;
    double angleThree = 0;
    double angleFour = 0;

    angleFour = angleThree;
    angleThree = angleTwo;
    angleTwo = angleOne;
    angleOne = angle;

    deltaA = angle - angleOne;
    deltaATwo = angleTwo - angleThree;
    deltaAThree = angleThree - angleFour;

    this.deltaA = (deltaA + deltaATwo + deltaAThree) / 3;
  }

  public double getDeltaA(){
    return deltaA;
  }

  public void angleUpdater(){
    if (cycles % 10 == 0){
      deltaA();
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_currentAngle = m_turretMotor.getSelectedSensorPosition(0);
    cycles++;
    angleUpdater();
  }

  public double getP() {
    return m_turretMotorConfig.slot0.kP;
  }

  public void setP(double P) {
    m_turretMotorConfig.slot0.kP = P;
  }

  public double getI() {
    return m_turretMotorConfig.slot0.kI;
  }

  public void setI(double I) {
    m_turretMotorConfig.slot0.kI = I;
  }

  public double getD() {
    return m_turretMotorConfig.slot0.kD;
  }

  public void setD(double D) {
    m_turretMotorConfig.slot0.kD = D;
  }
}
