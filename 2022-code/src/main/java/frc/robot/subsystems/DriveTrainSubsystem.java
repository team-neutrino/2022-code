package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;
  private RelativeEncoder m_encoder3;
  private RelativeEncoder m_encoder4;

  private CANSparkMax m_rightMotor1 =
      new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_RIGHT_1_ID, MotorType.kBrushless);
  private CANSparkMax m_rightMotor2 =
      new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_RIGHT_2_ID, MotorType.kBrushless);
  private CANSparkMax m_leftMotor1 =
      new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_LEFT_1_ID, MotorType.kBrushless);
  private CANSparkMax m_leftMotor2 =
      new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_LEFT_2_ID, MotorType.kBrushless);

  private MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
  private MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  public DriveTrainSubsystem() {
    m_leftMotors.setInverted(true);
    m_encoder1 = m_rightMotor1.getEncoder();
    m_encoder2 = m_rightMotor2.getEncoder();
    m_encoder3 = m_leftMotor1.getEncoder();
    m_encoder4 = m_leftMotor2.getEncoder();
  }

  @Override
  public void periodic() {
    // called once per scheduler run if you didn't already know
  }

  public void setMotors(double m_setRightSpeed, double m_setLeftSpeed) {
    m_leftMotors.set(m_setLeftSpeed);
    m_rightMotors.set(m_setRightSpeed);
  }

  public double getDriveEncoder1() {
    return m_encoder1.getVelocity();
  }

  public double getDriveEncoder2() {
    return m_encoder2.getVelocity();
  }

  public double getDriveEncoder3() {
    return m_encoder3.getVelocity();
  }

  public double getDriveEncoder4() {
    return m_encoder4.getVelocity();
  }
}
