package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

  private final DifferentialDriveOdometry m_odometry;

  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  public DriveTrainSubsystem() {
    m_leftMotors.setInverted(true);
    m_encoder1 = m_rightMotor1.getEncoder();
    m_encoder2 = m_rightMotor2.getEncoder();
    m_encoder3 = m_leftMotor1.getEncoder();
    m_encoder4 = m_leftMotor2.getEncoder();

    m_leftMotor1.setIdleMode(IdleMode.kCoast);
    m_leftMotor2.setIdleMode(IdleMode.kCoast);
    m_rightMotor1.setIdleMode(IdleMode.kCoast);
    m_rightMotor2.setIdleMode(IdleMode.kCoast);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
  }

  @Override
  public void periodic() {
    // called once per scheduler run if you didn't already know
  }

  public void setMotors(double m_setRightSpeed, double m_setLeftSpeed) {
    m_leftMotors.set(-m_setLeftSpeed);
    m_rightMotors.set(-m_setRightSpeed);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getYaw() {
    return m_navX.getYaw();
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

  public double getNavX() {
    return m_navX.getDisplacementX();
  }

  public double getNavY() {
    return m_navX.getDisplacementY();
  }

  public double getNavYaw() {
    return m_navX.getYaw();
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor1.setVoltage(-leftVolts);
    m_rightMotor1.setVoltage(-rightVolts);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getDriveEncoder1(), getDriveEncoder3());
  }
}
