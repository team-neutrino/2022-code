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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  public DriveTrainSubsystem() {
    m_rightMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    m_leftMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();

    m_rightMotor1.setIdleMode(IdleMode.kBrake);
    m_rightMotor2.setIdleMode(IdleMode.kBrake);
    m_leftMotor1.setIdleMode(IdleMode.kBrake);
    m_leftMotor2.setIdleMode(IdleMode.kBrake);

    m_rightMotors.setInverted(true);
    m_encoder1 = m_rightMotor1.getEncoder();
    m_encoder2 = m_rightMotor2.getEncoder();
    m_encoder3 = m_leftMotor1.getEncoder();
    m_encoder4 = m_leftMotor2.getEncoder();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
  }

  @Override
  public void periodic() {
    // called once per scheduler run if you didn't already know
    m_odometry.update(m_navX.getRotation2d(), m_encoder3.getPosition(), m_encoder1.getPosition());
    m_diffDrive.feed();
    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
  }

  public void setMotors(double m_setLeftSpeed, double m_setRightSpeed) {
    m_leftMotors.set(m_setLeftSpeed);
    m_rightMotors.set(m_setRightSpeed);
  }

  public void resetEncoders() {
    m_encoder1.setPosition(0);
    m_encoder2.setPosition(0);
    m_encoder3.setPosition(0);
    m_encoder4.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_navX.getRotation2d());
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

  public double getNavZ() {
      return m_navX.getDisplacementZ();
  }

  public double getNavYaw() {
    return m_navX.getYaw();
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor1.setVoltage(leftVolts);
    // m_leftMotor2.setVoltage(leftVolts); don't need if set follow?
    m_rightMotor1.setVoltage(rightVolts);
    // m_rightMotor2.setVoltage(rightVolts); same
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getDriveEncoder1(), getDriveEncoder3());
  }
}
