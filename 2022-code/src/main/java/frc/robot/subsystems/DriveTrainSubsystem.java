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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  private RelativeEncoder m_encoderR1;
  private RelativeEncoder m_encoderR2;
  private RelativeEncoder m_encoderL1;
  private RelativeEncoder m_encoderL2;

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
  private static final double K_GEAR_RATIO = 1.0 / 8.0;
  private static final double K_WHEEL_DIAMETER = 0.127;
  private static final double K_WHEEL_CIRCUMFERENCE = Math.PI * K_WHEEL_DIAMETER;
  private static final double K_ENCODER_CONVERSION = (K_GEAR_RATIO * K_WHEEL_CIRCUMFERENCE);

  NetworkTableEntry m_xEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("translationX");
  NetworkTableEntry m_yEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("translationY");

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

    m_leftMotor1.setInverted(true);
    m_leftMotor2.setInverted(true);

    m_leftMotor1.burnFlash();
    m_leftMotor2.burnFlash();
    m_rightMotor1.burnFlash();
    m_rightMotor2.burnFlash();

    m_encoderR1 = m_rightMotor1.getEncoder();
    m_encoderR2 = m_rightMotor2.getEncoder();
    m_encoderL1 = m_leftMotor1.getEncoder();
    m_encoderL2 = m_leftMotor2.getEncoder();

    m_encoderR1.setPositionConversionFactor(K_ENCODER_CONVERSION);
    m_encoderR2.setPositionConversionFactor(K_ENCODER_CONVERSION);
    m_encoderL1.setPositionConversionFactor(K_ENCODER_CONVERSION);
    m_encoderL2.setPositionConversionFactor(K_ENCODER_CONVERSION);

    m_encoderR1.setVelocityConversionFactor(K_ENCODER_CONVERSION / 60);
    m_encoderR2.setVelocityConversionFactor(K_ENCODER_CONVERSION / 60);
    m_encoderL1.setVelocityConversionFactor(K_ENCODER_CONVERSION / 60);
    m_encoderL2.setVelocityConversionFactor(K_ENCODER_CONVERSION / 60);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    resetOdometry(m_odometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    // called once per scheduler run if you didn't already know
    m_odometry.update(
        Rotation2d.fromDegrees(getYaw()), m_encoderL1.getPosition(), m_encoderR1.getPosition());
    m_xEntry.setNumber(m_odometry.getPoseMeters().getTranslation().getX());
    m_yEntry.setNumber(m_odometry.getPoseMeters().getTranslation().getY());
  }

  public void setMotors(double m_setLeftSpeed, double m_setRightSpeed) {
    m_leftMotors.set(-m_setLeftSpeed);
    m_rightMotors.set(-m_setRightSpeed);
  }

  public void resetEncoders() {
    m_encoderR1.setPosition(0);
    m_encoderR2.setPosition(0);
    m_encoderL1.setPosition(0);
    m_encoderL2.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_navX.reset();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getYaw() {
    return m_navX.getYaw() * -1;
  }

  public double getDriveEncoderR1() {
    return m_encoderR1.getVelocity();
  }

  public double getDriveEncoderR2() {
    return m_encoderR2.getVelocity();
  }

  public double getDriveEncoderL1() {
    return m_encoderL1.getVelocity();
  }

  public double getDriveEncoderL2() {
    return m_encoderL2.getVelocity();
  }

  public double getDriveEncoderR1Position() {
    return m_encoderR1.getPosition();
  }

  public double getDriveEncoderL1Position() {
    return m_encoderL1.getPosition();
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
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getDriveEncoderL1(), getDriveEncoderR1());
  }
}
