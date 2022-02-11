package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrainSubsystem extends SubsystemBase {
    private RelativeEncoder m_encoder1;
    private RelativeEncoder m_encoder2;
    private RelativeEncoder m_encoder3;   
    private RelativeEncoder m_encoder4;

    private CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_RIGHT_1_ID,MotorType.kBrushless);
    private CANSparkMax m_rightMotor2 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_RIGHT_2_ID, MotorType.kBrushless);
    private CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_LEFT_1_ID, MotorType.kBrushless);
    private CANSparkMax m_leftMotor2 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_LEFT_2_ID, MotorType.kBrushless);
    
    private MotorControllerGroup m_rightMotors; 
    private MotorControllerGroup m_leftMotors;

    private final DifferentialDriveOdometry m_odometry;
    private AHRS m_navX = new AHRS(SPI.Port.kMXP);

    public DriveTrainSubsystem()
    {
        m_leftMotor2.follow(m_leftMotor1);
        m_rightMotor2.follow(m_rightMotor1);
        m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
        m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
        m_leftMotors.setInverted(true);
        m_encoder1 = m_rightMotor1.getEncoder();
        m_encoder2 = m_rightMotor2.getEncoder();
        m_encoder3 = m_leftMotor1.getEncoder();
        m_encoder4 = m_leftMotor2.getEncoder();
        
        setTankDriveVolts(0, 0);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    }

    @Override
    public void periodic()
    {
        System.out.println("navx value" + m_navX.getDisplacementX());
        System.out.println("navY value" + m_navX.getDisplacementY());
        //called once per scheduler run if you didn't already know
    }
    
    public void setMotors(double m_setRightSpeed, double m_setLeftSpeed) 
    {
        m_leftMotors.set(-m_setLeftSpeed);
        m_rightMotors.set(-m_setRightSpeed);
    }

    public double getnavX(){
        return m_navX.getDisplacementX();
    }

    public double getnavY(){
        return m_navX.getDisplacementY();
    }
    
    public double getYaw()
    {
        return m_navX.getYaw();
    }

    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
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

    public void setTankDriveVolts(double leftVolts, double rightVolts)
    {
        m_leftMotors.setVoltage(-leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(getDriveEncoder1(), getDriveEncoder3());
    }
}


