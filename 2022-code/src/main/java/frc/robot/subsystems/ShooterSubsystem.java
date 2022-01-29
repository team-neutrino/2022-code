// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase
{
    /** Shooter Constants */
    private final double WHEEL_P = 0.08;
    private final double WHEEL_I = 0;
    private final double WHEEL_D = 2;

    private CANSparkMax m_wheelMotor;
    private CANSparkMax m_wheelMotor2;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;
  
    private double m_targetRPM;

    public ShooterSubsystem()
    {
        m_wheelMotor = new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
        m_wheelMotor2 = new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
        m_wheelMotor.restoreFactoryDefaults();
        m_wheelMotor2.restoreFactoryDefaults();
        m_wheelMotor2.follow(m_wheelMotor);

        m_wheelMotor.setInverted(true);
        m_wheelMotor2.setInverted(true);
        m_wheelMotor.setIdleMode(IdleMode.kCoast);
        m_wheelMotor2.setIdleMode(IdleMode.kCoast);

        m_encoder = m_wheelMotor.getEncoder();
        m_pidController = m_wheelMotor.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);
        m_pidController.setP(WHEEL_P);
        m_pidController.setI(WHEEL_I);
        m_pidController.setD(WHEEL_D);
        m_pidController.setOutputRange(.1, 1);
    }

    @Override
    public void periodic()
    {
    }

    public double getRPM()
    {
        return m_encoder.getVelocity();
    }

    public void setTargetRPM(double p_targetRPM)
    {
        m_targetRPM = p_targetRPM;
        m_pidController.setReference(m_targetRPM, ControlType.kVelocity);
    }

    public double getTargetRPM()
    {
        return m_targetRPM;
    }

    public void turnOff()
    {
        setPower(0);
    }

    public void setPower(double power)
    {
        m_wheelMotor.set(power);
    }
}
