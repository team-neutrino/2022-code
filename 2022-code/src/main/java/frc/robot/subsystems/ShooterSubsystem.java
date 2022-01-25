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
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase
{
    private CANSparkMax m_wheelMotor;
    private CANSparkMax m_wheelMotor2;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;
  
    private double m_targetRPM;

    /**
     * Creates a new Shooter.
     */

    public ShooterSubsystem()
    {
        m_wheelMotor = new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
        m_wheelMotor2 = new CANSparkMax(Constants.CANIDConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
        m_wheelMotor2.follow(m_wheelMotor);

        m_wheelMotor.restoreFactoryDefaults();
        m_wheelMotor2.restoreFactoryDefaults();
        m_wheelMotor.setInverted(true);
        m_wheelMotor2.setInverted(true);
        m_wheelMotor.setIdleMode(IdleMode.kCoast);
        m_wheelMotor2.setIdleMode(IdleMode.kCoast);

        m_encoder = m_wheelMotor.getEncoder();
        m_pidController = m_wheelMotor.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);
        m_pidController.setP(ShooterConstants.WHEEL_P);
        m_pidController.setI(ShooterConstants.WHEEL_I);
        m_pidController.setD(ShooterConstants.WHEEL_D);
        m_pidController.setOutputRange(0, 1);
    }

    @Override
    public void periodic()
    {
    }

    public double getRPM()
    {
        return m_encoder.getVelocity();
    }

    public void setPidOn(double p_targetRPM)
    {
        m_targetRPM = p_targetRPM;
        m_pidController.setReference(m_targetRPM, ControlType.kVelocity);
    }

    public double getTargetVelocity()
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
