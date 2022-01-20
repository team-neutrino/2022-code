// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;


@SuppressWarnings(
{ "all" })
public class ShooterSubsystem extends SubsystemBase
{
    private CANSparkMax m_wheelMotor;
    private CANSparkMax m_wheelMotor2;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;
  
    private double m_targetVelocity;

    /**
     * Creates a new Shooter.
     */

    public ShooterSubsystem()
    {
        m_wheelMotor = new CANSparkMax(Constants.CanId.MOTOR_CONTROLLER_SHOOTER1, MotorType.kBrushless);
        m_wheelMotor2 = new CANSparkMax(Constants.CanId.MOTOR_CONTROLLER_SHOOTER2, MotorType.kBrushless);
        m_wheelMotor2.follow(m_wheelMotor);

        m_wheelMotor.restoreFactoryDefaults();
        m_wheelMotor2.restoreFactoryDefaults();
        m_wheelMotor.setInverted(true);
        m_wheelMotor2.setInverted(true);
        m_wheelMotor.setIdleMode(IdleMode.kBrake);
        m_wheelMotor2.setIdleMode(IdleMode.kBrake);

        m_encoder = m_wheelMotor.getEncoder();
        m_pidController = m_wheelMotor.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);
        m_pidController.setP(Shooter.WHEEL_P);
        m_pidController.setI(Shooter.WHEEL_I);
        m_pidController.setD(Shooter.WHEEL_D);
        m_pidController.setOutputRange(-1, 1);
    }

    @Override
    public void periodic()
    {
    }

    public double getVelocity()
    {
        return m_encoder.getPosition();
    }

    public double getTargetVelocity()
    {
        return m_targetVelocity;
    }

    public void turnOff()
    {
        m_targetVelocity = 0;
        setPower(0);
    }

    public void setPower(double power)
    {
        m_wheelMotor.set(power);
    }

    public void setVelocity(double velocity)
    {
        m_targetVelocity = velocity;
        m_wheelMotor.set(velocity);
    }
}
