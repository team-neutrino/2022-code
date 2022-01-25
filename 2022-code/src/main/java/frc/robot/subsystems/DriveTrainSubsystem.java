package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveTrainSubsystem extends SubsystemBase {
    private CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_RIGHT_1_ID,MotorType.kBrushless);
    private CANSparkMax m_rightMotor2 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_RIGHT_2_ID, MotorType.kBrushless);
    private CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_LEFT_1_ID, MotorType.kBrushless);
    private CANSparkMax m_leftMotor2 = new CANSparkMax(Constants.CANIDConstants.DRIVETRAIN_MOTOR_LEFT_2_ID, MotorType.kBrushless);
    
    private MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
    private MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

    public DriveTrainSubsystem()
    {
        m_leftMotors.setInverted(true);
    }

    @Override
    public void periodic()
    {
        //called once per scheduler run if you didn't already know
    }
    public void setMotors(double m_setRightSpeed, double m_setLeftSpeed) 
    {
        m_leftMotors.set(m_setLeftSpeed);
        m_rightMotors.set(m_setRightSpeed);
    }
}

