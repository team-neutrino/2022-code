package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCON;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveTrainSubsystem extends SubsystemBase {
    private CANSparkMax m_rightMotor1 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_DRIVE_RIGHT_1,MotorType.kBrushless);
    private CANSparkMax m_rightMotor2 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_DRIVE_RIGHT_2, MotorType.kBrushless);
    private CANSparkMax m_leftMotor1 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_DRIVE_LEFT_1, MotorType.kBrushless);
    private CANSparkMax m_leftMotor2 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_DRIVE_LEFT_2, MotorType.kBrushless);
    private MotorControllerGroup m_rightmotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
    private MotorControllerGroup m_leftmotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

    private Joystick m_rightJoystick;
    private Joystick m_leftJoystick;

    public DriveTrainSubsystem(Joystick p_rightJoystick, Joystick p_leftJoystick)
    {
        m_leftmotors.setInverted(true);
        m_rightJoystick = p_rightJoystick;
        m_leftJoystick= p_leftJoystick;
        
    }
    @Override
    public void periodic()
    {
        m_rightmotors.set(m_rightJoystick.getY());
        m_leftmotors.set(m_leftJoystick.getY());
    }
}

