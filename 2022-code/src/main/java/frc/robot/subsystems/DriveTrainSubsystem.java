package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCON;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveTrainSubsystem extends SubsystemBase {
    private CANSparkMax m_rightmotor1 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_ONE,MotorType.kBrushless);
    private CANSparkMax m_rightmotor2 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_TWO, MotorType.kBrushless);
    private CANSparkMax m_leftmotor1 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_THREE, MotorType.kBrushless);
    private CANSparkMax m_leftmotor2 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_FOUR, MotorType.kBrushless);
    private MotorControllerGroup m_rightmotors = new MotorControllerGroup(m_rightmotor1, m_rightmotor2);
    private MotorControllerGroup m_leftmotors = new MotorControllerGroup(m_leftmotor1, m_leftmotor2);

    private Joystick m_rJ;
    private Joystick m_lJ;

    public DriveTrainSubsystem(Joystick m_rightJoystick, Joystick m_leftJoystick)
    {
        m_leftmotors.setInverted(true);
        m_rJ = m_rightJoystick;
        m_lJ= m_leftJoystick;
        
    }
    @Override
    public void periodic()
    {
        m_rightmotors.set(m_rJ.getY());
        m_leftmotors.set(m_lJ.getY());
       System.out.println(m_rightmotors.get()); 
       System.out.println(m_leftmotors.get());
    }
}

