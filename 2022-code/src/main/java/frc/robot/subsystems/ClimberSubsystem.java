package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;

public class ClimberSubsystem extends SubsystemBase {
    private final int SOLENOID_KEYPISTON_EXTEND = 0;
    private final int SOLENOID_KEYPISTON_RETRACT = 1;
    private double CLIMBER_SPEED = 0.5;

    private CANSparkMax m_climber1 = new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_1, MotorType.kBrushless);
    private CANSparkMax m_climber2 = new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_2, MotorType.kBrushless);
    private DoubleSolenoid m_keyPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_KEYPISTON_EXTEND, SOLENOID_KEYPISTON_RETRACT);
    

    public ClimberSubsystem() 
    {
        m_climber2.follow(m_climber1);
    }

    public void keyLock() 
    {
        m_keyPiston.set(Value.kForward);
    }

    public void keyUnlock()
    {
        m_keyPiston.set(Value.kReverse);
    }

    public void extendClimber()
    {
        m_climber1.set(CLIMBER_SPEED);
    }

    public void retractClimber() 
    {
        m_climber1.set(CLIMBER_SPEED*-1);
    }

    public void climberOff() 
    {
        m_climber1.set(0);
    }
}
