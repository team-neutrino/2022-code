package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final int SOLENOID_KEYPISTON_EXTEND = 2;
  private final int SOLENOID_KEYPISTON_RETRACT = 3;
  private double CLIMBER_SPEED = 0.5;
  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;

  private CANSparkMax m_climber1 =
      new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax m_climber2 =
      new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_2, MotorType.kBrushless);
  private DoubleSolenoid m_keyPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, SOLENOID_KEYPISTON_EXTEND, SOLENOID_KEYPISTON_RETRACT);
  private DigitalInput m_limitSwitch = new DigitalInput(Constants.DigitalConstants.CLIMBER_SWITCH);

  public ClimberSubsystem() {
    m_climber2.follow(m_climber1);
    m_encoder1 = m_climber1.getEncoder();
    m_encoder2 = m_climber2.getEncoder();
  }

  public void keyLock() {
    m_keyPiston.set(Value.kForward);
  }

  public void keyUnlock() {
    m_keyPiston.set(Value.kReverse);
  }

  public void extendClimber() {
    m_climber1.set(CLIMBER_SPEED);
  }

  public void retractClimber() {
    m_climber1.set(CLIMBER_SPEED * -1);
  }

  public void climberOff() {
    m_climber1.set(0);
  }

  public Boolean getLimitSwitch() {
    return m_limitSwitch.get();
  }

  public double getClimbEncoderOne() {

    return m_encoder1.getVelocity();
  }

  public double getClimbEncoderTwo() {

    return m_encoder2.getVelocity();
  }
}
