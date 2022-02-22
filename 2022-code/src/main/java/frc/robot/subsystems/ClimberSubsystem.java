package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

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
  private RelativeEncoder m_encoder;

  private CANSparkMax m_climber =
      new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_1, MotorType.kBrushless);
  private DoubleSolenoid m_keyPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, SOLENOID_KEYPISTON_EXTEND, SOLENOID_KEYPISTON_RETRACT);
  private DigitalInput m_limitSwitch = new DigitalInput(Constants.DigitalConstants.CLIMBER_SWITCH);

  public ClimberSubsystem() {
    m_climber.setIdleMode(IdleMode.kBrake);
    m_encoder = m_climber.getEncoder();
  }

  public void keyLock() {
    m_keyPiston.set(Value.kForward);
  }

  public void keyUnlock() {
    m_keyPiston.set(Value.kReverse);
  }

  public void extendClimber() {
    m_climber.set(CLIMBER_SPEED);
  }

  public void retractClimber() {
    m_climber.set(CLIMBER_SPEED * -1);
  }

  public void climberOff() {
    m_climber.set(0);
  }

  public Boolean getLimitSwitch() {
    return m_limitSwitch.get();
  }

  public double getClimbEncoderOne() {

    return m_encoder.getVelocity();
  }
}
