package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final int SOLENOID_KEYPISTON_RETRACT = 7;
  private double CLIMBER_UP_SPEED = 0.5;
  private double CLIMBER_DOWN_SPEED = -.5;
  private RelativeEncoder m_encoder;

  private CANSparkMax m_climber =
      new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_1, MotorType.kBrushless);
  private Solenoid m_keyPiston =
      new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID_KEYPISTON_RETRACT);
  private DigitalInput m_limitSwitch = new DigitalInput(Constants.DigitalConstants.CLIMBER_SWITCH);
  private final Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  @Override
  public void periodic() {}

  public ClimberSubsystem() {
    m_climber.setIdleMode(IdleMode.kBrake);
    m_climber.setInverted(true);
    m_climber.setSmartCurrentLimit(80);
    m_climber.setOpenLoopRampRate(.5);
    m_encoder = m_climber.getEncoder();
    m_climber.burnFlash();
  }

  public void keyLock() {
    m_keyPiston.set(false);
  }

  public void keyUnlock() {
    m_keyPiston.set(true);
  }

  public void extendClimber() {
    m_climber.set(CLIMBER_UP_SPEED);
  }

  public void retractClimber() {
    m_climber.set(CLIMBER_DOWN_SPEED);
  }

  public void climberOff() {
    m_climber.set(0);
  }

  public Boolean getLimitSwitch() {
    return m_limitSwitch.get();
  }

  public double getClimbEncoder() {

    return m_encoder.getVelocity();
  }

  public void compressorOn() {
    m_compressor.enableDigital();
  }

  public void compressorOff() {
    m_compressor.disable();
  }
}
