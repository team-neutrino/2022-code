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
  private double CLIMBER_UP_SPEED = 0.5;
  private double CLIMBER_DOWN_SPEED = -.5;
  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;
  //add limit switch, should determine how and when

  private CANSparkMax m_climberArm1 =
      new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax m_climberArm2 =
      new CANSparkMax(Constants.CANIDConstants.CLIMBER_MOTOR_1, MotorType.kBrushless);

  @Override
  public void periodic() {}

  public ClimberSubsystem() {
    m_climberArm1.setIdleMode(IdleMode.kBrake);
    m_climberArm2.setIdleMode(IdleMode.kBrake);

    m_climberArm1.setInverted(true);
    m_climberArm2.setInverted(true);

    m_climberArm1.setOpenLoopRampRate(.5);
    m_climberArm2.setOpenLoopRampRate(.5);

    m_encoder1 = m_climberArm1.getEncoder();
    m_encoder2 = m_climberArm2.getEncoder();
  }

  public void retractClimberArm1() {
    m_climberArm1.set(CLIMBER_DOWN_SPEED);
  }

  public void retractClimberArm2()
  {
    m_climberArm2.set(CLIMBER_DOWN_SPEED);
  }

  public void extendClimberArm1()
  {
    m_climberArm1.set(CLIMBER_UP_SPEED);
  }

  public void extendClimberArm2()
  {
    m_climberArm2.set(CLIMBER_UP_SPEED);
  }

  public void setclimberArm1Off() {
    m_climberArm1.set(0);
  }

  public void setClimberArm2Off() {
    m_climberArm2.set(0);
  }

  public double getClimbArm1Encoder() {

    return m_encoder1.getVelocity();
  }

  public double getClimbArm2Encoder() {

    return m_encoder2.getVelocity();
  }
}
