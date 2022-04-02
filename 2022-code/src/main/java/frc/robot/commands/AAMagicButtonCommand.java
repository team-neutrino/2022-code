package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class AAMagicButtonCommand extends CommandBase {

  private final double LIMELIGHT_MULTIPLICATION = 6.0;
  private ShooterSubsystem m_shooter;
  private IndexSubsystem m_index;
  private LimelightSubsystem m_limelight;
  private TurretPIDSubsystem m_turret;
  private double m_targetRPM;
  private Timer m_timer;
  private double m_time;

  public AAMagicButtonCommand(
      ShooterSubsystem p_shooter,
      IndexSubsystem p_index,
      LimelightSubsystem p_limelight,
      TurretPIDSubsystem p_turret,
      double p_time) {
    m_shooter = p_shooter;
    m_limelight = p_limelight;
    m_index = p_index;
    m_timer = new Timer();
    m_time = p_time;
    addRequirements(m_index, m_shooter, m_limelight);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLimelightOn();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.getTv() == true) {
      m_turret.setTargetAngle(
          m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx());
    } else {
      m_turret.stop();
    }

    m_targetRPM = m_shooter.CalculateRPM();
    m_shooter.setTargetRPM(m_targetRPM);
    if (m_shooter.magicShooter(m_shooter.getRPM1(), m_targetRPM)) {
      m_index.MotorOneStart();
      m_index.MotorTwoStart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.MotorOneStop();
    m_index.MotorTwoStop();
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() >= m_time) {
      return true;
    }
    return false;
  }
}
