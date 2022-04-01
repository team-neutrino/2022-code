package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MagicButtonCommand extends CommandBase {

  private ShooterSubsystem m_shooter;
  private IndexSubsystem m_index;
  private double m_targetRPM;

  public MagicButtonCommand(ShooterSubsystem p_shooter, IndexSubsystem p_index) {
    m_shooter = p_shooter;
    m_index = p_index;
    addRequirements(m_index, m_shooter);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
