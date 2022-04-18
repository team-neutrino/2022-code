package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ClimbRetractCommand extends CommandBase {
  private ClimberSubsystem m_climberSubsystem;
  private ShooterSubsystem m_shooter;

  public ClimbRetractCommand(ClimberSubsystem subsystem, ShooterSubsystem p_shooter) {
    m_climberSubsystem = subsystem;
    m_shooter = p_shooter;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_shooter.turnOff();
  }

  @Override
  public void execute() {
    if (m_climberSubsystem.getLimitSwitch()) {
      m_climberSubsystem.climberOff();
    } else {
      m_climberSubsystem.retractClimber();
    }
    m_climberSubsystem.compressorOff();
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.keyLock();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
