package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ClimbExtendCommand extends CommandBase {
  private ClimberSubsystem m_climberSubsystem;
  private ShooterSubsystem m_shooter;

  public ClimbExtendCommand(ClimberSubsystem subsystem, ShooterSubsystem p_shooter) {
    m_climberSubsystem = subsystem;
    m_shooter = p_shooter;
    addRequirements(subsystem, m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.turnOff();
  }

  @Override
  public void execute() {
    m_climberSubsystem.extendClimber();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
