package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class Climb2ExtendCommand extends CommandBase {
  private ClimberSubsystem m_climberSubsystem;

  public Climb2ExtendCommand(ClimberSubsystem subsystem) {
    m_climberSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climberSubsystem.extendClimberArm2();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
