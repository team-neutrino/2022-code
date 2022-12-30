package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LightSubystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubSystem m_intake;
  private LightSubystem m_light;

  public IntakeCommand(IntakeSubSystem p_intake, LightSubystem p_light) {
    m_intake = p_intake;
    m_light = p_light;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeOn();
    m_light.red();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
