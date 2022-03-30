package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class IntakeDefaultCommand extends CommandBase {
  private IntakeSubSystem m_intake;
  private ColorSubsystem m_color;
  private CommandScheduler m_scheduler;
  private Command m_redBallOutCommand;
  private Timer m_timer = new Timer();

  public IntakeDefaultCommand(IntakeSubSystem p_intake, ColorSubsystem p_color) {
    m_intake = p_intake;
    m_color = p_color;
    addRequirements(m_intake);

    m_redBallOutCommand = new StartEndCommand(
      m_intake., onEnd, requirements)
  }

  private void startSequence() {
    m_timer.start();  
  }

  private void endSequence() {
    m_timer.stop();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setUp();
    m_intake.setIntakeOff();

    if (m_color.getIsRed()) {
      m_scheduler.schedule(false, new IntakeRedBallOutCommand)
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
