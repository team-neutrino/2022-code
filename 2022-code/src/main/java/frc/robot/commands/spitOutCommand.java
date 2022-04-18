package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class spitOutCommand extends CommandBase {
  private IntakeSubSystem m_intake;
  private ColorSubsystem m_colorSensor;
  private IndexSubsystem m_index;
  private Timer m_timer = new Timer();
  public spitOutCommand(
      IntakeSubSystem p_intake, ColorSubsystem p_colorSensor, IndexSubsystem p_index) {
    m_intake = p_intake;
    m_colorSensor = p_colorSensor;
    m_index = p_index;
    addRequirements(p_intake, p_colorSensor, p_index);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.get() == 0) {
        m_intake.setUp();
        m_intake.setIntakeOff();
        m_timer.stop();
        m_timer.reset();
    }
    
    if (m_colorSensor.error() && m_timer.get() == 0) {
        m_timer.start();
    }
    else if (m_timer.get() >= 3) {
        m_timer.stop();
        m_timer.reset(); 
    }
    else if (m_timer.get() >= 0.1) {
        m_index.MotorOneReverse();
        m_index.MotorTwoReverse();
        m_intake.setDown();
        m_intake.setIntakeReverse();
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
