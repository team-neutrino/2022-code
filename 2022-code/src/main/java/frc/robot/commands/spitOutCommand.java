package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ColorSubsystem;

public class spitOutCommand extends CommandBase {
  private IntakeSubSystem m_intake;
  private ColorSubsystem m_colorSensor;
  private IndexSubsystem m_index;

  public spitOutCommand(IntakeSubSystem p_intake, ColorSubsystem p_colorSensor, IndexSubsystem p_index) {
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
    m_intake.setUp();
    m_intake.setIntakeOff();
    if (m_colorSensor.error() == true){
    m_index.MotorOneReverse();  
    m_index.MotorTwoReverse();
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
