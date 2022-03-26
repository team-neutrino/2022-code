package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainDefaultCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private final XboxController m_xBox;

  public DriveTrainDefaultCommand(DriveTrainSubsystem subsystem, XboxController p_xBox) {
    m_driveTrainSubsystem = subsystem;
    m_xBox = p_xBox;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_driveTrainSubsystem.setMotors(m_xBox.getLeftX(), m_xBox.getRightY());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
