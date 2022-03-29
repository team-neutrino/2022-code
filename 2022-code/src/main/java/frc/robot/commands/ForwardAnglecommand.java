package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;

public class ForwardAnglecommand extends CommandBase {

  private TurretPIDSubsystem m_turret;

  /** Creates a new TurretManualAimCommand. */
  public ForwardAnglecommand(TurretPIDSubsystem p_turret) {

    m_turret = p_turret;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_turret.setTargetAngle(300.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
