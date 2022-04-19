package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class ClimbRetractCommand extends CommandBase {
  private ClimberSubsystem m_climberSubsystem;
  private ShooterSubsystem m_shooter;
  private TurretPIDSubsystem m_turret;

  public ClimbRetractCommand(
      ClimberSubsystem p_climber, ShooterSubsystem p_shooter, TurretPIDSubsystem p_turret) {
    m_climberSubsystem = p_climber;
    m_shooter = p_shooter;
    m_turret = p_turret;
    addRequirements(m_climberSubsystem, m_shooter, m_turret);
  }

  @Override
  public void initialize() {
    m_shooter.turnOff();
  }

  @Override
  public void execute() {
    m_turret.setPower(0);
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
