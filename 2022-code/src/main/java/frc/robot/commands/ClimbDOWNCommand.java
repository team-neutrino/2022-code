package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDOWNCommand extends CommandBase {
    private ClimberSubsystem m_climberSubsystem;

    public ClimbDOWNCommand(ClimberSubsystem subsystem) 
    {
        m_climberSubsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() 
    {
    }
  
    @Override
    public void execute() 
    {
      m_climberSubsystem.keyUnlock();
      m_climberSubsystem.retractClimber();
    }

    @Override
    public void end(boolean interrupted) 
    {
      m_climberSubsystem.climberOff();
      m_climberSubsystem.keyLock();
    }
  
    @Override
    public boolean isFinished() 
    {
      return false;
    }
}
