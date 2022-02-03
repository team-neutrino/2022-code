package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbRetractCommand extends CommandBase {
    private ClimberSubsystem m_climberSubsystem;

    public ClimbRetractCommand(ClimberSubsystem subsystem) 
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
      m_climberSubsystem.retractClimber();
    }

    @Override
    public void end(boolean interrupted) 
    {
    }
  
    @Override
    public boolean isFinished() 
    {
       return m_climberSubsystem.getLimitSwitch();
    }
}
