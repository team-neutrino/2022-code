package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUPCommand extends CommandBase {
    private ClimberSubsystem m_climberSubsystem;
    
    public ClimbUPCommand (ClimberSubsystem subsystem)
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
        m_climberSubsystem.extendClimber();
      }
      
      @Override
      public void end(boolean interrupted) 
      {
        m_climberSubsystem.climberOff();
      }
    
      @Override
      public boolean isFinished() 
      {
        return false;
      }

}
