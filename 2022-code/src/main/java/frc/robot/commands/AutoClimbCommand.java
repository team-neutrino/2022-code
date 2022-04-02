package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class AutoClimbCommand extends CommandBase {
    
    private ClimberSubsystem m_climber;
    private Timer m_timer;

    public AutoClimbCommand(ClimberSubsystem p_climber) {
        p_climber = m_climber;
        m_timer = new Timer();
        addRequirements(m_climber);
    }
    @Override
    public void initialize() {
        m_timer.start();
    }
    
    @Override
    public void execute() {
       if (m_timer.get() >= 5) {
           m_climber.extendClimberArm1();
       }
       if ( m_timer.get() <= 6) {
           m_climber.retractClimberArm1();
       }
       if (m_timer.get() <= )
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
