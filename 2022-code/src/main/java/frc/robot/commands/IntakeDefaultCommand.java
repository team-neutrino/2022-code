package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeDefaultCommand extends CommandBase {
    private IntakeSubsystem m_intake;

    public IntakeDefaultCommand(IntakeSubsystem subsystem){
        m_intake = subsystem;
        addRequirements(subsystem);

    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setUp(); 
        m_intake.setIntakeOff();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }   
}
