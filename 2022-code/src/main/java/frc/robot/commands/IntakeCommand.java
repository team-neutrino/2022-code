package frc.robot.commands;
import frc.robot.subsystems.IntakeSubSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;


public class IntakeCommand extends CommandBase {
    private IntakeSubSystem m_intake;

    public IntakeCommand(IntakeSubSystem subsystem){
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
        m_intake.setDown();

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