package frc.robot.commands;
import frc.robot.subsystems.IntakeSubSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeCommand extends CommandBase {
    private IntakeSubSystem m_intake;
    private boolean m_isOn;

    public IntakeCommand(IntakeSubSystem subsystem, boolean p_isOn){
        m_intake = subsystem;
        m_isOn = p_isOn;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_isOn) {
            m_intake.setDown();
            m_intake.setIntakeOn();
        }
        else {
            m_intake.setUp();
            m_intake.setIntakeOff();
        }
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