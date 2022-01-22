package frc.robot.commands;

import frc.robot.subsystems.IntakeSubSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class IntakeCommand extends CommandBase {
    private IntakeSubSystem m_intake;

    public IntakeCommand(IntakeSubSystem p_trigger, boolean p_triggers){
        m_intake = p_trigger;
    }

    public void runIntake(){
        if(){
            m_intake.setDown();
        }
        else{
           m_intake.setUp(); 
        }
    }

    
}
