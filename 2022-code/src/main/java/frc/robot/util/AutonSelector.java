package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Trajectories.TestAuton;
import frc.robot.subsystems.LimelightSubsystem;

public class AutonSelector {

    private SendableChooser<Command> m_chooser;
    private LimelightSubsystem m_limelight;
    private TestAuton m_testTraj;
    private TestAuton m_secondTestTraj; 

    public AutonSelector(LimelightSubsystem p_limelight) {
        m_chooser = new SendableChooser<>();
        m_limelight = p_limelight;
        m_testTraj = new TestAuton(m_limelight);
        m_secondTestTraj = new TestAuton(m_limelight);
        
        m_chooser.setDefaultOption("Basic Boy", m_testTraj);
        m_chooser.addOption("Secondary Basic Boy", m_secondTestTraj);
        SmartDashboard.putData(m_chooser);
    }  

    public Command getChooserSelect() {
        return m_chooser.getSelected();
    }
}










