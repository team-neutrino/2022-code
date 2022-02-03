package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Trajectories.BasicBoy;
import frc.robot.subsystems.LimelightSubsystem;

public class AutonSelector {

    private SendableChooser<Command> m_chooser;
    private LimelightSubsystem m_limelight;
    private BasicBoy m_basicBoy;
    private BasicBoy m_secondBasicBoy; 

    public AutonSelector(LimelightSubsystem p_limelight) {
        m_limelight = p_limelight;
        m_basicBoy = new BasicBoy(m_limelight);
        m_secondBasicBoy = new BasicBoy(m_limelight);
    }  

    public Command getChooserSelect() {
        m_chooser = new SendableChooser<>();
        SmartDashboard.putData(m_chooser);
        return m_chooser.getSelected();
    }
}










