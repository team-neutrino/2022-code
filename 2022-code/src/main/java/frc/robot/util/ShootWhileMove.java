package frc.robot.util;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class ShootWhileMove {
    
    TurretPIDSubsystem m_Turret;
    LimelightSubsystem m_Limelight;

    public ShootWhileMove(TurretPIDSubsystem p_Turret, LimelightSubsystem p_Limelight){

        m_Turret = p_Turret;
        m_Limelight = p_Limelight;

    }

    

}
