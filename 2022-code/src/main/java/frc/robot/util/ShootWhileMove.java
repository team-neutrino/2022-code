package frc.robot.util;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class ShootWhileMove {
    
    TurretPIDSubsystem m_turret;
    LimelightSubsystem m_limelight;
    double LIMELIGHT_MULTIPLICATION = 10.0;

    public ShootWhileMove(TurretPIDSubsystem p_turret, LimelightSubsystem p_limelight){

        m_turret = p_turret;
        m_limelight = p_limelight;

    }

    public double feedfowardTanV(){
        double deltaTx = m_limelight.getDeltaTx() * LIMELIGHT_MULTIPLICATION;
        double deltaA = m_turret.getDeltaA();
        double distance = m_limelight.getDistance();
        double tanV = (deltaTx / 0.2 + deltaA / 0.2) * distance;
    }

}
