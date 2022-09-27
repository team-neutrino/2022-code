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

    /*conversion needed from encoder counts to some actual unit. Right now, the deltaA is in encoder units and the limelight
     * values are also apparently being converted by taking degrees and multiplying by 10. the change in angle by the turret
     * needs to be in radians since that is a unitless measure that actually converts to a distance. If the limelight output
     * is in degrees that should be easy enough already. This should then be added to or replaced with the input that is set
     * current angle I think somewhere in a default command. I think I had a question about that that I also wanted to ask?
     * Look into it. 
     */

    public double feedfowardTanV(){
        double deltaTx = m_limelight.getDeltaTx() * LIMELIGHT_MULTIPLICATION;
        double deltaA = m_turret.getDeltaA();
        double distance = m_limelight.getDistance();
        double tanV = (deltaTx / 0.2 + deltaA / 0.2) * distance;
    }

}
