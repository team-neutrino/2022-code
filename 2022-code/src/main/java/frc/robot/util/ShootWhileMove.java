package frc.robot.util;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class ShootWhileMove {
    
    TurretPIDSubsystem m_turret;
    LimelightSubsystem m_limelight;
    double LIMELIGHT_MULTIPLICATION = 10.0;
    double TX_TO_TURRET_COUNTS_CONVERSION = 165/240;

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

    public double integrateAngularV() {
        double deltaTx = m_limelight.getDeltaTx() * TX_TO_TURRET_COUNTS_CONVERSION;
        double deltaA = m_turret.getDeltaA();
        System.out.println("Delta A: " + deltaA);
        System.out.println("Delta Tx: " + deltaTx);
        double deltaTxTwo = 0.0;
        double deltaATwo = 0.0;
        double deltaTxThree = 0.0;
        double deltaAThree = 0.0;
        double deltaTxFour = 0.0;
        double deltaAFour = 0.0;
        //double deltaTxFive = 0.0;
        //double deltaAFive = 0.0;
        //deltaTxFive = deltaTxFour;
        //deltaAFive = deltaAFour;
        deltaTxFour = deltaTxThree;
        deltaAFour = deltaAThree;
        deltaTxThree = deltaTxTwo;
        deltaAThree = deltaATwo;
        deltaTxTwo = deltaTx;
        deltaATwo = deltaA;
        // double distance = m_limelight.getDistance();
        double angularVelocity = (deltaTx / 0.1) + (deltaA / 0.1);
        double angularVelocityTwo = (deltaTxTwo / 0.1) + (deltaATwo / 0.1);
        double angularVelocityThree = (deltaTxThree / 0.1) + (deltaATwo / 0.1);
        double angularVelocityFour = (deltaTxFour / 0.1) + (deltaAFour / 0.1);
        // double angularVelocityFive = (deltaTxFive / 0.2) + (deltaAFive / 0.2);
        double vAverage = (angularVelocity + angularVelocityTwo) / 2;
        double vAverageTwo = (angularVelocityThree + angularVelocityFour) / 2;
        double integrateV = (vAverage + vAverageTwo) * 0.2; 
        System.out.println(integrateV);
        return integrateV;
    }

}
