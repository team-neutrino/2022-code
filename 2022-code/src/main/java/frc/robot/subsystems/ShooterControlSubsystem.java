// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;

public class ShooterControlSubsystem extends SubsystemBase {
  /** Creates a new ShooterControlSubsystem. */

  TreeMap<Double, Double> m_distanceRPMData = new TreeMap<Double, Double>();
  LimelightSubsystem m_limelight;
  

  public ShooterControlSubsystem(LimelightSubsystem p_limelight) {

    m_limelight = p_limelight;

    m_distanceRPMData.put(1.0, 100.0);
    m_distanceRPMData.put(2.0, 200.0);
  }

  public double InterpolateDistance() {

    Double smallerDistance = 0.0;
    Double largerDistance = 0.0;
    Double RPM = 0.0;

  //  boolean foundTheLargeOne = false;

  double limeLightDistance = 1.5; //m_limelight.getDistance();

  if (limeLightDistance <= m_distanceRPMData.firstKey())
    {
     System.out.println(m_distanceRPMData.get(m_distanceRPMData.firstKey()));
     return m_distanceRPMData.get(m_distanceRPMData.firstKey());

    }
  else if (limeLightDistance >= m_distanceRPMData.lastKey())
    {
      System.out.println(m_distanceRPMData.get(m_distanceRPMData.lastKey()));
      return m_distanceRPMData.get(m_distanceRPMData.lastKey());
    }
  else
  { 
    for (Double a : m_distanceRPMData.keySet())
    {
      {
       if (a >= limeLightDistance)
       {
        largerDistance = a;
        System.out.println(" large value " + a);
        break;
       }
       else 
       {
        smallerDistance = a;
        System.out.println("small value " + a);
       }
      }
    }
  }
    
// replace constant with limelight distance 
// System.out.println(m_smallerDistance);
// System.out.println(m_distanceRPMData.keySet());
RPM = m_distanceRPMData.get(smallerDistance) + ((limeLightDistance - smallerDistance)) * 
( (m_distanceRPMData.get(largerDistance) - m_distanceRPMData.get(smallerDistance)) / ( largerDistance - smallerDistance));  
  
  System.out.println(RPM);
// System.out.println(m_distanceRPMData.get(m_largerDistance));
  return RPM;

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(RPM);
    // System.out.println(m_distanceRPMData.keySet());
/*
    for (Double a : m_distanceRPMData.keySet())
    {
      if (a >= 1.5) {
        System.out.println(a);
      }

    }
    */
 }
}