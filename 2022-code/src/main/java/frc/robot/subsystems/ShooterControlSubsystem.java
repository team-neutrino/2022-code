// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import frc.robot.subsystems.LimelightSubsystem;

public class ShooterControlSubsystem extends SubsystemBase {
  /** Creates a new ShooterControlSubsystem. */

  TreeMap<Double, Double> m_distanceRPMData = new TreeMap<Double, Double>();
  LimelightSubsystem m_limelight;
  Double m_smallerDistance;
  Double m_largerDistance;
  Double RPM=420.0;

  public ShooterControlSubsystem(LimelightSubsystem p_limelight) {

    m_limelight = p_limelight;

    m_distanceRPMData.put(1.0, 100.0);
    m_distanceRPMData.put(2.0, 200.0);
  }

  public double InterpolateDistance() {

    boolean foundTheSmallOne = false;

    for (Double a : m_distanceRPMData.keySet())
    {
      if (a >= 1.5)
      {
        if(false == foundTheSmallOne)
        {
          a = m_smallerDistance;
          foundTheSmallOne = true;
        }
        else
        {
          m_largerDistance = a;
          break;
        }
      }
    }
// replace constant with limelight distance 
System.out.println(m_smallerDistance);
 RPM = m_distanceRPMData.get(m_smallerDistance);
  
/*  + ((1.5 -  m_smallerDistance)) * 
    
  ( (m_distanceRPMData.get(m_largerDistance) - m_distanceRPMData.get(m_smallerDistance)) / ( m_largerDistance - m_smallerDistance)); */
  
  System.out.println(RPM);
  return RPM;

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(RPM);
  }
}
