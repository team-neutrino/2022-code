// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.LimelightSubsystem;
import java.util.*;

/** Add your docs here. */
public class CalculateRPM {

  TreeMap<Double, Double> m_distanceRPMData = new TreeMap<Double, Double>();

  LimelightSubsystem m_limelight;

  public CalculateRPM(LimelightSubsystem p_limelight) {

    m_limelight = p_limelight;

    m_distanceRPMData.put(.999, 2000.0);
    m_distanceRPMData.put(1.55, 2300.0);
    m_distanceRPMData.put(2.05, 2500.0);
    m_distanceRPMData.put(2.50, 2850.0);
    m_distanceRPMData.put(3.00, 2980.0);
    m_distanceRPMData.put(3.00, 2980.0);
    m_distanceRPMData.put(6.90, 5000.0);


  }

  public double InterpolateDistance() {

    double smallerDistance = 0.0;
    double largerDistance = 0.0;
    double rpm = 0.0;

    double limeLightDistance = m_limelight.getDistance();

    if (limeLightDistance <= m_distanceRPMData.firstKey()) {
      return m_distanceRPMData.get(m_distanceRPMData.firstKey());
    } else if (limeLightDistance >= m_distanceRPMData.lastKey()) {
      return m_distanceRPMData.get(m_distanceRPMData.lastKey());
    } else {
      for (Double a : m_distanceRPMData.keySet()) {
        if (a >= limeLightDistance) {
          largerDistance = a;
          break;
        } else {
          smallerDistance = a;
        }
      }
    }

    rpm =
        m_distanceRPMData.get(smallerDistance)
            + ((limeLightDistance - smallerDistance))
                * ((m_distanceRPMData.get(largerDistance) - m_distanceRPMData.get(smallerDistance))
                    / (largerDistance - smallerDistance));

    return rpm;
  }
}
