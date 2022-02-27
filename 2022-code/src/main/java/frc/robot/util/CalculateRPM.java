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

    m_distanceRPMData.put(1.1, 2650.0);
    m_distanceRPMData.put(1.3, 2800.0);
    m_distanceRPMData.put(1.52, 2900.0);
    m_distanceRPMData.put(1.78, 2950.0);
    m_distanceRPMData.put(2.0, 3300.0);
    m_distanceRPMData.put(2.5, 3650.0);
    m_distanceRPMData.put(2.8, 3800.0);
    m_distanceRPMData.put(3.0, 3900.0);
    m_distanceRPMData.put(3.5, 4200.0);
    m_distanceRPMData.put(3.8, 4350.0);
    m_distanceRPMData.put(4.0, 4500.0);
    m_distanceRPMData.put(4.2, 4700.0);
    m_distanceRPMData.put(4.66, 5350.0);
    m_distanceRPMData.put(4.9, 5350.0);
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
