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

  public ShooterControlSubsystem(LimelightSubsystem p_limelight) {

    m_limelight = p_limelight;

    m_distanceRPMData.put(3.5, 200.0);
  }

  public double InterpolateDistance() {

    boolean foundTheSmallOne = false;
  for (Double a : m_distanceRPMData.keySet())
  {
    if (a >= m_limelight.getDistance())
    {
      if(false == foundTheSmallOne)
      {
      a = m_smallerDistance;
      foundTheSmallOne = true;
      }
      else
      {
        bigOne = a;
        break;
      }

    }
  
  }
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
