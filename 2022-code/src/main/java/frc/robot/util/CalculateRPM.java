// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.LimelightSubsystem;
import java.util.*;
wqegjqygeyjqw
/** Add your docs here. */
public class CalculateRPM {

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
