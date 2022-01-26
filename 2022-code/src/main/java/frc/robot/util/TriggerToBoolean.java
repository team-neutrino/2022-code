/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Add your docs here.
 */
public class TriggerToBoolean extends Trigger
{
    GenericHID m_Joystick;
    int m_JoystickID;
    double m_Threshold;

    public TriggerToBoolean(GenericHID p_Joystick, int p_JoystickID, double p_Threshold)
    {
        m_Joystick = p_Joystick;
        m_JoystickID = p_JoystickID;
        m_Threshold = p_Threshold;
    }

    @Override
    public boolean get()
    {
        if (m_Joystick.getRawAxis(m_JoystickID) > m_Threshold)
            return true;
        else
            return false;
    }
}