// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSubsystem extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch m_colorMatcher = new ColorMatch();
  private boolean m_isBlue;
  private boolean m_isRed;

  private final Color K_BLUE = new Color(0.145, 0.586, 0.742);
  private final Color K_RED = new Color(0.898, 0.277, 0.172);
  private final int K_IS_BALL_THRESHOLD = 250;

  /** Creates a new ColorSubsystem. */
  public ColorSubsystem() {
    m_colorMatcher.addColorMatch(K_BLUE);
    m_colorMatcher.addColorMatch(K_RED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = getSensorColor();
    m_isBlue = (isBlue(detectedColor));
    m_isRed = (isRed(detectedColor));
  }

  public int getProximity() {
    return m_colorSensor.getProximity();
  }

  public boolean isBall() {
    return getProximity() > K_IS_BALL_THRESHOLD;
  }

  public boolean getIsBlue() {
    return m_isBlue;
  }

  public boolean getIsRed() {
    return m_isRed;
  }

  public Color getSensorColor() {
    return m_colorSensor.getColor();
  }

  public boolean isBlue(Color detectedColor) {
    ColorMatchResult matchResult = m_colorMatcher.matchClosestColor(detectedColor);
    boolean isBlue = false;
    if (matchResult.color == K_BLUE) isBlue = true;
    return isBlue;
  }

  public boolean isRed(Color detectedColor) {
    ColorMatchResult matchResult = m_colorMatcher.matchClosestColor(detectedColor);
    boolean isRed = false;
    if (matchResult.color == K_RED) isRed = true;
    return isRed;
  }
}
