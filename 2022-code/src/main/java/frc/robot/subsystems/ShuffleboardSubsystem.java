// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.Locale;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private ShuffleboardTab m_debugTab;
  private NetworkTableEntry m_setShooterRPM;
  private NetworkTableEntry m_shooterRPMGraph;
  private NetworkTableEntry m_turretPositionGraph;
  private ShooterSubsystem m_shooter;
  private HttpCamera LLFeed;
  private NetworkTableEntry m_turretAngle;
  private TurretPIDSubsystem m_turret;
  private NetworkTableEntry m_timer;
  private ClimberSubsystem m_climber;
  private DriveTrainSubsystem m_drivetrain;
  private IndexSubsystem m_index;
  private LimelightSubsystem m_limelight;
  private NetworkTableEntry m_driveVariables[] = new NetworkTableEntry[7];
  private NetworkTableEntry m_climberVariables[] = new NetworkTableEntry[3];
  private NetworkTableEntry m_indexVariables[] = new NetworkTableEntry[2];
  private NetworkTableEntry m_limelightVariables[] = new NetworkTableEntry[5];
  private NetworkTableEntry m_shooterVariables[] = new NetworkTableEntry[2];
  private NetworkTableEntry m_shooterPID[] = new NetworkTableEntry[3];
  private NetworkTableEntry m_turretPID[] = new NetworkTableEntry[3];

  /** Creates a new shuffleboard. */
  public ShuffleboardSubsystem(
      ShooterSubsystem p_shooter,
      TurretPIDSubsystem p_turret,
      ClimberSubsystem p_climber,
      DriveTrainSubsystem p_drivetrain,
      IndexSubsystem p_index,
      LimelightSubsystem p_limelight) {
    m_shooter = p_shooter;
    m_turret = p_turret;
    m_climber = p_climber;
    m_drivetrain = p_drivetrain;
    m_index = p_index;
    m_limelight = p_limelight;

    driveStationTab();
    debugTab();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_timer.setDouble(DriverStation.getMatchTime());
    m_shooterVariables[0].setDouble(m_shooter.getRPM1());
    m_shooterVariables[1].setDouble(m_shooter.getRPM2());
    m_shooterRPMGraph.setDouble(m_shooter.getRPM1());
    m_shooter.setP(m_shooterPID[0].getDouble(0.0));
    m_shooter.setI(m_shooterPID[1].getDouble(0.0));
    m_shooter.setD(m_shooterPID[2].getDouble(0.0));

    m_turretAngle.setDouble(m_turret.getCurrentAngle());
    m_turretPositionGraph.setDouble(m_turret.getCurrentAngle());
    m_turret.setP(m_turretPID[0].getDouble(0.0));
    m_turret.setI(m_turretPID[1].getDouble(0.0));
    m_turret.setD(m_turretPID[2].getDouble(0.0));

    m_driveVariables[0].setDouble(m_drivetrain.getDriveEncoder1());
    m_driveVariables[1].setDouble(m_drivetrain.getDriveEncoder2());
    m_driveVariables[2].setDouble(m_drivetrain.getDriveEncoder3());
    m_driveVariables[3].setDouble(m_drivetrain.getDriveEncoder4());
    m_driveVariables[4].setString(String.format("%,.2f", m_drivetrain.getNavX()));
    m_driveVariables[5].setString(String.format("%,.2f", m_drivetrain.getNavY()));
    m_driveVariables[6].setString(String.format("%,.2f", m_drivetrain.getNavYaw()));

    m_climberVariables[0].setDouble(m_climber.getClimbEncoderOne());
    m_climberVariables[1].setDouble(m_climber.getClimbEncoderTwo());
    m_climberVariables[2].setBoolean(m_climber.getLimitSwitch());

    m_indexVariables[0].setDouble(m_index.getIndexEncoder1());
    m_indexVariables[1].setBoolean(m_index.getBeamBreak());

    m_limelightVariables[0].setString(String.format("%,.2f", m_limelight.getTx()));
    m_limelightVariables[1].setString(String.format("%,.2f", m_limelight.getTy()));
    m_limelightVariables[2].setString(String.format("%,.2f", m_limelight.getTa()));
    m_limelightVariables[3].setBoolean(m_limelight.getTv());
    m_limelightVariables[4].setString(String.format("%,.2f", m_limelight.getDistance()));
  }

  public double getTestRPM() {
    return m_setShooterRPM.getDouble(0.0);
  }

  public void driveStationTab() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");
    m_shooterVariables[0] =
        m_drivestationTab
            .add("Shooter RPM", 0)
            .withPosition(0, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 6000))
            .getEntry();
    m_setShooterRPM = m_drivestationTab.add("Set Shooter RPM", 0).withPosition(2, 0).getEntry();
    m_timer = m_drivestationTab.add("Match Time", 0).withPosition(0, 5).withSize(8, 1).getEntry();

    try {
      LLFeed =
          new HttpCamera(
              "limelight", "http://limelight.local:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
      CameraServer.startAutomaticCapture(LLFeed);
      m_drivestationTab
          .add(LLFeed)
          .withPosition(0, 2)
          .withSize(3, 3)
          .withWidget(BuiltInWidgets.kCameraStream);
      m_drivestationTab
          .add(CameraServer.startAutomaticCapture())
          .withPosition(3, 0)
          .withSize(5, 5)
          .withWidget(BuiltInWidgets.kCameraStream);
    } catch (VideoException e) {
    }
  }

  public void debugTab() {
    m_debugTab = Shuffleboard.getTab("Debug Tab");

    m_shooterVariables[0] =
        m_debugTab
            .add("Shooter RPM", 0)
            .withPosition(0, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 6000))
            .getEntry();
    m_shooterVariables[1] =
        m_debugTab
            .add("Shooter RPM2", 0)
            .withPosition(2, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 6000))
            .getEntry();
    m_shooterRPMGraph =
        m_debugTab
            .add("Shooter RPM v Time", 0)
            .withPosition(0, 2)
            .withSize(3, 2)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
    m_shooterPID[0] =
        m_debugTab.add("Shooter P", m_shooter.getP()).withPosition(3, 2).withSize(1, 1).getEntry();
    m_shooterPID[1] =
        m_debugTab.add("Shooter I", m_shooter.getI()).withPosition(3, 3).withSize(1, 1).getEntry();
    m_shooterPID[2] =
        m_debugTab.add("Shooter D", m_shooter.getD()).withPosition(3, 4).withSize(1, 1).getEntry();

    m_driveVariables[0] =
        m_debugTab.add("DriveRMotor1", 0).withPosition(7, 2).withSize(1, 1).getEntry();
    m_driveVariables[1] =
        m_debugTab.add("DriveRMotor2", 0).withPosition(8, 2).withSize(1, 1).getEntry();
    m_driveVariables[2] =
        m_debugTab.add("DriveLMotor1", 0).withPosition(7, 3).withSize(1, 1).getEntry();
    m_driveVariables[3] =
        m_debugTab.add("DriveLMotor2", 0).withPosition(5, 2).withSize(1, 1).getEntry();
    m_driveVariables[4] = m_debugTab.add("Navx X", "Navx X1").withPosition(8, 4).withSize(1, 1).getEntry();
    m_driveVariables[5] = m_debugTab.add("Navx Y", "Navx Y1").withPosition(9, 4).withSize(1, 1).getEntry();
    m_driveVariables[6] =
        m_debugTab.add("Navx Yaw", "Navx Yaw1"  ).withPosition(10, 4).withSize(1, 1).getEntry();

    m_climberVariables[0] =
        m_debugTab.add("Climber 1", 0).withPosition(9, 0).withSize(1, 1).getEntry();
    m_climberVariables[1] =
        m_debugTab.add("Climber 2", 0).withPosition(10, 0).withSize(1, 1).getEntry();
    m_climberVariables[2] =
        m_debugTab.add("Climber Switch", 0).withPosition(11, 0).withSize(1, 1).getEntry();

    m_turretAngle = m_debugTab.add("TurretAngle", 6).withPosition(7, 0).withSize(1, 1).getEntry();
    m_turretPositionGraph =
        m_debugTab
            .add("Turret Angle v Time", 0)
            .withPosition(4, 1)
            .withSize(3, 2)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
    m_turretPID[0] =
        m_debugTab.add("Turret P", m_turret.getP()).withPosition(4, 0).withSize(1, 1).getEntry();
    m_turretPID[1] =
        m_debugTab.add("Turret I", m_turret.getI()).withPosition(5, 0).withSize(1, 1).getEntry();
    m_turretPID[2] =
        m_debugTab.add("Turret D", m_turret.getD()).withPosition(6, 0).withSize(1, 1).getEntry();

    m_indexVariables[0] =
        m_debugTab.add("Index Motor 2", 0).withPosition(5, 4).withSize(1, 1).getEntry();
    m_indexVariables[1] =
        m_debugTab.add("Index Beambreak", 0).withPosition(6, 4).withSize(1, 1).getEntry();

    m_limelightVariables[0] =
        m_debugTab.add("Limelight Tx", "Limelight Tx1").withPosition(10, 2).withSize(1, 1).getEntry();
    m_limelightVariables[1] =
        m_debugTab.add("Limelight Ty", "Limelight Ty1").withPosition(11, 2).withSize(1, 1).getEntry();
    m_limelightVariables[2] =
        m_debugTab.add("Limelight Ta", "Limelight Ta1").withPosition(12, 2).withSize(1, 1).getEntry();
    m_limelightVariables[3] =
        m_debugTab.add("Limelight Tv", 0).withPosition(10, 3).withSize(1, 1).getEntry();
    m_limelightVariables[4] =
        m_debugTab.add("Limelight Distance", "Limelight Distance1").withPosition(11, 3).withSize(1, 1).getEntry();
  }
}
