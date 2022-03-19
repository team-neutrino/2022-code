// package frc.robot.util;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.Autonomi.TwoBall.TwoBallAuton;
// import frc.robot.commands.Trajectories.TestTrajectory;
// import frc.robot.subsystems.DriveTrainSubsystem;
// import frc.robot.subsystems.IndexSubsystem;
// import frc.robot.subsystems.IntakeSubSystem;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.TurretPIDSubsystem;

// public class AutonSelector {

//   private SendableChooser<Command> m_chooser;
//   private LimelightSubsystem m_limelight;
//   private TestTrajectory m_testTraj;
//   private TestTrajectory m_secondTestTraj;
//   private TwoBallAuton m_twoBallAuton;

//   public AutonSelector(
//       DriveTrainSubsystem p_drive,
//       TurretPIDSubsystem p_turret,
//       IntakeSubSystem p_intake,
//       IndexSubsystem p_index,
//       ShooterSubsystem p_shooter,
//       LimelightSubsystem p_limelight) {
//     m_chooser = new SendableChooser<>();
//     m_limelight = p_limelight;
//     m_testTraj = new TestTrajectory(m_limelight);
//     m_secondTestTraj = new TestTrajectory(m_limelight);
//     m_twoBallAuton = new TwoBallAuton(p_drive, p_turret, p_intake, p_index, p_shooter,
// p_limelight);

//     m_chooser.setDefaultOption("Basic Boy", m_testTraj);
//     m_chooser.addOption("Secondary Basic Boy", m_secondTestTraj);
//     m_chooser.addOption("TwoBall Auton", m_twoBallAuton);
//     SmartDashboard.putData(m_chooser);
//   }

//   public Command getChooserSelect() {
//     return m_chooser.getSelected();
//   }
// }
