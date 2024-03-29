package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainDefaultCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private Joystick m_rightJoystick;
  private Joystick m_leftJoystick;

  public DriveTrainDefaultCommand(
      DriveTrainSubsystem subsystem, Joystick p_leftJoystick, Joystick p_rightJoystick) {
    m_driveTrainSubsystem = subsystem;
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_driveTrainSubsystem.setMotors(
        squareDrive(m_leftJoystick.getY()), squareDrive(m_rightJoystick.getY()));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public double squareDrive(double joystickVal) {
    final double SCALE = 1.2;
    return Math.pow(joystickVal, 2) * ((joystickVal < 0) ? -SCALE : SCALE);
  }
}
