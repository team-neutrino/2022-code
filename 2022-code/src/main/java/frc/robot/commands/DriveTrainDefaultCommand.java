package frc.robot.commands;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

public class DriveTrainDefaultCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrainSubsystem;
    private Joystick m_rightJoystick;
    private Joystick m_leftJoystick;

public DriveTrainDefaultCommand(DriveTrainSubsystem subsystem, Joystick p_rightJoystick, Joystick p_leftJoystick) {
    m_driveTrainSubsystem = subsystem;
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;
    addRequirements(subsystem);
}
@Override
public void initialize() {
}

@Override
public void execute() {
 m_driveTrainSubsystem.setMotors(m_leftJoystick.getY(), m_rightJoystick.getY());
}
@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
  return false;
}
}
