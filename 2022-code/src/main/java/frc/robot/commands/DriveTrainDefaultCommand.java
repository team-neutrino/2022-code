package frc.robot.commands;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainDefaultCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrainSubsystem;

public DriveTrainDefaultCommand(DriveTrainSubsystem subsystem) {
    m_driveTrainSubsystem = subsystem;
    addRequirements(subsystem);
}
@Override
public void initialize() {
}

@Override
public void execute() {
 m_driveTrainSubsystem.setMotors(m_driveTrainSubsystem.leftJoystickPosition(), m_driveTrainSubsystem.rightJoystickPosistion());
}
@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
  return false;
}
}
