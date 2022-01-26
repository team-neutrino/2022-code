// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;

public class IndexMotorCommand extends CommandBase {
  /** Creates a new IndexMotorCommand. */
  private final IndexSubsystem index;
  public IndexMotorCommand(IndexSubsystem indexSubsystem) 
  {
    index = indexSubsystem;
    addRequirements(indexSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(index.getBeamBreak()==false)
    { 
      index.motorOneStart();
      index.MotorTwoStop();
    }
    else
    {
      index.motorOneStop();
      index.MotorTwoStart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
   {
    return false;
  }
}
