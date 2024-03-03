// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands.KickerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class DefaultKicker extends Command {
  /** Creates a new kickerCommand. */
  private KickerSubsystem kickerSubsystem;
  private double kickerOutput;

  public DefaultKicker(KickerSubsystem kickerSubsystem, double kickerOutput) {
  this.kickerSubsystem = kickerSubsystem;
  this.kickerOutput = kickerOutput;
  addRequirements(kickerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******** inside DeafaultKicker");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!KickerSubsystem.isNoteIn()){
      // System.out.println("******** DeafaultKicker Set Output");
      kickerSubsystem.setKickerOutput(kickerOutput);
    }
    else kickerSubsystem.setKickerOutput(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // System.out.println("********exit deafaultKicker");


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
