// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class AutoKickerCommand extends Command {
  /** Creates a new kickerCommand. */
  private KickerSubsystem kickerSubsystem;
  ShootingSubsystem shootingSubsystem;
  private double kickerOutput;

  public AutoKickerCommand(KickerSubsystem kickerSubsystem, ShootingSubsystem shootingSubsystem, double kickerOutput) {
  this.kickerSubsystem = kickerSubsystem;
  this.kickerOutput = kickerOutput;
  this.shootingSubsystem = shootingSubsystem;
  addRequirements(kickerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******** inside AutoKickerCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("velocity: " + ((Constants.SHOOTING_SPEAKER_VELCITY - 5000) < shootingSubsystem.getVelocity()));
    if((Constants.SHOOTING_SPEAKER_VELCITY - 5000) < shootingSubsystem.getVelocity()){
      kickerSubsystem.setKickerOutput(kickerOutput);
      System.out.println("set Output");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kickerSubsystem.setKickerOutput(0);
    System.out.println("******** exit AutoKickerCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("is note in: " + !kickerSubsystem.isNoteIn());
    return !kickerSubsystem.isNoteIn(); //TODO: change this to return true when the sensor is false (when the note is out)
  }
}
