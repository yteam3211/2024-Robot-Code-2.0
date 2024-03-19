// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands.KickerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class KickerOutput extends Command {
  KickerSubsystem kickerSubsystem;
  ShootingSubsystem shootingSubsystem;
  Double output;

  public KickerOutput(KickerSubsystem kickerSubsystem, ShootingSubsystem shootingSubsystem, Double output) {
    this.kickerSubsystem = kickerSubsystem;
    this.output = output;
    this.shootingSubsystem = shootingSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shootingSubsystem.setShooterOutput(0.2);
    kickerSubsystem.setKickerOutput(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kickerSubsystem.setKickerOutput(0);
    shootingSubsystem.setShooterOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
