// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootingCommand extends Command {
  private ShootingSubsystem shootingSubsystem;
  private double shootingVelocity;
  private double kickerOutput; 
  /** Creates a new shootingCommand. */
  public ShootingCommand(ShootingSubsystem shootingSubsystem, double shootingVelocity, double kickerOutput) {
    this.shootingSubsystem = shootingSubsystem;
    this.shootingVelocity = shootingVelocity;
    this.kickerOutput = kickerOutput;
    addRequirements(shootingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    shootingSubsystem.setShooterVelocity(shootingVelocity);
    if(Math.abs(shootingVelocity - shootingSubsystem.getVelocity()) < Constants.SHOOTING_VELOCITY_TRESHOLD)
    {
      shootingSubsystem.setKickerOutput(kickerOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootingSubsystem.setKickerOutput(0);
    shootingSubsystem.setShooterVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
