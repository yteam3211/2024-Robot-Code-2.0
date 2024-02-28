
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands.ShootingWheelsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootingVelocity extends Command {
  private ShootingSubsystem shootingSubsystem;
  private double velocity;
  public ShootingVelocity(ShootingSubsystem shootingSubsystem, double velocity) {
    this.shootingSubsystem = shootingSubsystem;
    this.velocity = velocity;
    addRequirements(shootingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******** inside ShootingOutput");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootingSubsystem.setShooterVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootingSubsystem.setShooterOutput(0);
    System.out.println("********exit ShootingOutput");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
