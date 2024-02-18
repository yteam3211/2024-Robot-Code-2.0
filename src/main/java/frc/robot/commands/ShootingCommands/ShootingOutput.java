// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootingOutput extends Command {
  private ShootingSubsystem shootingSubsystem;
  private double output;
  public ShootingOutput(ShootingSubsystem shootingSubsystem, double output) {
    this.shootingSubsystem = shootingSubsystem;
    this.output = output;
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
    shootingSubsystem.setShooterOutput(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("********exit ShootingOutput");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
