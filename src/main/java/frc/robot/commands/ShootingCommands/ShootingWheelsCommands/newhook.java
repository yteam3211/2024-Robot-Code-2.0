// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands.ShootingWheelsCommands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;

public class newhook extends Command {
  private ShootingSubsystem shootingSubsystem;
  private double pos;
  public newhook(ShootingSubsystem shootingSubsystem, double pos) {
    this.shootingSubsystem = shootingSubsystem;
    this.pos = pos;
    addRequirements(shootingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootingSubsystem.setMode(NeutralMode.Brake);
    System.out.println("******** inside ShootingOutput");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootingSubsystem.setShooterPos(shootingSubsystem.getPosition() + pos);
    System.out.println("*********** pos " + shootingSubsystem.getPosition() + "*************");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
