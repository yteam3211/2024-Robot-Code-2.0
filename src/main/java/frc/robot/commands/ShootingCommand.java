// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.shootingSubsystem;

public class ShootingCommand extends CommandBase {
  private final shootingSubsystem shootingSubsystem;
  private final CartridgeSubsystem cartridgeSubsystem;
  private double velocity;
  private double RPM;
  private Timer timer = new Timer();
  /** Creates a new ShootingCommand. */
  public ShootingCommand(shootingSubsystem shootingSubsystem, CartridgeSubsystem cartridgeSubsystem, double velocity, double RPM) {
  this.shootingSubsystem = shootingSubsystem;
  this.cartridgeSubsystem = cartridgeSubsystem;
  this.velocity = velocity;
  this.RPM = RPM;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootingSubsystem.setVelocity(velocity);
    if (shootingSubsystem.GetVelocity() >= RPM) {
      cartridgeSubsystem.setOutput(velocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
