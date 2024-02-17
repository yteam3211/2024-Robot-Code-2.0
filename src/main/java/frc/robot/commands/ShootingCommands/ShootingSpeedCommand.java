// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootingSpeedCommand extends Command {
  /** Creates a new ShooterWheelsCommand. */
  private ShootingSubsystem shootingSubsystem;
  private KickerSubsystem kickerSubsystem;
  private double shootingVelocity;
  private double kickerOutput;
  public ShootingSpeedCommand(ShootingSubsystem shootingSubsystem, KickerSubsystem kickerSubsystem,double shootingVelocity,double kickerOutput) {
    this.shootingSubsystem = shootingSubsystem;
    this.kickerSubsystem = kickerSubsystem;
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
  public void execute() { 
     shootingSubsystem.setShooterVelocity(shootingVelocity);
    //  System.out.println("out " + shootingSubsystem.getVelocity());
    if ((shootingVelocity - 2000) < shootingSubsystem.getVelocity()) {
      // System.out.println("in " + shootingSubsystem.getVelocity());
      kickerSubsystem.setKickerOutput(kickerOutput);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootingSubsystem.setShooterOutput(0);
    kickerSubsystem.setKickerOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   if (Math.abs(shootingVelocity - shootingSubsystem.getVelocity()) < Constants.SHOOTING_VELOCITY_TRESHOLD) {
  //     return true;
  //   }
  //   else
  //   {
  //     return false;

  //   }
  return false;
  }
}
