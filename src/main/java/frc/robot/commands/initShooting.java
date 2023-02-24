// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSubsystem;

public class InitShooting extends CommandBase {
  private final ShootingSubsystem shootingSubsystem;



  /** Creates a new InitShooting. */
  public InitShooting(ShootingSubsystem shootingSubsystem) {
    this.shootingSubsystem = shootingSubsystem ;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("rrrrr");
    shootingSubsystem.setOutput(-0.15);
     
      
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootingSubsystem.setOutput(0);
      shootingSubsystem.resetEncoder();
    shootingSubsystem.changeDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shootingSubsystem.GetPosition() <= shootingSubsystem.Min){
      return true;
    }
    return false;
    
  }
}
