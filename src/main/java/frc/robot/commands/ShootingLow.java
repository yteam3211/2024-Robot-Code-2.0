// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShootingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShootingLow extends CommandBase {
  private final ShootingSubsystem shootingSubsystem;
  double output;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootingLow(ShootingSubsystem shootingSubsystem, double output) {
    this.shootingSubsystem = shootingSubsystem;
    this.output = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootingSubsystem.setOutput(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shootingSubsystem.ShooingMotor.getPosition() > 0){
      return true;
  }
  else
      return false;  
  }
}