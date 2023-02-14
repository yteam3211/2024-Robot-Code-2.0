// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CollectSubsyste;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.armSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class resetCommand extends CommandBase {
  private final ShootingSubsystem shootingSubsystem;
  private final CollectSubsyste collectSubsyste;
  private final armSubsystem armSubsystem;

  
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public resetCommand(ShootingSubsystem shootingSubsystem, CollectSubsyste collectSubsyste,  armSubsystem armSubsystem) {
    this.shootingSubsystem = shootingSubsystem;
    this.collectSubsyste = collectSubsyste;
    this.armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootingSubsystem,collectSubsyste);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootingSubsystem.resetEncoder();
    collectSubsyste.reSetEncoder();
    armSubsystem.resetEncoder();
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
