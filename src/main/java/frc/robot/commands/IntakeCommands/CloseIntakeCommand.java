// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;

public class CloseIntakeCommand extends CommandBase {
  /** Creates a new CloseIntakeCommand. */
  private CollectSubsystem collectSubsystem;
  private double output;

  public CloseIntakeCommand(CollectSubsystem collectSubsystem, double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collectSubsystem = collectSubsystem;
    addRequirements(collectSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collectSubsystem.setOutput(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectSubsystem.resetEncoder();
    collectSubsystem.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collectSubsystem.isClose();
  }
}
