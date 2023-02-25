// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.armSubsystem;


public class ArmOutputCommand extends CommandBase {
  private final armSubsystem armSubsystem;
  // private DoubleSupplier output;
  private double output;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmOutputCommand(armSubsystem armSubsystem, double output) {
    this.armSubsystem = armSubsystem;
    this.output = output;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // armSubsystem.setArmOutput(output.getAsDouble() / 3);
    armSubsystem.setArmOutput(output);
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
