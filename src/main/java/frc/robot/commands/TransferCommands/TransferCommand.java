// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TransferCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class TransferCommand extends Command {
  TransferSubsystem transferSubsystem;
  double output;

  /** Creates a new TransferCommand. */
  public TransferCommand(TransferSubsystem transferSubsystem, double output) {
    this.transferSubsystem = transferSubsystem;
    this.output = output;
    addRequirements(transferSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******** inside TransferCommand");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transferSubsystem.setOutput(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if(!KickerSubsystem.isNoteIn()){
      transferSubsystem.setOutput(0);
      System.out.println("********exit TransferCommand");
    // }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
