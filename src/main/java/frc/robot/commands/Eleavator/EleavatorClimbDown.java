// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Eleavator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.gains;

public class EleavatorClimbDown extends Command {
  private ElevatorSubsystem eleavatorSubsystem;
  private double eleavatorPosition;
  /** Creates a new Eleavator. */
  public EleavatorClimbDown(ElevatorSubsystem eleavatorSubsystem ,double eleavatorPosition) {
    this.eleavatorSubsystem = eleavatorSubsystem;
    this.eleavatorPosition =eleavatorPosition;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    eleavatorSubsystem.changeStation(gains.EleavatorClimbDowm);
    System.out.println("******** inside EleavatorClimbDown");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    eleavatorSubsystem.setPosition(eleavatorPosition);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("********exit EleavatorClimbDown");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
}
