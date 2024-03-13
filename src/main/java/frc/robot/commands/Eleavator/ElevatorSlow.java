// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Eleavator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.gains;

public class ElevatorSlow extends Command {
  private ElevatorSubsystem eleavatorSubsystem;
  private Boolean up;
  /** Creates a new Eleavator. */
  public ElevatorSlow(ElevatorSubsystem eleavatorSubsystem, Boolean up) {
    this.eleavatorSubsystem = eleavatorSubsystem;
    this.up = up;
    addRequirements(eleavatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    eleavatorSubsystem.changeStation(gains.EleavatorUp);
    System.out.println("******** inside EleavatorSlow;");

    // eleavatorSubsystem.changeStation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    eleavatorSubsystem.setPosition(up ? (eleavatorSubsystem.getElevatorHight() + 10) : (eleavatorSubsystem.getElevatorHight() - 10));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("********exit EleavatorSlow");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
}
