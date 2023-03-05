// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.armCollectSubsystem;

public class setPointCollectCommand extends CommandBase {
  private final CollectSubsystem collectSubsystem;
  // private final armCollect armCollect;
  private double point;
  /** Creates a new setPoitCollectCommand. */
  public setPointCollectCommand(CollectSubsystem collectSubsyste, double point) {
    this.collectSubsystem = collectSubsyste;
    this.point = point;
    // this.armCollect = armCollect;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collectSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectSubsystem.setPosition(point);
    collectSubsystem.setPosition(point);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("123456789", point);
    collectSubsystem.setPosition(point);
    collectSubsystem.setPosition(point);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectSubsystem.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
