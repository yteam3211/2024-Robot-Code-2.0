// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectSubsyste;

public class setPointCollectCommand extends CommandBase {
  private final CollectSubsyste collectSubsyste;
  private double point;
  /** Creates a new setPoitCollectCommand. */
  public setPointCollectCommand(CollectSubsyste collectSubsyste,double point) {
    this.collectSubsyste = collectSubsyste;
    this.point = point;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collectSubsyste);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectSubsyste.setPosition(point);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("123456789", point);
    collectSubsyste.setPosition(point);
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
