// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConeLimelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;

public class DriveToTarget extends CommandBase {
  /** Creates a new DriveToTarget. */
  protected PIDController pidX = new PIDController();
  protected PIDController pidY = new PIDController();
  protected PIDController pidR = new PIDController();
  // protected Gains
  public DriveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
