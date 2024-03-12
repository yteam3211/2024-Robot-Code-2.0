// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwereCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;
import frc.util.vision.Limelight;

public class SwerveForward extends Command {
  /** Creates a new TurnSwerveCommand. */
  private Swerve swerve;
  private double output;

  public SwerveForward(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }
     
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("********inside SwerveForward");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(new Translation2d(0.05, 0), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("********exit SwerveForward");
    swerve.drive(new Translation2d(0.0, 0.0), 0, true);
    swerve.lockWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}