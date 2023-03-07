// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;

public class TurnToZeroCommand extends CommandBase {
  private Swerve swerve;
  private boolean count = true;
  private Timer timer = new Timer();
  protected Gains gains = new Gains("gains r", 0.05, 0, 0);

  protected PIDController pid = new PIDController(gains);

  /** Creates a new TurnToZeroCommand. */
  public TurnToZeroCommand(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTargetPosition(0);
    pid.setMaxOutput(Constants.Swerve.maxSpeed * 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("inside " + Swerve.gyro.getYaw());
    double Output = pid.getOutput(Swerve.gyro.getYaw());
    Output += 0.1 * Constants.Swerve.maxAngularVelocity * Math.signum(Output);

    swerve.drive(new Translation2d(0.07, 0.07), Output, false, true);
  
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
