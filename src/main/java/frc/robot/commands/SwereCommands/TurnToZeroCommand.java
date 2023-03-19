// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwereCommands;

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
  private boolean rightPosition;
  private Timer timer = new Timer();
  protected Gains gains = new Gains("gains r", 0.08, 0, 0.4);

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
    rightPosition = true;
    timer.reset();
    pid.setTargetPosition(0);
    pid.setMaxOutput(Constants.SwerveConst.maxSpeed * 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("inside " + Swerve.gyro.getYaw());
    double Output = pid.getOutput(Swerve.gyro.getYaw());
    Output += 0.1 * Constants.SwerveConst.maxAngularVelocity * Math.signum(Output);
    if(Math.abs(Swerve.gyro.getYaw()) > 1.5){
      swerve.drive(new Translation2d(0.046, 0.046), Output, false, true);
      timer.reset();
      rightPosition = true;
    }
    else if(Math.abs(Swerve.gyro.getYaw()) < 1.5 && rightPosition){
      System.out.println("timer started!! ");
      timer.start();
      rightPosition = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.3);
  }
}
