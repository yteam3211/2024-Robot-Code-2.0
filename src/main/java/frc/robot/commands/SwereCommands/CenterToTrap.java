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

public class CenterToTrap extends Command {
  /** Creates a new TurnSwerveCommand. */
  private Swerve swerve;
  private Limelight limelight;
  protected Gains gains = new Gains("turn gains", 0.025, 0, 0.006);           
  protected PIDController pid = new PIDController(gains);
  private double output;

  public CenterToTrap(Swerve swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    addRequirements(swerve);
  }
     
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("********inside CenterToTrap");
    pid.setMaxOutput(Constants.SwerveConstant.maxSpeed * 0.8);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setTargetPosition(0);
    output = pid.getOutput(limelight.getX()) *-1;
    output += 0.1 * Constants.SwerveConstant.maxAngularVelocity * Math.signum(output);
    swerve.drive(new Translation2d(output, 0), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("********exit CenterToTrap");
    swerve.drive(new Translation2d(0.0, 0.0), 0, true);
    swerve.lockWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(limelight.getX()) < 1; 
  }
}