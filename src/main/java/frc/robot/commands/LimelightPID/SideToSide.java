// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightPID;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;
import frc.util.vision.Limelight;

public class SideToSide extends CommandBase {
  /** Creates a new SideToSideApriltag. */
  protected Limelight limelight;
  protected Swerve swerve;
  protected Gains gainsX = new Gains("gains x", 0.05, 0, 0);
  protected Gains gainsY = new Gains("gains y", 0.25, 0, 0);
  protected Gains gainsR = new Gains("gains r", 0.05, 0, 0);
  protected PIDController pidX = new PIDController(gainsX);
  protected PIDController pidY = new PIDController(gainsY);
  protected PIDController pidR = new PIDController(gainsR);

  public SideToSide(Limelight limelight, Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX.setTargetPosition(0);
    pidY.setTargetPosition(0);
    pidR.setTargetPosition(0);
    pidX.setMaxOutput(Constants.Swerve.maxSpeed * 0.6);
    pidY.setMaxOutput(Constants.Swerve.maxSpeed * 0.6);
    pidR.setMaxOutput(Constants.Swerve.maxAngularVelocity * 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    swerve.drive(
    new Translation2d(pidY.getOutput(limelight.getY()), pidX.getOutput(limelight.getX())),
    pidR.getOutput(Swerve.gyro.getYaw()),
    false, // Field oriented by the controller switch
    true);
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
