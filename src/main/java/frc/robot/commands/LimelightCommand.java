// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;
import frc.util.vision.Limelight;

public class LimelightCommand extends CommandBase {
  /** Creates a new SideToSideApriltag. */
  protected Limelight limelight;
  protected Swerve swerve;
  protected boolean AprilTag;
  protected boolean count = false;
  protected double yPos;
  protected double xPos;
  protected Timer timer = new Timer();
  protected Gains gainsX = new Gains("gains x", 0.04, 0, 0);
  protected Gains RRgainsY = new Gains("gains y", 0.17, 0, 0.045);
  protected Gains gainsR = new Gains("gains r", 0.05, 0, 0);
  // protected Gains gainsX = new Gains("gains x", 0.03, 0, 0);
  protected Gains gainsY = new Gains("gains y", 0.45, 0, 0.045);
  // protected Gains gainsR = new Gains("gains r", 0.05, 0, 0);
  protected PIDController pidX = new PIDController(gainsX);
  protected PIDController pidY = new PIDController(gainsY);
  protected PIDController pidR = new PIDController(gainsR);

  public LimelightCommand(Limelight limelight, Swerve swerve, boolean AprilTag, double yPos, double xPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    this.AprilTag = AprilTag;
    this.yPos = yPos;
    this.xPos = xPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    if (AprilTag){                            
      limelight.setPipeline(0);
    }
    else{
      limelight.setPipeline(1);
      pidY.setGains(RRgainsY);
    }
    pidX.setTargetPosition(xPos);
    pidY.setTargetPosition(yPos);
    pidR.setTargetPosition(0);
    pidX.setMaxOutput(Constants.Swerve.maxSpeed * 0.6);
    pidY.setMaxOutput(Constants.Swerve.maxSpeed * 0.6);
    pidR.setMaxOutput(Constants.Swerve.maxAngularVelocity * 0.6);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOutput = pidX.getOutput(limelight.getX());
    double yOutput = pidY.getOutput(limelight.getY());
    double rOutput = pidR.getOutput(Swerve.gyro.getYaw());

    xOutput += 0.02 * Constants.Swerve.maxSpeed * Math.signum(xOutput);
    yOutput += 0.02 * Constants.Swerve.maxSpeed * Math.signum(yOutput);
    rOutput += 0.02 * Constants.Swerve.maxAngularVelocity * Math.signum(rOutput);

    System.out.println("x output: " + xOutput);
    // System.out.println("y output: " + yOutput);
    // if(Math.abs(limelight.getY()) > Math.abs(yPos) + 0.3 || Math.abs(limelight.getX()) > 0.3 || Math.abs(Swerve.gyro.getYaw()) > 1.5){
    if(Math.abs(limelight.getY()) < Math.abs(yPos) + 0.4 && Math.abs(limelight.getX()) > 1.3 && Math.abs(Swerve.gyro.getYaw()) > 0.5 && count){
      count = false;
      timer.reset();
      timer.start();
    }
    else{
      timer.reset();
      count = true;
      swerve.drive(new Translation2d(yOutput, xOutput), rOutput, false, true);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(9);
  }
}
