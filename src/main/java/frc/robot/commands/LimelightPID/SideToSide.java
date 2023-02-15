// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightPID;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;
import frc.util.vision.Limelight;

public class SideToSide extends CommandBase {
  /** Creates a new SideToSideApriltag. */
  protected Limelight limelight;
  protected Swerve swerve;
  protected PIDController pidX = new PIDController();
  protected Gains gainsX = new Gains("gains x", 0, 0, 0);

  public SideToSide(Limelight limelight, Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX.setTargetPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    swerve.drive(
    new Translation2d(pidX.getOutput(limelight.getX()), 0),
    0,
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
