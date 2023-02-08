// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConeLimelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.util.vision.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class SideToSideRetroReflective extends CommandBase {
  /** Creates a new SideToSideLimelight. */
  private Limelight limelight = null;
  private Swerve swerve;
  public SideToSideRetroReflective(Limelight limelight, Swerve swerve) {
    this.limelight = limelight;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(limelight.getX()) > 0.5){
      swerve.drive(            
          new Translation2d(limelight.getX() > 0 ? -0.1 : 0.1, 0.0).times(Constants.Swerve.maxSpeed), 
          0.0, 
       false, //Field oriented by the controller switch
       true
      );
  }
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
