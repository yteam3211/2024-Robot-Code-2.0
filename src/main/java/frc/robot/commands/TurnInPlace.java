// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;

public class TurnInPlace extends CommandBase {
  /** Creates a new TurnZero. */
  private Swerve swerve;
  public TurnInPlace(Swerve swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(swerve.getYaw().getDegrees())> 2){
      swerve.drive(            
          new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
          swerve.getYaw().getDegrees() > 0 ? swerve.getYaw().getDegrees() : -swerve.getYaw().getDegrees(), 
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
    if (RobotButtons.driver.getPOV() == 180 && Math.abs(swerve.getYaw().getDegrees())> 2)
      return true;
    else
      return false;
  }
}
