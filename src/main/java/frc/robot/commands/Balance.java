// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;

public class Balance extends CommandBase {
  /** Creates a new balance. */
  protected Gains gainsTIP = new Gains("gains turn in place", 0.3, 0, 0);
  protected Gains gainsBAL = new Gains("gains balance", 0., 0, 0);
  protected PIDController pidTIP = new PIDController(gainsTIP);
  protected PIDController pidBAL = new PIDController(gainsBAL);
  protected Swerve swerve;
  public Balance(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidTIP.setTargetPosition(0);
    pidBAL.setTargetPosition(0);
    pidBAL.setMaxOutput(Constants.Swerve.maxSpeed * 0.1);
    pidTIP.setMaxOutput(Constants.Swerve.maxAngularVelocity * 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      swerve.drive(            
                  new Translation2d(0, 0.05 * Math.signum(Swerve.gyro.getPitch())).times(Constants.Swerve.maxSpeed), 
                  0,
                  false, //Field oriented by the controller switch
              true
              );
      // swerve.drive(            
      //             new Translation2d(0, pidBAL.getOutput(Swerve.gyro.getPitch())), 
      //             pidBAL.getOutput(Swerve.gyro.getYaw()), 
      //             false, //Field oriented by the controller switch
      //         true
      //         );
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


