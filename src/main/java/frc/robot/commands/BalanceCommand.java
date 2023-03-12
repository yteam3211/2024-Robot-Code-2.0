// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.util.PID.PIDController;

public class BalanceCommand extends CommandBase {
  protected Swerve swerve;
  protected PIDController balancePID = new PIDController();
  /** Creates a new BalanceCommand. */
  public BalanceCommand(Swerve swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePID.setGains(swerve.balanceGains);
    balancePID.setTargetPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(swerve.gyro.getRoll()) > 2.5){
      swerve.drive(            
      new Translation2d(balancePID.getOutput(swerve.gyro.getRoll()), 0).times(Constants.SwerveConst.maxSpeed * 0.3), 
        0, 
        false, //Field oriented by the controller switch
      true
      );
    }
    else{
      swerve.setStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotButtons.forwardJoystick.getAsBoolean() || RobotButtons.sidesJoystick.getAsBoolean() || RobotButtons.rotationJoystick.getAsBoolean();
  }
}
