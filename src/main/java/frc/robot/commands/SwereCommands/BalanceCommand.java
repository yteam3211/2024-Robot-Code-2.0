// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwereCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;

public class BalanceCommand extends CommandBase {
  /** Creates a new BalanceCommand. */
  protected Swerve swerve;
  protected PIDController balancePID = new PIDController(); // create PID controller for the balance PID 
  protected Gains balanceGains = new Gains("balance gains", 0.02, 0.0001, 0.32); // create gains for the PID controller 
  protected boolean isAutonomus; // check if the command is called during autonomus
  protected Timer timer = new Timer(); // create timer, use to check how much time has passed since the robot has balanced

  public BalanceCommand(Swerve swerve, boolean isAutonmous){
    this.swerve = swerve;
    this.isAutonomus = isAutonmous;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePID.setGains(balanceGains); // set the gains for the PID controller 
    balancePID.setTargetPosition(0); // set the the target position for the PID controller (in this case, the desired angle)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(swerve.gyro.getRoll()) > 2.5){ // check if the angle of the robot is bigger than 2.5 degrees
      swerve.drive(                             // start drive command for the swerve using PID controller
      new Translation2d(balancePID.getOutput(swerve.gyro.getRoll()), 0).times(Constants.SwerveConstant.maxSpeed * 0.27), 
      0, 
      true
      );
    }
    else{ // if the robot angle is lower then 2.5 degrees (AKA balanced)
      swerve.setStop(); // set the swerve wheels to an X form (lock the wheels) so the robot wont slide from the ramp 
      timer.start(); // start a timer when the rbot is balanced
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { // stop the command if any of the joysticks are moved, or if the time has passed
    return RobotButtons.forwardJoystick.getAsBoolean() || RobotButtons.sidesJoystick.getAsBoolean() || RobotButtons.rotationJoystick.getAsBoolean() || (timer.hasElapsed(0.3) && this.isAutonomus);
  }
}