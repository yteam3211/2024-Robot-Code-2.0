// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwereCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;

public class LockWheelsCommand extends Command {
  private Swerve swerve;
  private boolean isAuto = false;


  /** Creates a new lockWheelsCommnad. */
  public LockWheelsCommand(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******** inside LockWheelsCommnad");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.lockWheels();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("********exit LockWheelsCommnad");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;// RobotButtons.forwardJoystick.getAsBoolean() || RobotButtons.sidesJoystick.getAsBoolean() || RobotButtons.rotationJoystick.getAsBoolean();
  }
}
