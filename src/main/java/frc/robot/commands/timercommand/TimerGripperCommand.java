// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import frc.robot.commands.gripperCommand;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.armSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class TimerGripperCommand extends CommandBase {
  private final GripperSubsystem gripperSubsystem;
  private double position;
  private double seconds;
  private Timer timer = new Timer();
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TimerGripperCommand(double position, double seconds, GripperSubsystem gripperSubsystem) {
    this.gripperSubsystem = gripperSubsystem;
    this.position = position;
    this.seconds = seconds;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripperSubsystem.setGripperPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return timer.hasElapsed(seconds);  
    }
}

