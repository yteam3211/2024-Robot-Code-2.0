// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import frc.robot.subsystems.armSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class TimerArmPosition extends CommandBase {
  private final armSubsystem armSubsystem;
  private double position;
  private boolean deadBand;
  private double delay;
  private double stop;
  private Timer timer = new Timer();
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TimerArmPosition(armSubsystem armSubsystem, double position, double delay, double stop) {
    this.armSubsystem = armSubsystem;
    this.position = position;
    this.delay = delay;
    this.stop = stop;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    timer.delay(delay);
    armSubsystem.setPosition(position);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // armSubsystem.setPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(stop)){
      timer.reset();
      return true;
    }
    else
      return false;
      
    }
}

