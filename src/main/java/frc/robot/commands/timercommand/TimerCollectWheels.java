// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TimerCollectWheels extends CommandBase {
  private final collectWheelsSubsystem collectWheels;
  private double WheelsOutput;
  private double centeringOutput;
  private double seconds;
  private double delay;
  private Timer timer = new Timer();


  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public 
  TimerCollectWheels(collectWheelsSubsystem collectWheels, double WheelsOutput, double centeringOutput,double seconds, double delay) {
    this.collectWheels = collectWheels;
    this.WheelsOutput = WheelsOutput;
    this.centeringOutput = centeringOutput;
    this.seconds = seconds;
    this.delay = delay;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collectWheels);
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
    if(timer.hasElapsed(delay)){
      collectWheels.WheelsSetOutput(WheelsOutput);
      collectWheels.centeringSetOutput(centeringOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectWheels.WheelsSetOutput(0);
    collectWheels.centeringSetOutput(0);
  }  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer.hasElapsed(seconds);  
    }
}

