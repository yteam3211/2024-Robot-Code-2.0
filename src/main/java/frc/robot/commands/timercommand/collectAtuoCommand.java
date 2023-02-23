// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheels;

public class collectAtuoCommand extends CommandBase {
    private final collectWheels collectWheels;
    private double WheelsOutput;
    private double centeringOutput;
    private Timer timer = new Timer();
    private int timeOfFunctioning;
    
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public collectAtuoCommand(collectWheels collectWheels, double Output, double output, int timeOfFunctioning) {
      this.collectWheels = collectWheels;
      this.WheelsOutput = output;
      this.centeringOutput = Output;
      this.timeOfFunctioning = timeOfFunctioning;


  
      // Use addRequirements() here to declare subsystem dependencies.
      // addRequirements(collectSubsystem);
    } 
  
    // Called when the command is initially scheduled.
    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj2.command.Command#initialize()
     */
    @Override
    public void initialize() {
      timer.reset();
      timer.start();
    }
  


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
      collectWheels.WheelsSetOutput(WheelsOutput);
      collectWheels.centeringSetOutput(centeringOutput);
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
      if (timer.hasElapsed(timeOfFunctioning)){
        timer.reset();
        return true;
      }
      else
        return false;
      }
  }
