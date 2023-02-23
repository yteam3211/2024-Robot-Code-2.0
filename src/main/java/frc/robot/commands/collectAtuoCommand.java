// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectOutoSubsystem;
import frc.robot.subsystems.CollectSubsystem;

public class collectAtuoCommand extends CommandBase {
    private final CollectSubsystem collectSubsystem;
    private double output;
    private double Output;
    private Timer timer = new Timer();
    private int timeOfFunctioning;
    
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public collectAtuoCommand(double Output, double output, CollectSubsystem collectSubsystem, int timeOfFunctioning) {
      this.collectSubsystem = collectSubsystem; 
      this.output = output;
      this.Output = Output;
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
        collectSubsystem.setOutput(output);
        collectSubsystem.output(Output);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      collectSubsystem.setOutput(0);
      collectSubsystem.output(0);
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
