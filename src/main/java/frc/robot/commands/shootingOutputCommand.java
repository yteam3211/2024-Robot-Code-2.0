// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.shootingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class shootingOutputCommand extends CommandBase {
  private final CartridgeSubsystem cartridgeSubsystem;
  private double output;
  private double velocity;
  private double max;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public shootingOutputCommand(CartridgeSubsystem cartridgeSubsystem, double output, double max) {
    this.cartridgeSubsystem = cartridgeSubsystem;
    this.output = output;
    this.max = max;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cartridgeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(cartridgeSubsystem.GetOutput());
    cartridgeSubsystem.setOutput(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridgeSubsystem.setOutput(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // output += 0.001;
    // if (output == 0.95)
    //   return true; 
    if(cartridgeSubsystem.GetPosition() >= max){
      return true;
    }else{
      return false;
    }


    // return false;
    // if(output>0){
    //   return shootingSubsystem.isShootingUp(); 
    // }
    //  else{
    //   return shootingSubsystem.isShootingDown();
    //  } 
}
}


