// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommnads;

import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.shootingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class CartridgeOutputCommand extends CommandBase {
  private final CartridgeSubsystem cartridgeSubsystem;
  private double output;
  private double velocity;
  private double max;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CartridgeOutputCommand(CartridgeSubsystem cartridgeSubsystem, double output, double max) {
    this.cartridgeSubsystem = cartridgeSubsystem;
    this.output = output;
    this.max = max;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(cartridgeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("position: " + cartridgeSubsystem.GetPosition());
    
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
    // System.out.println("signum: " + (Math.signum(output) == Math.signum(1)));
    // System.out.println("over Max: " + (cartridgeSubsystem.GetPosition() >= max));
    if(output < 0){
      System.out.println("inside signum 0");
      if(cartridgeSubsystem.GetPosition() <= max){
        System.out.println("inside overMax 0");
        return true;
      }
      else{
        System.out.println("inside overMax 888");
        return false;
    }
  }
    if(output > 0){
      System.out.println("inside signum");
      if(cartridgeSubsystem.GetPosition() >= max){
        System.out.println("inside overMax");
        return true;
      }else{
        System.out.println("inside overMax 888");
        return false;
      }
    }else{
      System.out.println("over");
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



