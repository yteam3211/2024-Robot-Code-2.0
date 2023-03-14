// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class armCollectOutput extends CommandBase {
  private final armCollectSubsystem armCollect;
  private double output;
  private double timerSeconds = 0;
  private Timer timer = new Timer();

  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public armCollectOutput(armCollectSubsystem armCollect, double output, double timerSeconds) {
    this.armCollect = armCollect;
    this.output = output;
    this.timerSeconds = timerSeconds;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armCollect);
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
    // if (armCollect.isShootingDown()){ 
    //   armCollect.resetArmCollectEncoder();
    // }
    if(timer.hasElapsed(timerSeconds)){
      armCollect.setArmCollectOutput(output);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armCollect.resetArmCollectEncoder();
    armCollect.setArmCollectOutput(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // return false;
    // if(output>0){
    return armCollect.isShootingDown();
    //  } 
    // else{
    //   return false;
    // }
}
}


