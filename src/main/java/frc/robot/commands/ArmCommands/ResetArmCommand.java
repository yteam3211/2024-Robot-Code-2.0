// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armCollectSubsystem;

public class ResetArmCommand extends CommandBase {
  private final armCollectSubsystem armCollect;
  // private double armOutput;
  /** Creates a new ResetArmCommand. */
  public ResetArmCommand(armCollectSubsystem armCollect) {
    // this.armOutput = armOutput;
    this.armCollect = armCollect;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armCollect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if(armCollect.isShootingDown()) {
      armCollect.resetArmCollectEncoder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
