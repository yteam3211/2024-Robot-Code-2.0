// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.shootingSubsystem;
import frc.robot.subsystems.armCollectSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class resetCommand extends InstantCommand {
  private final shootingSubsystem shootingSubsystem;
  private final CollectSubsystem collectSubsystem;
  private final armCollectSubsystem armCollectSubsystem;
  private final CartridgeSubsystem cartridgeSubsystem;


  public resetCommand(shootingSubsystem shootingSubsystem, CollectSubsystem collectSubsystem,  armCollectSubsystem armCollectSubsystem, CartridgeSubsystem cartridgeSubsystem) {
    this.shootingSubsystem = shootingSubsystem;
    this.collectSubsystem = collectSubsystem;
    this.armCollectSubsystem = armCollectSubsystem;
    this.cartridgeSubsystem = cartridgeSubsystem;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    shootingSubsystem.resetEncoder();
    collectSubsystem.resetEncoder();
    armCollectSubsystem.resetArmCollectEncoder();
    cartridgeSubsystem.resetEncoder();    
}
}
