// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CollectSubsyste;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.armSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetCommand extends InstantCommand {
  private final ShootingSubsystem shootingSubsystem;
  private final CollectSubsyste collectSubsyste;
  private final armSubsystem armSubsystem;

  public ResetCommand(ShootingSubsystem shootingSubsystem, CollectSubsyste collectSubsyste,  armSubsystem armSubsystem) {
    this.shootingSubsystem = shootingSubsystem;
    this.collectSubsyste = collectSubsyste;
    this.armSubsystem = armSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootingSubsystem.resetEncoder();
    collectSubsyste.reSetEncoder();
    armSubsystem.resetEncoder();
  }
}
