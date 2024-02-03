// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PitchingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class restShooting extends InstantCommand {
  private PitchingSubsystem pitchingSubsystem;
  public restShooting(PitchingSubsystem pitchingSubsystem) {
    this.pitchingSubsystem = pitchingSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pitchingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchingSubsystem.SetOutput(0);
  }
}
