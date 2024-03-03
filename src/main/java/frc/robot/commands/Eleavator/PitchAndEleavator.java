// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Eleavator;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PitchAndEleavator extends ParallelCommandGroup{
  /** Creates a new PitchindAnDEleavator. */
  public PitchAndEleavator( PitchingSubsystem pitchingSubsystem,ElevatorSubsystem eleavatorSubsystem, double angle,double eleavatorPos) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(  
      new PitchPos(pitchingSubsystem, angle),
      new EleavatorUpCommand(eleavatorSubsystem, eleavatorPos)
    );
  }
}
