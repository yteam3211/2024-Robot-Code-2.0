// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Eleavator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShootingCommands.PitchPos;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenElevatorCommanGroup extends ParallelCommandGroup {
  /** Creates a new OpenElevatorCommanGroup. */
  public OpenElevatorCommanGroup(ElevatorSubsystem elevatorSubsystem, PitchingSubsystem pitchingSubsystem, Double ElevatorPos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new EleavatorUpCommand(elevatorSubsystem, 0),
      new PitchPos(pitchingSubsystem, 15).onlyWhile(() -> pitchingSubsystem.getAbsolutePosition() > 17)
      
    );
  }
}
