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
public class CloseElevatorCommandGroup extends ParallelCommandGroup {
  /** Creates a new CloseElevatorCommandGroup. */
  public CloseElevatorCommandGroup(ElevatorSubsystem elevatorSubsystem, PitchingSubsystem pitchingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new EleavatorDown(elevatorSubsystem, 0),
      new PitchPos(pitchingSubsystem, 0).onlyWhile(() -> pitchingSubsystem.getAbsolutePosition() < -5)
    );
  }
}
