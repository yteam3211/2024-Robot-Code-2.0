// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Eleavator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerOutput;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingOutput;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapOpenElevator extends SequentialCommandGroup {
  /** Creates a new TrapOpenElevator. */
  public TrapOpenElevator(ElevatorSubsystem elevatorSubsystem, PitchingSubsystem pitchingSubsystem, KickerSubsystem kickerSubsystem, ShootingSubsystem shootingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new EleavatorUpCommand(elevatorSubsystem, 510),
        new PitchPos(pitchingSubsystem, 24)),
      new WaitCommand(4),
      // new ParallelDeadlineGroup(
      //   new WaitCommand(0.3), 
      //   new ParallelCommandGroup(
      //     new ShootingOutput(shootingSubsystem, 0.1), 
      //     new KickerOutput(kickerSubsystem, shootingSubsystem, 0.4))),
      // new EleavatorUpCommand(elevatorSubsystem, 420),
      // new ParallelCommandGroup(
      //   new SequentialCommandGroup(
      //     new WaitCommand(0.4),
      //     new ParallelDeadlineGroup(
      //       new WaitCommand(0.5), 
      //       new ParallelCommandGroup(
      //         new ShootingOutput(shootingSubsystem, 0.1), 
      //         new KickerOutput(kickerSubsystem, shootingSubsystem, 0.4)))),
      //     new PitchPos(pitchingSubsystem, 10)),
      // new EleavatorUpCommand(elevatorSubsystem, 560),
        new ParallelCommandGroup(
          new ShootingOutput(shootingSubsystem, 0.07), 
          new KickerOutput(kickerSubsystem, shootingSubsystem, 0.12)).onlyWhile(() -> KickerSubsystem.isNoteIn()),
        new ShootingOutput(shootingSubsystem, -0.1).onlyWhile(() -> !KickerSubsystem.isNoteIn()),
      new PitchPos(pitchingSubsystem, 5),
      new EleavatorUpCommand(elevatorSubsystem, 450),
      new WaitCommand(1),
      new EleavatorUpCommand(elevatorSubsystem, 635),
      new ParallelDeadlineGroup(
        new WaitCommand(0.3),
        new ParallelCommandGroup(
          new ShootingOutput(shootingSubsystem, 0.08), 
          new KickerOutput(kickerSubsystem, shootingSubsystem, 0.2)))
      //   new ParallelDeadlineGroup(
      //     new WaitCommand(0.5), 
      //     new ParallelCommandGroup(
      //       new ShootingOutput(shootingSubsystem, 0.1), 
      //       new KickerOutput(kickerSubsystem, shootingSubsystem, 0.4))),
      // new PitchPos(pitchingSubsystem, -10)
      
          
    );
  }
}
