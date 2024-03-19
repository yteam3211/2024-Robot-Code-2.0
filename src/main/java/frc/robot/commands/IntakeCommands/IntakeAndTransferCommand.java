// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerIntakeCommand;
import frc.robot.commands.IntakeCommands.TransferCommands.TransferCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndTransferCommand extends SequentialCommandGroup {
  /** Creates a new IntakeAndTransferCommand. */
  public IntakeAndTransferCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,ShootingSubsystem shootingSubsystem,KickerSubsystem kickerSubsystem,PitchingSubsystem pitchingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand(\][]));
      addCommands(
        new ParallelCommandGroup(
         new IntakeCommand(intakeSubsystem, Constants.INTAKE_OPEN_POSITION, Constants.INTAKE_WHEELS_VELOCITY),
          new TransferCommand(transferSubsystem, 0.93),
          new KickerIntakeCommand(kickerSubsystem, shootingSubsystem, 0.14),
          new PitchPos(pitchingSubsystem, 20).onlyIf(() -> (pitchingSubsystem.getAbsolutePosition() < 10))
        ),
        new PitchPos(pitchingSubsystem, 0)



    );
    }
}
