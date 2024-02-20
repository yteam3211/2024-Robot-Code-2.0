// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Eleavator.EleavatorCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.ShootingCommands.KickerCommand;
import frc.robot.commands.ShootingCommands.PitchPos;
import frc.robot.commands.TransferCommands.TransferCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.TransferSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeGroupCommand extends SequentialCommandGroup {
  /** Creates a new IntakeAndTransferCommand. */
  public AutoIntakeGroupCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,ShootingSubsystem shootingSubsystem,KickerSubsystem kickerSubsystem,PitchingSubsystem pitchingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        new ParallelCommandGroup(
          new IntakeCommand(intakeSubsystem, Constants.INTAKE_OPEN_POSITION, -1000).onlyWhile(() -> !kickerSubsystem.isNoteIn()),
          new TransferCommand(transferSubsystem, 0.8),//.onlyWhile(() -> kickerSubsystem.isNoteIn()),
          new KickerCommand(kickerSubsystem, 0.4))
        );
    }
}
