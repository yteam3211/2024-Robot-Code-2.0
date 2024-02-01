// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShootingCommands.KickerCommand;
import frc.robot.commands.ShootingCommands.ShootingSpeedCommand;
import frc.robot.commands.TransferCommands.TransferCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.TransferSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndTransferCommand extends ParallelCommandGroup {
  /** Creates a new IntakeAndTransferCommand. */
  public IntakeAndTransferCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,ShootingSubsystem shootingSubsystem,KickerSubsystem kickerSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeBackwordsCommand(intakeSubsystem, -0.8), // TODO: change to IntakeCommand
      new TransferCommand(transferSubsystem, 0.8),
      new  ShootingSpeedCommand(shootingSubsystem, 20000),
      new KickerCommand(kickerSubsystem,shootingSubsystem,0.8)
    );
  }
}
