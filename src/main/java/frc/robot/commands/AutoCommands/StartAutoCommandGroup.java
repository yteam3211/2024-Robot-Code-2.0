// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ShootingCommands.KickerCommands.DefaultKicker;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartAutoCommandGroup extends SequentialCommandGroup {
  /** Creates a new StartAutoCommandGroup. */
  public StartAutoCommandGroup(ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem, KickerSubsystem kickerSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShooingWheels(shootingSubsystem, Constants.SHOOTING_SPEAKER_VELCITY),
      new ParallelDeadlineGroup(
        new PitchPos(pitchingSubsystem, 52),
        new DefaultKicker(kickerSubsystem, 0.1)),
      new AutoKickerCommand(kickerSubsystem, shootingSubsystem, Constants.KICKER_OUTPUT)
    );
  }
}
