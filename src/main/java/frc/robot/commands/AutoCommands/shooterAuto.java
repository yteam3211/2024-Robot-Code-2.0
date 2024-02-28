// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ShootingCommands.KickerCommands.DefaultKicker;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerIntakeCommand;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.util.commands.TimeCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shooterAuto extends SequentialCommandGroup {
  /** Creates a new shooterAuto. */
  public shooterAuto(ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem, KickerSubsystem KickerSubsystem) {
   
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    addCommands(new  AutoShooingWheels(shootingSubsystem, Constants.SHOOTING_SPEAKER_VELCITY),
    new PitchPos(pitchingSubsystem, 48),
    new ParallelDeadlineGroup(
      // new TimeCommand(2000)
     new WaitCommand(2)),
    new KickerIntakeCommand(KickerSubsystem, shootingSubsystem, 0.4)
    );
    // addCommands(new FooCommand(), new BarCommand());
  }
}
