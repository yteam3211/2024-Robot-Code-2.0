// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.ShootingMath;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.PitchCommands.SpeakerPitchCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.util.vision.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DistanceAutoShootingGroup extends SequentialCommandGroup {
  /** Creates a new DistanceAutoShootingGroup. */
  public DistanceAutoShootingGroup(Limelight limelight, PitchingSubsystem pitchingSubsystem, ElevatorSubsystem elevatorSubsystem, ShootingMath shootingMath, ShootingSubsystem shootingSubsystem, KickerSubsystem kickerSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PitchPos(pitchingSubsystem, 40),
      new AutoKickerCommand(kickerSubsystem, shootingSubsystem, Constants.KICKER_OUTPUT)
    );
  }
}
