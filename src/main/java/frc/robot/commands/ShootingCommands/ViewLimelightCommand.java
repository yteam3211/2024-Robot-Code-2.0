// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.ShootingMath;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.SwereCommands.TurnSwerveCommand;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ViewLimelightCommand extends ParallelCommandGroup {
  /** Creates a new ViewLimelightCommand. */
  public ViewLimelightCommand(Swerve swerve, PitchingSubsystem pitchingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnSwerveCommand(swerve, 0).onlyWhile(
          () -> (Math.abs(swerve.gyro.getYaw() - ShootingMath.getEstematedSpeakerShootingAngle(swerve)) > Constants.TURN_SWERVE_TRESHOLD)),
      new PitchPos(pitchingSubsystem, Constants.LIMELIGHT_lOOKING_ANGLE).onlyWhile(
          () -> (Math.abs(Constants.LIMELIGHT_lOOKING_ANGLE - pitchingSubsystem.getAbsolutePosition()) > Constants.LIMELIGHT_lOOKING_ANGLE_TRESHOLD))
    );
  }
}
