// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingOutput;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingVelocity;
import com.fasterxml.jackson.databind.jsontype.DefaultBaseTypeLimitingValidator;

import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.PitchCommands.SpeakerPitchCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.ShootingMath;
import frc.robot.commands.SwereCommands.LockWheelsCommand;
import frc.robot.commands.SwereCommands.TurnToShootingCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.util.vision.Limelight;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerShootingCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CompleteSpeakerShootingCommand extends ParallelRaceGroup {
  /** Creates a new CompleteSootingCommand. */
  public CompleteSpeakerShootingCommand(Swerve swerve, Limelight limelight, ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem,ElevatorSubsystem eleavatorSubsystem,KickerSubsystem kickerSubsystem, ShootingMath shootingMath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(         //TODO: change the eleavator position and the kickers output
      // !limelight.isValid() ? Math.ab s(Constants.LIMELIGHT_lOOKING_ANGLE - pitchingSubsystem.getAbsolutePosition()) > Constants.LIMELIGHT_lOOKING_ANGLE_TRESHOLD ? new PitchPos(pitchingSubsystem, Constants.LIMELIGHT_lOOKING_ANGLE) : new WaitCommand(0) : new EleavatorCommand(eleavatorSubsystem, 0) ,
      // new ViewLimelightCommand(swerve, pitchingSubsystem).onlyWhile(() -> !limelight.isValid()),
        new ShootingVelocity(shootingSubsystem, Constants.SHOOTING_SPEAKER_VELCITY),
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new WaitCommand(0.1),
            new SpeakerPitchCommand(limelight, pitchingSubsystem, eleavatorSubsystem, shootingMath, shootingSubsystem)),
          new TurnToShootingCommand(swerve, limelight, shootingMath),
          new ParallelRaceGroup(
            new LockWheelsCommand(swerve),
            new SequentialCommandGroup(
              new ParallelRaceGroup(
                // new WaitCommand(0.5),
                new SpeakerPitchCommand(limelight, pitchingSubsystem, eleavatorSubsystem, shootingMath, shootingSubsystem)).andThen(() -> shootingMath.setShootingCondition(true)),
              new KickerShootingCommand(kickerSubsystem, shootingSubsystem, Constants.KICKER_OUTPUT).onlyWhile(() -> !RobotButtons.kicker.getAsBoolean()))))
    );
        
  }
}
