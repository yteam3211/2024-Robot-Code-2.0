// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AllianceSpecs;
import frc.robot.Constants;
import frc.robot.ShootingMath;
import frc.robot.commands.Elevator.CloseElevatorCommandGroup;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.SwereCommands.DriveToTarget;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.util.vision.Limelight;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchAMP;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingOutput;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CompleteAMPShootingCommand extends ParallelCommandGroup {
  /** Creates a new CompleteAMPShootingCommand. */
  // public CompleteAMPShootingCommand(Swerve swerve, Limelight limelight, ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem,ElevatorSubsystem eleavatorSubsystem,KickerSubsystem kickerSubsystem, ShootingMath shootingMath, AllianceSpecs allianceSpecs) {
  public CompleteAMPShootingCommand(ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem,ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ShootingOutput(shootingSubsystem, 0.2),
      new PitchAMP(pitchingSubsystem, elevatorSubsystem, -40, 20),
       new ElevatorUpCommand(elevatorSubsystem, 500).onlyIf(() -> KickerSubsystem.isNoteIn())
      // new ParallelCommandGroup(
      //   // new PitchPos(pitchingSubsystem, Constants.AMP_PITCHING_ANGLE),
      //   // new ShootingOutput(shootingSubsystem, 0.2))


    );
  }
}
