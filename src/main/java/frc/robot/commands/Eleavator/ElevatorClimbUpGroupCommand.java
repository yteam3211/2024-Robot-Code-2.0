// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Eleavator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingOutput;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.shootingHook;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorClimbUpGroupCommand extends ParallelCommandGroup {
  /** Creates a new ElevatorClimbUpGroupCommand. */
  public ElevatorClimbUpGroupCommand(ElevatorSubsystem elevatorSubsystem, ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new EleavatorUpCommand(elevatorSubsystem, Constants.CLIMB_ELEVATOR_HIGHT),
    new PitchPos(pitchingSubsystem, 0),
    new shootingHook(shootingSubsystem, -1800)
    );
  }
}
