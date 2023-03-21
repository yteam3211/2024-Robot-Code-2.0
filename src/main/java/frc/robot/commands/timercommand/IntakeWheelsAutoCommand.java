// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommands.collectWheelsCommand;
import frc.robot.subsystems.collectWheelsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeWheelsAutoCommand extends SequentialCommandGroup {
  /** Creates a new IntakeWheelsAutoCommand. */
  public IntakeWheelsAutoCommand(collectWheelsSubsystem collectWheels, double WheelsOutput, double centeringOutput, double delayStartWheels , double deadLineWheels) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(delayStartWheels),
      new ParallelDeadlineGroup(
        new WaitCommand(deadLineWheels),
        new collectWheelsCommand(collectWheels, WheelsOutput, centeringOutput)
      )
    );
  }
}
