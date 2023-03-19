// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootingCommnads.CubeFixtureGroupCommand;
import frc.robot.subsystems.CartridgeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCubeFixture extends SequentialCommandGroup {
  /** Creates a new AutoCubeFixture. */
  public AutoCubeFixture(CartridgeSubsystem cartridgeSubsystem, double delaySeconds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(delaySeconds),
      new CubeFixtureGroupCommand(cartridgeSubsystem, 0, 0.15, 1400, -0.2, 20),
      new CubeFixtureGroupCommand(cartridgeSubsystem, 0, 0.15, 1400, -0.2, 20)
    );
  }
}
