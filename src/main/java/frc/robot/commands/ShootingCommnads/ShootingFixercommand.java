// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommnads;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.shootingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingFixercommand extends SequentialCommandGroup {
  /** Creates a new ShootingFixercommand. */
  public ShootingFixercommand(CartridgeSubsystem cartridgeSubsystem, double upOutput, int upMax, double downOutput, double downMin) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    for (int i =0; i < 1000; i++) {
      addCommands(
        new CartridgeOutputCommand(cartridgeSubsystem, upOutput, upMax),
        new CartridgeOutputCommand(cartridgeSubsystem, downOutput, downMin)
      );
    }
  }
}