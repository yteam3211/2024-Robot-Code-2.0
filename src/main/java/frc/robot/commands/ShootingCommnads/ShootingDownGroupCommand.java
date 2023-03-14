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
public class ShootingDownGroupCommand extends SequentialCommandGroup {
  /** Creates a new ShootingGroupCommand. */
  public ShootingDownGroupCommand(CartridgeSubsystem cartridgeSubsystem, shootingSubsystem shootingSubsystem, armCollectSubsystem armCollectSubsystem,
    double ShootingVelocity, double CartridgeOutput, double ShootingBackOutput, double max) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, ShootingVelocity, CartridgeOutput),
      new CartridgeOutputCommand(cartridgeSubsystem, ShootingBackOutput, max)
    );
  }
}
