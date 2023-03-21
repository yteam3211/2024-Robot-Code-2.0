// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.autos.AutoCommand;
import frc.robot.commands.resetCommand;
import frc.robot.commands.IntakeCommands.setPointCollectCommand;
import frc.robot.commands.ShootingCommnads.ShootingCommand;
import frc.robot.commands.ShootingCommnads.ShootingGroupCommand;
import frc.robot.commands.SwereCommands.TurnToZeroCommand;
import frc.robot.commands.timercommand.TimerCollectWheels;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;
import frc.robot.subsystems.shootingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Next2Human3Cubes extends SequentialCommandGroup {
  /** Creates a new Next2Human3Cubes. */
  public Next2Human3Cubes(Swerve swerve,
  CollectSubsystem collectSubsystem,
  CartridgeSubsystem cartridgeSubsystem,
  collectWheelsSubsystem collectWheels, armCollectSubsystem armCollectSubsystem, shootingSubsystem shootingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> swerve.zeroGyro()), new resetCommand(shootingSubsystem, collectSubsystem, armCollectSubsystem, cartridgeSubsystem),
    new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, 0.75, 0.3),
    new InstantCommand(() -> collectSubsystem.setPosition(290)),
    new StartAuto(AutoCommand.getAutoCommand(swerve, "Next 2 Human - start", 5), armCollectSubsystem, swerve),
    new InstantCommand(() -> armCollectSubsystem.setArmCollectPosition(Constants.ARM_OPEN_POSITION)),
    new ParallelDeadlineGroup(
      AutoCommand.getAutoCommand(swerve, "next 2 human & 1 cube", 2),
      new TimerCollectWheels(collectWheels, Constants.COLLECT_WHEELS_OUTPUT, Constants.CENTERING_WHEELS_OUTPUT, 3, 0.3),
      new SequentialCommandGroup(
        new WaitCommand(1.1),
        new InstantCommand(() -> collectSubsystem.setPosition(0))
      ),
      new AutoCubeFixture(cartridgeSubsystem, 2)
    ),
    new TurnToZeroCommand(swerve),
    new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , Constants.SHOOTING_LOW),
    new moveInParallel(swerve, collectSubsystem, collectWheels, armCollectSubsystem, cartridgeSubsystem, AutoCommand.getAutoCommand(swerve, "Next To Human - 3 cubes Additon", 5), Constants.COLLECT_OPEN_POSITION, Constants.ARM_OPEN_POSITION, 5, 0.7,false)
    );
  }
}
