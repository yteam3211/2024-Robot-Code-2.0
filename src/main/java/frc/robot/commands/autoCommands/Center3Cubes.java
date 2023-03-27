// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.autos.AutoCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;
import frc.robot.subsystems.shootingSubsystem;
import frc.robot.commands.ShootingCommnads.ShootingCommand;
import frc.robot.commands.ShootingCommnads.ShootingGroupCommand;
import frc.robot.commands.SwereCommands.BalanceCommand;
import frc.robot.commands.SwereCommands.TurnToZeroCommand;
import frc.robot.commands.resetCommand;
// import frc.robot.commands.ClosingCollectGroupCommand;
import frc.robot.commands.ShootingCommnads.CartridgeOutputCommand;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Center3Cubes extends SequentialCommandGroup {
  /** Creates a new Center3Cubes. */
  public Center3Cubes(Swerve swerve,
  CollectSubsystem collectSubsystem,
  CartridgeSubsystem cartridgeSubsystem,
  collectWheelsSubsystem collectWheels, shootingSubsystem shootingSubsystem, armCollectSubsystem armCollectSubsystem
) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand(() -> swerve.zeroGyro()), 
    new resetCommand(shootingSubsystem, collectSubsystem, armCollectSubsystem, cartridgeSubsystem),
    new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, 0.75, 0.3),
    new StartAuto(AutoCommand.getAutoCommand(swerve, "center - start Close To Human Cube", 3), armCollectSubsystem, swerve),
    new moveInParallel(swerve, collectSubsystem, collectWheels, armCollectSubsystem, cartridgeSubsystem, AutoCommand.getAutoCommand(swerve, "center - 3 cubes - collect second cube", 3), Constants.COLLECT_OPEN_POSITION, Constants.ARM_OPEN_POSITION, 2, 0.5, false),
    new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , Constants.SHOOTING_HIGH),
    new moveInParallel(swerve, collectSubsystem, collectWheels, armCollectSubsystem, cartridgeSubsystem, AutoCommand.getAutoCommand(swerve, "center - 3 cubes - collect third cube", 3), Constants.COLLECT_OPEN_POSITION, Constants.ARM_OPEN_POSITION, 2, 0.5 , false),
    new BalanceCommand(swerve, true),
    new TurnToZeroCommand(swerve),
    new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , Constants.SHOOTING_HIGH)
    );
  }
}
