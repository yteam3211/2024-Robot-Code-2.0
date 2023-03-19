// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import frc.robot.autos.AutoCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.robot.subsystems.shootingSubsystem;
import frc.util.vision.Limelight;
import frc.robot.commands.ShootingCommnads.ShootingCommand;
import frc.robot.commands.ShootingCommnads.ShootingGroupCommand;
import frc.robot.commands.SwereCommands.BalanceCommand;
import frc.robot.commands.SwereCommands.LimelightCommand;
// import frc.robot.commands.ClosingCollectGroupCommand;
import frc.robot.commands.resetCommand;
import frc.robot.commands.ArmCommands.ArmCollectCommand;
import frc.robot.commands.ShootingCommnads.CartridgeOutputCommand;
import frc.robot.commands.timercommand.TimerArmPosition;

    // addCommands(new FooCommand(), new BarCommand());
public class FarFromHumanCube extends SequentialCommandGroup {

  /** Creates a new FarFromHuman. */
  public FarFromHumanCube(Swerve swerve,
  CollectSubsystem collectSubsystem,
  CartridgeSubsystem cartridgeSubsystem,
  collectWheels collectWheels, armCollectSubsystem armCollectSubsystem, Limelight limelight, shootingSubsystem shootingSubsystem) {

    addCommands(new InstantCommand(() -> swerve.zeroGyro()), new resetCommand(shootingSubsystem, collectSubsystem, armCollectSubsystem, cartridgeSubsystem),
    new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, 0.75, 0.3),
    new StartAuto(AutoCommand.getAutoCommand(swerve, "far from human + cube - start"), armCollectSubsystem, swerve),
    new moveInParallel(swerve, collectSubsystem, collectWheels, armCollectSubsystem, cartridgeSubsystem, AutoCommand.getAutoCommand(swerve, "far from human + cube"), 250, 5.2, 1.7, 0.2),
    new LimelightCommand(limelight, swerve, true, -0.2, 0),
    new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , 5.2, 0 , 0.4, 0.51)
    );
  }
}