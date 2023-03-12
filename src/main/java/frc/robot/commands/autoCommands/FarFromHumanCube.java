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
import frc.robot.commands.ArmCollectCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.commands.ShootingGroupCommand;
import frc.robot.commands.StartAuto;
// import frc.robot.commands.ClosingCollectGroupCommand;
import frc.robot.commands.resetCommand;
import frc.robot.commands.shootingOutputCommand;
import frc.robot.commands.timercommand.TimerArmPosition;
import frc.robot.commands.timercommand.moveInParallel;

    // addCommands(new FooCommand(), new BarCommand());
public class FarFromHumanCube extends SequentialCommandGroup {
  private Swerve s_Swerve;
  private CartridgeSubsystem cartridgeSubsystem;
  private CollectSubsystem collectSubsystem;
  // private ClosingCollectGroupCommand collectGroupCommand;
  private timeSetPointCollectCommand timeSetPointCollectCommand;
  private TimerArmPosition TimerArmPosition;
  private collectWheels collectWheels;
  private armCollectSubsystem armCollectSubsystem;
  private shootingSubsystem shootingSubsystem;
  private Limelight limelight;
  /** Creates a new FarFromHuman. */
  public FarFromHumanCube(Swerve swerve,
  CollectSubsystem collectSubsystem,
  CartridgeSubsystem ShootingSubsystem,
  collectWheels collectWheels, armCollectSubsystem armCollectSubsystem, Limelight limelight, shootingSubsystem shootingSubsystem) {
    this.s_Swerve = swerve;
    this.collectSubsystem = collectSubsystem;
    this.cartridgeSubsystem = cartridgeSubsystem;
    this.collectWheels = collectWheels;
    this.limelight = limelight;
    this.shootingSubsystem = shootingSubsystem;
    this.armCollectSubsystem = armCollectSubsystem;
    addCommands(new InstantCommand(() -> swerve.zeroGyro()), new resetCommand(shootingSubsystem, collectSubsystem, armCollectSubsystem, cartridgeSubsystem),
    new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, 0.5, 0.4),
    new StartAuto(AutoCommand.getAutoCommand(swerve, "far from human + cube - start"), armCollectSubsystem, swerve),
    new moveInParallel(swerve, collectSubsystem, collectWheels, armCollectSubsystem, cartridgeSubsystem, AutoCommand.getAutoCommand(swerve, "far from human + cube"), 290, 5.2, 5, 1, -0.3, 0.2),
    new LimelightCommand(limelight, swerve, true, -0.2, 0),
    new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , 5.2, 0 , 0.3, 0.5)
    );
  }
}
