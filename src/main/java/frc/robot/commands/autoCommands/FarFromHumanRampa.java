// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import frc.robot.autos.FarFromHumanAndRamp;
import frc.robot.autos.next2human;
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
// import frc.robot.commands.ClosingCollectGroupCommand;
import frc.robot.commands.resetCommand;
import frc.robot.commands.shootingOutputCommand;
import frc.robot.commands.timercommand.TimerArmPosition;
import frc.robot.commands.timercommand.moveInParallel;

    // addCommands(new FooCommand(), new BarCommand());
public class FarFromHumanRampa extends SequentialCommandGroup {
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
  public FarFromHumanRampa(Swerve swerve,
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
    new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, 6000, 0.4),
    new moveInParallel(swerve, collectSubsystem, collectWheels, FarFromHumanAndRamp.getAutoCommand(swerve), 290, 5, 1),
    new BalanceCommand(swerve)
    );
  }
}
