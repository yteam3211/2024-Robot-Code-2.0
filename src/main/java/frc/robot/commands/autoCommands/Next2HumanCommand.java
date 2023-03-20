// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import frc.robot.autos.AutoCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;
import frc.robot.subsystems.shootingSubsystem;
import frc.util.vision.Limelight;
import frc.robot.commands.ShootingCommnads.ShootingCommand;
import frc.robot.commands.ShootingCommnads.ShootingGroupCommand;
import frc.robot.commands.SwereCommands.BalanceCommand;
import frc.robot.commands.SwereCommands.LimelightCommand;
import frc.robot.commands.SwereCommands.TurnToZeroCommand;
// import frc.robot.commands.ClosingCollectGroupCommand;
import frc.robot.commands.resetCommand;
import frc.robot.commands.ArmCommands.ArmCollectCommand;
import frc.robot.commands.ArmCommands.armCollectOutput;
import frc.robot.commands.ShootingCommnads.CartridgeOutputCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Next2HumanCommand extends SequentialCommandGroup {
  /** Creates a new Next2Human. */
  // private final Swerve s_Swerve;

  public Next2HumanCommand(Swerve swerve,
  CollectSubsystem collectSubsystem,
  CartridgeSubsystem cartridgeSubsystem,
  collectWheelsSubsystem collectWheels, armCollectSubsystem armCollectSubsystem, Limelight limelight, shootingSubsystem shootingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> swerve.zeroGyro()), new resetCommand(shootingSubsystem, collectSubsystem, armCollectSubsystem, cartridgeSubsystem),
    new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, 0.75, 0.3),
    new StartAuto(AutoCommand.getAutoCommand(swerve, "Next 2 Human - start", 3), armCollectSubsystem, swerve),
    new moveInParallel(swerve, collectSubsystem, collectWheels, armCollectSubsystem, cartridgeSubsystem, AutoCommand.getAutoCommand(swerve, "next 2 human & 1 cube", 3), Constants.COLLECT_OPEN_POSITION, Constants.ARM_OPEN_POSITION, 2, 0.2),
    new LimelightCommand(limelight, swerve, true, -0.2, 0),
    new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , Constants.SHOOTING_LOW)
    ); 
  }
}
