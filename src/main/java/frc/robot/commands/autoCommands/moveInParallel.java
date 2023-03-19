// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.autos.AutoCommand;
import frc.robot.commands.ArmCommands.ArmCollectCommand;
import frc.robot.commands.ArmCommands.armCollectOutput;
import frc.robot.commands.IntakeCommands.collectWheelsCommand;
import frc.robot.commands.IntakeCommands.setPointCollectCommand;
import frc.robot.commands.ShootingCommnads.CartridgeOutputCommand;
import frc.robot.commands.ShootingCommnads.CubeFixtureGroupCommand;
import frc.robot.commands.SwereCommands.TurnToZeroCommand;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.util.commands.TurnInPlace;
import frc.robot.subsystems.armSubsystem;
import frc.robot.commands.timercommand.TimerCollectWheels;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveInParallel extends SequentialCommandGroup {
  /** Creates a new collectInParallel. */
  public moveInParallel(Swerve s_Swerve, CollectSubsystem collectSubsystem,
  collectWheels collectWheels, armCollectSubsystem armCollectSubsystem, CartridgeSubsystem cartridgeSubsystem, Command movment,
   double collectPoint, double armCollectPoint, double collectSeconds, double collectDelay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
    new ParallelCommandGroup(
      movment,
      new CartridgeOutputCommand(cartridgeSubsystem, 0.2, 10),
      new timeSetPointCollectCommand(collectSubsystem, armCollectSubsystem, collectPoint, armCollectPoint, collectSeconds, collectDelay),
      new TimerCollectWheels(collectWheels, -0.5, -0.15, collectSeconds + 1, collectDelay),
      new AutoCubeFixture(cartridgeSubsystem, collectSeconds - 0.5),
      new ArmCollectCommand(armCollectSubsystem, 0.3 , collectSeconds + 1.5)
      ),
      new TurnToZeroCommand(s_Swerve)
    );
  }
}

