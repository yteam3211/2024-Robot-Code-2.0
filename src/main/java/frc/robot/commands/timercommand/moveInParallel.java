// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.next2human;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.robot.subsystems.armSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveInParallel extends ParallelCommandGroup {
  collectAtuoCommand collectAtuoCommand;
  collectWheels collectWheels;
  timeSetPointCollectCommand timeSetPointCollectCommand;
  CollectSubsystem collectSubsystem;
  TimerArmPosition timerArmPosition;
  armSubsystem armSubsystem;
  private final Swerve s_Swerve;


  /** Creates a new collectInParallel. */
  public moveInParallel(collectAtuoCommand collectAtuoCommand,Swerve s_Swerve,
   armSubsystem armSubsystem,
    TimerArmPosition timerArmPosition,
     timeSetPointCollectCommand timeSetPointCollectCommand,
     next2human next2human,
     CollectSubsystem collectSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.collectSubsystem = collectSubsystem;
    this.timerArmPosition = timerArmPosition;
    this.armSubsystem = armSubsystem;
    this.s_Swerve = s_Swerve;


    addCommands(new TimerArmPosition(armSubsystem, -63.5,0.8,4),
    new timeSetPointCollectCommand(collectSubsystem ,3000), next2human.getAutoCommand(s_Swerve));
  }
}
