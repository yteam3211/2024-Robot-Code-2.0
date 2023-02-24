// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.robot.subsystems.armSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class openInParallel extends ParallelCommandGroup {
  collectAtuoCommand collectAtuoCommand;
  collectWheels collectWheels;
  timeSetPointCollectCommand timeSetPointCollectCommand;
  CollectSubsystem collectSubsystem;
  TimerArmPosition timerArmPosition;
  armSubsystem armSubsystem;


  /** Creates a new collectInParallel. */
  public openInParallel(collectAtuoCommand collectAtuoCommand, armSubsystem armSubsystem, TimerArmPosition timerArmPosition, timeSetPointCollectCommand timeSetPointCollectCommand,CollectSubsystem collectSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.collectSubsystem = collectSubsystem;
    this.timerArmPosition = timerArmPosition;
    this.armSubsystem = armSubsystem;


    addCommands(new TimerArmPosition(armSubsystem, -63.5,0.8,4),
    new timeSetPointCollectCommand(collectSubsystem ,3000));
  }
}
