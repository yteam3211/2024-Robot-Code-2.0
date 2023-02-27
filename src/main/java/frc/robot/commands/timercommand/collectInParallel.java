// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class collectInParallel extends ParallelCommandGroup {
  collectAtuoCommand collectAtuoCommand;
  collectWheels collectWheels;
  timeSetPointCollectCommand timeSetPointCollectCommand;
  CollectSubsystem collectSubsystem;

  /** Creates a new collectInParallel. */
  public collectInParallel(collectWheels collectWheels,collectAtuoCommand collectAtuoCommand,timeSetPointCollectCommand timeSetPointCollectCommand,CollectSubsystem collectSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.collectAtuoCommand = collectAtuoCommand;
    this.collectSubsystem = collectSubsystem;
    this.collectWheels = collectWheels;
    this.timeSetPointCollectCommand = timeSetPointCollectCommand;


    addCommands(new collectAtuoCommand(collectWheels,0.5,0.5,10),
    new timeSetPointCollectCommand(collectSubsystem ,3000, 7));
  }
}
