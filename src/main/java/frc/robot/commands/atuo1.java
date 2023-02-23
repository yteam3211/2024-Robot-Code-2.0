// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.TestAuto;
import frc.robot.autos.centerToRampa;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.commands.timercommand.TimerArmPosition;
import frc.robot.commands.timercommand.TimerGripperCommand;
import frc.robot.commands.timercommand.openInParallel;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class atuo1 extends SequentialCommandGroup {
  /** Creates a new atuo1. */
  private final Swerve s_Swerve;
  private ShootingSubsystem ShootingSubsystem;
  private CollectSubsystem collectSubsystem;
  private armSubsystem armSubsystem;
  private collectGroupCommand collectGroupCommand;
  private timeSetPointCollectCommand timeSetPointCollectCommand;
  private TimerArmPosition TimerArmPosition;
  private openInParallel openInParallel;
  private TimerGripperCommand timerGripperCommand;

  
  public atuo1(Swerve swerve,openInParallel openInParallel,
  centerToRampa centerToRampa,
  armSubsystem armSubsystem,
  TimerGripperCommand timerGripperCommand,
  CollectSubsystem collectSubsystem,
  ShootingSubsystem ShootingSubsystem
) {
    this.s_Swerve = swerve;
    this.collectSubsystem = collectSubsystem;
    this.ShootingSubsystem = ShootingSubsystem;
    this.armSubsystem = armSubsystem;
    this.openInParallel = openInParallel;
    this.timerGripperCommand = timerGripperCommand;
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  
    addCommands(new resetCommand(ShootingSubsystem, collectSubsystem, armSubsystem),
    openInParallel,
    timerGripperCommand,
    centerToRampa.getAutoCommand(s_Swerve));
  }
}
