// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.centerToRampa;
import frc.robot.autos.next2human;
import frc.robot.autos.rampa;
import frc.robot.commands.collectOutput;
import frc.robot.commands.setPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.robot.subsystems.armSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveInParallel extends ParallelCommandGroup {
  Swerve s_Swerve;
  CollectSubsystem collectSubsystem;
  collectWheels collectWheels;
  double point;
  double delayForTheCollect;
  double collectSeconds;
  double collectDelay;

  /** Creates a new collectInParallel. */
  public moveInParallel(Swerve s_Swerve, CollectSubsystem collectSubsystem,
  collectWheels collectWheels,
   double point, double collectSeconds, double collectDelay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.s_Swerve = s_Swerve;
    this.collectSubsystem = collectSubsystem;
    this.collectWheels = collectWheels;
    this.point = point;
    this.collectSeconds = collectSeconds;
    this.collectDelay = collectDelay;

    

    addCommands(
    new timeSetPointCollectCommand(collectSubsystem, point, collectSeconds, collectDelay),
    new collectOutput(collectWheels, 0.5, 0.5),
    centerToRampa.getAutoCommand(s_Swerve)
    );
    
  }
}

