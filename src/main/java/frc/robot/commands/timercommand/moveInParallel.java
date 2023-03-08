// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.timercommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.centerToRampa;
import frc.robot.autos.next2human;
import frc.robot.autos.rampa;
import frc.robot.commands.collectWheelsCommand;
import frc.robot.commands.setPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.robot.subsystems.armSubsystem;
import frc.robot.commands.timercommand.TimerCollectWheels;;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveInParallel extends ParallelCommandGroup {
  private Swerve s_Swerve;
  private CollectSubsystem collectSubsystem;
  private collectWheels collectWheels;
  private double point;
  private double delayForTheCollect;
  private double collectSeconds;
  private double collectDelay;
  private Command movment;

  /** Creates a new collectInParallel. */
  public moveInParallel(Swerve s_Swerve, CollectSubsystem collectSubsystem,
  collectWheels collectWheels, Command movment,
   double point, double collectSeconds, double collectDelay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.movment = movment;
    this.s_Swerve = s_Swerve;
    this.collectSubsystem = collectSubsystem;
    this.collectWheels = collectWheels;
    this.point = point;
    this.collectSeconds = collectSeconds;
    this.collectDelay = collectDelay;
    

    addCommands(
    movment,
    new timeSetPointCollectCommand(collectSubsystem, point, collectSeconds, collectDelay),
    new TimerCollectWheels(collectWheels, -0.5, -0.15, collectSeconds + 1, collectDelay)
    
    );
  }
}

