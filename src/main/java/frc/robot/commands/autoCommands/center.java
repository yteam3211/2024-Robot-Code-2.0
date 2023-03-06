// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotButtons;
import frc.robot.autos.centerToRampa;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.collectGroupCommand;
import frc.robot.commands.shootingOutputCommand;
import frc.robot.commands.timercommand.TimerArmPosition;
import frc.robot.commands.timercommand.TimerGripperCommand;
import frc.robot.commands.timercommand.moveInParallel;
import frc.robot.commands.timercommand.openInParallel;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class center extends SequentialCommandGroup {
  /** Creates a new atuo1. */
  private Swerve s_Swerve;
  private CartridgeSubsystem ShootingSubsystem;
  private CollectSubsystem collectSubsystem;
  private GripperSubsystem gripperSubsystem;
  private collectGroupCommand collectGroupCommand;
  private timeSetPointCollectCommand timeSetPointCollectCommand;
  private TimerArmPosition TimerArmPosition;
  private collectWheels collectWheels;
  

  
  public center(Swerve swerve,
  armSubsystem armSubsystem,
  CollectSubsystem collectSubsystem,
  CartridgeSubsystem ShootingSubsystem,
  GripperSubsystem gripperSubsystem,
  collectWheels collectWheels
) {
    this.s_Swerve = swerve;
    this.collectSubsystem = collectSubsystem;
    this.ShootingSubsystem = ShootingSubsystem;
    this.collectWheels = collectWheels;
    
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  
    addCommands(new InstantCommand(() -> swerve.zeroGyro()), new shootingOutputCommand(ShootingSubsystem, 0.5, 6930),
    new moveInParallel(swerve, collectSubsystem, collectWheels, centerToRampa.getAutoCommand(s_Swerve), 290, 7, 2),
    // centerToRampa.getAutoCommand(s_Swerve),
    new BalanceCommand(swerve)
    );  
  }
}
