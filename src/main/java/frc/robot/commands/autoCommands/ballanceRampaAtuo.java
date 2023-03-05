// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotButtons;
import frc.robot.autos.rampa;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.timercommand.timeSetPointCollectCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.collectGroupCommand;
import frc.robot.commands.shootingOutputCommand;
import frc.robot.commands.timercommand.TimerArmPosition;
import frc.robot.commands.timercommand.TimerGripperCommand;
import frc.robot.commands.timercommand.openInParallel;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ballanceRampaAtuo extends SequentialCommandGroup {
  /** Creates a new atuo1. */
  private final Swerve s_Swerve;
  private ShootingSubsystem ShootingSubsystem;
  private CollectSubsystem collectSubsystem;
  private GripperSubsystem gripperSubsystem;
  private armSubsystem armSubsystem;
  private collectGroupCommand collectGroupCommand;
  private timeSetPointCollectCommand timeSetPointCollectCommand;
  private TimerArmPosition TimerArmPosition;
  

  
  public ballanceRampaAtuo(Swerve swerve,
  armSubsystem armSubsystem,
  CollectSubsystem collectSubsystem,
  ShootingSubsystem ShootingSubsystem,
  GripperSubsystem gripperSubsystem
) {
    this.s_Swerve = swerve;
    this.collectSubsystem = collectSubsystem;
    this.ShootingSubsystem = ShootingSubsystem;
    this.armSubsystem = armSubsystem;
    
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  
    addCommands(new shootingOutputCommand(ShootingSubsystem, 0.5, 6930),  rampa.getAutoCommand(s_Swerve),
    new TeleopSwerve(swerve, () -> 0, () -> 0, () -> 0, () -> false,0.7));   
  
  }
}
