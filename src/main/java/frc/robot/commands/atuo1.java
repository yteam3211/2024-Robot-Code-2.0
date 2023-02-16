// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.TestAuto;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.armSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class atuo1 extends SequentialCommandGroup {
  /** Creates a new atuo1. */
  private final Swerve s_Swerve;
  private ShootingSubsystem ShootingSubsystem;
  private CollectSubsystem CollectSubsystem;
  private armSubsystem armSubsystem;
  private armPosition humanArmPosition;
  private armPosition secondArmPosition;
  private collectCommand collectCommand;
  
  public atuo1(Swerve swerve,TestAuto testAuto,armPosition secondArmPosition,armPosition humanArmPosition,armSubsystem armSubsystem,CollectSubsystem CollectSubsystem,ShootingSubsystem ShootingSubsystem) {
    this.s_Swerve = swerve;
    this.CollectSubsystem = CollectSubsystem;
    this.ShootingSubsystem = ShootingSubsystem;
    this.armSubsystem = armSubsystem;
    this.humanArmPosition = humanArmPosition;
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  
    addCommands(new resetCommand(ShootingSubsystem,  CollectSubsystem, armSubsystem), secondArmPosition, testAuto.getAutoCommand(s_Swerve),collectOutput);
  }
}
