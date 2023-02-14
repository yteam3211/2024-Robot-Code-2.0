// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.TestAuto;
import frc.robot.subsystems.CollectSubsyste;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.armSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class atuo1 extends SequentialCommandGroup {
  /** Creates a new atuo1. */
  private RobotButtons robotButtons = new RobotButtons();
    private final Swerve s_Swerve = new Swerve(robotButtons);
  private ShootingSubsystem ShootingSubsystem = new ShootingSubsystem();
  private CollectSubsyste CollectSubsystem = new CollectSubsyste();
  private armSubsystem armSubsystem = new armSubsystem();
  private armPosition humanArmPosition = new armPosition(armSubsystem, -7);
  private armPosition secondArmPosition = new armPosition(armSubsystem, -25.4);
  private final TestAuto testAuto = new TestAuto(); 
  public atuo1() {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(new resetCommand(ShootingSubsystem,  CollectSubsystem, armSubsystem), testAuto.getAutoCommand(s_Swerve), humanArmPosition);
  }
}
