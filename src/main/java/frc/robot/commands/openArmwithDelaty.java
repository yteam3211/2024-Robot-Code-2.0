// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.shootingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class openArmwithDelaty extends ParallelDeadlineGroup {

  public openArmwithDelaty(shootingSubsystem shootingSubsystem, armCollectSubsystem armCollectSubsystem,
      CartridgeSubsystem cartridgeSubsystem, double position, double seconds, double CartridgeOutput, double velocity) {
    super(
        new WaitCommand(1.5),
        new ArmCollectCommand(armCollectSubsystem, position, seconds),
        new SequentialCommandGroup(
            new WaitCommand(0.2),
            new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem, velocity, CartridgeOutput)
        )
    );
  }
}
