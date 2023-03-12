// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.shootingSubsystem;

public class ShootingCommand extends CommandBase {
  private final shootingSubsystem shootingSubsystem;
  private final CartridgeSubsystem cartridgeSubsystem;
  private final armCollectSubsystem armCollectSubsystem;
  private double ShootingOutput;
  private boolean isShootingUp = false;
  private double RPM;
  private double CartridgeOutput;
  private Timer timer = new Timer();
  /** Creates a new ShootingCommand. */
  public ShootingCommand(shootingSubsystem shootingSubsystem, CartridgeSubsystem cartridgeSubsystem, armCollectSubsystem armCollectSubsystem, double ShootingOutput, double CartridgeOutput) {
  this.shootingSubsystem = shootingSubsystem;
  this.cartridgeSubsystem = cartridgeSubsystem;
  this.armCollectSubsystem = armCollectSubsystem;
  this.ShootingOutput = ShootingOutput;
  this.RPM = RPM;
  this.CartridgeOutput = CartridgeOutput;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // new InstantCommand(() -> armCollectSubsystem.setArmCollectPosition(5)).schedule();
    // new WaitCommand(10).schedule();
    isShootingUp = false;
    timer.reset();
    timer.start();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("right output" + shootingSubsystem.GetRightShootingWheelsOutput());
    System.out.println("left output" + shootingSubsystem.getLeftShootingWheelsOutput());
    shootingSubsystem.setShootingOutput(ShootingOutput);
    if ((shootingSubsystem.GetRightShootingWheelsOutput() >= (ShootingOutput * -1) + 0.02) && (shootingSubsystem.getLeftShootingWheelsOutput() >= ShootingOutput - 0.02)) {
      if(cartridgeSubsystem.GetPosition() >= cartridgeSubsystem.max){
        cartridgeSubsystem.setOutput(0);
        isShootingUp = true;
      }else if(!isShootingUp){
        cartridgeSubsystem.setOutput(CartridgeOutput);
      }
    }

    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridgeSubsystem.setOutput(0);
    // timer.delay(1.3);
    shootingSubsystem.setShootingOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
    // return cartridgeSubsystem.GetPosition() >= cartridgeSubsystem.max;

    
  }
}

