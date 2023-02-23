// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<<< HEAD:src/main/java/frc/robot/commands/TurnToZero.java
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;

public class TurnToZero extends CommandBase {
  /** Creates a new TurnZero. */
  protected Swerve swerve;
  protected Gains gains = new Gains("pid", 0, 0, 0);
  protected PIDController pidController = new PIDController(gains);
  public TurnToZero(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
========
import frc.robot.subsystems.ShootingSubsystem;

public class InitShooting extends CommandBase {
  private final ShootingSubsystem shootingSubsystem;



  /** Creates a new InitShooting. */
  public InitShooting(ShootingSubsystem shootingSubsystem) {
    this.shootingSubsystem = shootingSubsystem ;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootingSubsystem);
>>>>>>>> origin/shooting:src/main/java/frc/robot/commands/initShooting.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setTargetPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/TurnToZero.java
    if (Math.abs(swerve.getYaw().getDegrees())> 2){
      swerve.drive(            
          new Translation2d(0.0, 0.0), 
          pidController.getOutput(Swerve.gyro.getYaw()), 
       false, //Field oriented by the controller switch
       true
      );
  }
  }
========
    System.out.println("rrrrr");
    shootingSubsystem.setOutput(-0.15);
     
      
 }
>>>>>>>> origin/shooting:src/main/java/frc/robot/commands/initShooting.java

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootingSubsystem.setOutput(0);
      shootingSubsystem.resetEncoder();
    shootingSubsystem.changeDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shootingSubsystem.GetPosition() <= shootingSubsystem.Min){
      return true;
    }
    return false;
    
  }
}
