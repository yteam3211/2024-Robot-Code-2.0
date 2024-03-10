// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands.PitchCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;

public class PitchAMP extends Command {
  /** Creates a new PitchPos. */
    private PitchingSubsystem pitchingSubsystem;
    private double angleDegrees;
    private ElevatorSubsystem elevatorSubsystem;
    private double eleveatorHight;
  public PitchAMP(PitchingSubsystem pitchingSubsystem,ElevatorSubsystem elevatorSubsystem, double angleDegrees, double eleveatorHight) {
    this.pitchingSubsystem = pitchingSubsystem;
    this.angleDegrees = angleDegrees;
    this.elevatorSubsystem = elevatorSubsystem;
    this.eleveatorHight = eleveatorHight;
    addRequirements(pitchingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******** inside PitchPos");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("pitch" + pitchingSubsystem.getAbsolutePosition());
    if (elevatorSubsystem.getElevatorHight() > eleveatorHight) {
       pitchingSubsystem.setPosition(angleDegrees);  

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("********exit PitchPos");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // return false;
     return Math.abs(pitchingSubsystem.getAbsolutePosition() - angleDegrees) < 2;
    
    // return true;
  }
}
