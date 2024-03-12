// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands.PitchCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShootingMath;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.util.vision.Limelight;

// @author Noya Aberjil

public class SpeakerPitchCommand extends Command {
  private Limelight limelight;
  private PitchingSubsystem pitchingSubsystem;
  private ElevatorSubsystem eleavatorSubsystem;
  private ShootingMath shootingMath;
  protected ShootingSubsystem shootingSubsystem;
  private double targetAngle;



  /** Creates a new PichCommand. */
  public SpeakerPitchCommand(Limelight limelight, PitchingSubsystem pitchingSubsystem, ElevatorSubsystem eleavatorSubsystem, ShootingMath shootingMath, ShootingSubsystem shootingSubsystem) {
    this.limelight = limelight;
    this.pitchingSubsystem = pitchingSubsystem;
    this.eleavatorSubsystem = eleavatorSubsystem;
    this.shootingMath = shootingMath;
    this.shootingSubsystem = shootingSubsystem;
    addRequirements(pitchingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******** inside PitchCommand");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetAngle = shootingMath.getAngleToSpeaker(eleavatorSubsystem, limelight);
  //  angleToSpeakerDegrees = pitchingSubsystem.getAngleToSpeaker(eleavatorSubsystem, limelight);
    System.out.println("************ angle to shooting "+ targetAngle +" ******** current: " + pitchingSubsystem.getAbsolutePosition());
    pitchingSubsystem.setPosition(targetAngle);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pitchingSubsystem.setPosition(targetAngle);
    System.out.println("********exit PitchCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pitchingSubsystem.getAbsolutePosition() - targetAngle) < (targetAngle / 32);
  }
}
