// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.util.vision.Limelight;

// @author Noya Aberjil

public class PitchCommand extends Command {
  private Limelight limelight;
  private PitchingSubsystem pitchingSubsystem;
  ElevatorSubsystem eleavatorSubsystem;
  private double angleToGoalDegrees ;
  private double angleToGoalRadians;
  private double distanceFromLimelightToSpeaker;
  private double angleToSpeakerRadians;
  private double angleToSpeakerDegrees;
  private double hightLimelightToApriltag;
  private double hightShootingToSpeaker;
  private double distanceFromShooterToSpeaker;


  /** Creates a new PichCommand. */
  public PitchCommand(Limelight limelight, PitchingSubsystem pitchingSubsystem, ElevatorSubsystem eleavatorSubsystem) {
    this.limelight = limelight;
    this.pitchingSubsystem = pitchingSubsystem;
    this.eleavatorSubsystem = eleavatorSubsystem;
    addRequirements(pitchingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!limelight.isValid()){
      pitchingSubsystem.setPosition(20);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hightLimelightToApriltag = Constants.SPEAKER_APRILTAG_HIGHT - pitchingSubsystem.getVerticalLimelightHightFromfloor(eleavatorSubsystem);
    if(limelight.isValid()){
      distanceFromLimelightToSpeaker = limelight.getDistanceToTarget(hightLimelightToApriltag, pitchingSubsystem.getAbsolutePosition());
    }
    hightShootingToSpeaker = Constants.SPEAKER_HIGHT - (pitchingSubsystem.getVerticalLimelightHightFromfloor(eleavatorSubsystem) + Constants.VERTICAL_LIMELIGHT_TO_CENTER_SHOOTER);
    distanceFromShooterToSpeaker = distanceFromLimelightToSpeaker + Constants.HORIZONTAL_LIMELIGHT_TO_CENTER_SHOOTER;
    angleToSpeakerRadians = Math.atan(hightShootingToSpeaker / distanceFromShooterToSpeaker);
    angleToSpeakerDegrees = Math.toDegrees(angleToSpeakerRadians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pitchingSubsystem.setPosition(angleToSpeakerDegrees);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
