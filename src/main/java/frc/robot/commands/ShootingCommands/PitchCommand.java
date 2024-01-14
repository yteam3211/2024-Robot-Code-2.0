// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PitchingSubsystem;
import frc.util.vision.Limelight;

// @author Noya Aberjil

public class PitchCommand extends Command {
  private Limelight limelight;
  private PitchingSubsystem pitchingSubsystem;
  private double angleToGoalDegrees ;
  private double angleToGoalRadians;
  private double distanceFromLimelightToGoalCM;
  private double angleToSpeakerRadians;
  private double angleToSpeakerDegrees;

  /** Creates a new PichCommand. */
  public PitchCommand(Limelight limelight, PitchingSubsystem pitchingSubsystem) {
    this.limelight = limelight;
    this.pitchingSubsystem = pitchingSubsystem;
    addRequirements(pitchingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleToGoalDegrees = Constants.LIMELIGHT_ANGLE_FROM_VERTICAL + limelight.getY();
    angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    distanceFromLimelightToGoalCM = (Constants.SPEAKER_APRILTAG_HIGHT - Constants.LIMELIGHT_LENS_HIGHT_CM)/ Math.tan(angleToGoalRadians);
    angleToSpeakerRadians = Math.atan(distanceFromLimelightToGoalCM / Constants.SPEAKER_HIGHT);
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
