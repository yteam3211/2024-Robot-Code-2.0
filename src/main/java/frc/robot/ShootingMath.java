package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.dashboard.SuperSystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.motor.SuperTalonFX;
import frc.util.vision.Limelight;
import frc.util.vision.Limelight.limelightCameraMode;

/**
  * a class contains all of the neccesary math for shooting
  */
public class ShootingMath extends SuperSystem {

    public Swerve swerve;
    public ElevatorSubsystem elevatorSubsystem;
    public PitchingSubsystem pitchingSubsystem;
    public Limelight limelight;
    public double distanceFromLimelightToSpeaker;
    public double hightShootingFromFloor;
    public double hightShootingToSpeaker;
    public double distanceFromShooterToSpeaker;
    public double angleToSpeakerRadians;
    public double angleToSpeakerDegrees;

    public ShootingMath(Swerve swerve, ElevatorSubsystem elevatorSubsystem, PitchingSubsystem pitchingSubsystem, Limelight limelight) { 
        super("Shooting Math");
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.pitchingSubsystem = pitchingSubsystem;
        this.limelight = limelight;
        // setDefaultCommand(new PitchCommand(limelight, pitchingSubsystem, elevatorSubsystem, this));
    }

    public static double getSpeakerShootingDistance(Swerve swerve){
      double distance = Math.sqrt(Math.pow(AllianceSpecs.SpeakerPose.getY() - swerve.getPose().getY(), 2) + Math.pow(AllianceSpecs.SpeakerPose.getX() - swerve.getPose().getX(),2));
      distance *= 1000;
      return distance;
    }

    public static double getEstematedSpeakerShootingAngle(Swerve swerve){
        double estematedAngle = Math.toDegrees(Math.atan(Math.abs(AllianceSpecs.SpeakerPose.getY() - swerve.getPose().getY()) / Math.abs(AllianceSpecs.SpeakerPose.getX() - swerve.getPose().getX())));
        if(AllianceSpecs.SpeakerPose.getY() - swerve.getPose().getY() < 0){
            estematedAngle *= -1;
        }
        return estematedAngle;
    }

    public boolean isShootingRange(Swerve swerve){
        return getSpeakerShootingDistance(swerve) < Constants.MAX_SHOOTING_RANGE;
    }


  /**
   * get the vertical hight of the Limelight lens from the pivot point of the shooting system
   * @return the limelight vertical hight in Millimeters
   */
    public double getVerticalLimelightHightFromPivot(){
        return Math.sin(Math.toRadians(pitchingSubsystem.getAbsolutePosition() + Constants.LIMELIGHT_OFFSET_ANGLE_FROM_PIVOT)) * Constants.LIMELIGHT_TO_PIVOT;
      }

  /**
   * get the horizontal distance of the Limelight lens from the pivot point of the shooting system
   * @return the limelight horizontal distance in Millimeters
   */
    public double getHorizontalLimelightDistanceFromPivot(){
        return Math.cos(Math.toRadians(pitchingSubsystem.getAbsolutePosition() + Constants.LIMELIGHT_OFFSET_ANGLE_FROM_PIVOT)) * Constants.LIMELIGHT_TO_PIVOT;
      }

    public Pose2d getLimelightFromRobotCenter(Swerve  swerve){
      double LLToCenter = getHorizontalLimelightDistanceFromPivot() + Constants.ROBOT_CENTER_TO_PIVOT;
      double Xpos = Math.cos(swerve.getPose().getRotation().getRadians()) * LLToCenter;
      double Ypos = Math.sin(swerve.getPose().getRotation().getRadians()) * LLToCenter;
      return new Pose2d(Xpos, Ypos, new Rotation2d());
    }    
      /**
   * get the vertical hight of the Limelight lens from the floor
   * @return the limelight vertical hight in Millimeters
   */
    public double getVerticalLimelightHightFromfloor(ElevatorSubsystem eleavatorSubsystem){
      return Constants.FLOOR_TO_CLOSE_ELEAVATOR + eleavatorSubsystem.getElevatorHight() + Constants.RIDER_BOTTOM_TO_PITCH_PIVOT_VERTICAL + getVerticalLimelightHightFromPivot();
    }

    public double getAngleToSpeaker(ElevatorSubsystem elevatorSubsystem, Limelight limelight){
      distanceFromLimelightToSpeaker = getSpeakerShootingDistance(swerve);
      hightShootingFromFloor = (getVerticalLimelightHightFromfloor(elevatorSubsystem) + Constants.VERTICAL_LIMELIGHT_TO_CENTER_SHOOTER);
      hightShootingToSpeaker = Constants.SPEAKER_HIGHT - (getVerticalLimelightHightFromfloor(elevatorSubsystem) + Constants.VERTICAL_LIMELIGHT_TO_CENTER_SHOOTER);
      distanceFromShooterToSpeaker = (distanceFromLimelightToSpeaker + Constants.HORIZONTAL_LIMELIGHT_TO_CENTER_SHOOTER);
      angleToSpeakerRadians = Math.atan(hightShootingToSpeaker / distanceFromShooterToSpeaker);
      angleToSpeakerDegrees = Math.toDegrees(angleToSpeakerRadians);
      return angleToSpeakerDegrees;
    } 

    public void setShootingCondition(Boolean shootingCondition){
      SmartDashboard.putBoolean("Shootable", shootingCondition);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getAngleToSpeaker(elevatorSubsystem, Robot.m_robotContainer.getLimelight());
    getTab().putInDashboard("hight shooter from floor", hightShootingFromFloor, false);
    getTab().putInDashboard("hight Shooting To Speaker", hightShootingToSpeaker, false);
    getTab().putInDashboard("limelight to AprilTag", distanceFromShooterToSpeaker, false);
    getTab().putInDashboard("shooting angle", getAngleToSpeaker(elevatorSubsystem, limelight), false);
    getTab().putInDashboard("get Vertical Limelight Hight From Pivot", getVerticalLimelightHightFromPivot(), false);
    getTab().putInDashboard("Swerve Angle", getEstematedSpeakerShootingAngle(swerve), false);
    SmartDashboard.putNumber("target shooting Angle", getEstematedSpeakerShootingAngle(swerve));
    // getTab().putInDashboard("Limelight Vertical Hight", getVerticalLimelightHightFromfloor(elevatorSubsystem), false);
  }
}
