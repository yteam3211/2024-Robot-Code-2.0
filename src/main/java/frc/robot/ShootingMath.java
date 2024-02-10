package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;
import frc.robot.commands.ShootingCommands.PitchCommand;
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
      double distance = Math.sqrt(Math.pow(AllianceSpecs.speakerPos.getY() - swerve.getPose().getY(), 2) + Math.pow(AllianceSpecs.speakerPos.getX() - swerve.getPose().getX(),2));
      distance *= 1000;
      return distance;
    }

    public static double getEstematedSpeakerShootingAngle(Swerve swerve){
        double estematedAngle = Math.toDegrees(Math.atan(Math.abs(AllianceSpecs.speakerPos.getY() - swerve.getPose().getY()) / Math.abs(AllianceSpecs.speakerPos.getX() - swerve.getPose().getX())));
        if(AllianceSpecs.speakerPos.getY() - swerve.getPose().getY() < 0){
            estematedAngle *= -1;
        }
        return estematedAngle;
    }

    public static boolean isShootingRange(Swerve swerve){
        return getSpeakerShootingDistance(swerve) < Constants.MAX_SHOOTING_RANGE;
    }

    public double getVerticalLimelightHightFromPivot(){
        return Math.sin(Math.toRadians(pitchingSubsystem.getAbsolutePosition() + Constants.LIMELIGHT_OFFSET_ANGLE_FROM_PIVOT)) * Constants.LIMELIGHT_TO_PIVOT;
      }
    
    public double getVerticalLimelightHightFromfloor(ElevatorSubsystem eleavatorSubsystem){
      return Constants.FLOOR_TO_CLOSE_ELEAVATOR + eleavatorSubsystem.getElevatorHight() + Constants.RIDER_BOTTOM_TO_PITCH_PIVOT_VERTICAL + getVerticalLimelightHightFromPivot();
    }

    public double getAngleToSpeaker(ElevatorSubsystem elevatorSubsystem, Limelight limelight){
      // if(!limelight.isValid()){
      //   if(getAbsolutePosition() < -5 || getAbsolutePosition() > 15 || Math.abs(Robot.m_robotContainer.getSwerve().getYaw().getDegrees() - Robot.m_robotContainer.getSwerve().getEstematedSpeakerShootingAngle()) > Constants.ESTEMATED_ANGLE_TRESHOLD){ // TODO: fix it
      //     setPosition(5);
      //   }else {
      //     elevatorSubsystem.setPosition(elevatorSubsystem.getElevatorHight() + 5);
      //   }
      // }
      distanceFromLimelightToSpeaker = getSpeakerShootingDistance(swerve);
      hightShootingToSpeaker = Constants.SPEAKER_HIGHT - (getVerticalLimelightHightFromfloor(elevatorSubsystem) + Constants.VERTICAL_LIMELIGHT_TO_CENTER_SHOOTER);
      distanceFromShooterToSpeaker = (distanceFromLimelightToSpeaker + Constants.HORIZONTAL_LIMELIGHT_TO_CENTER_SHOOTER);
      angleToSpeakerRadians = Math.atan(hightShootingToSpeaker / distanceFromShooterToSpeaker);
      angleToSpeakerDegrees = Math.toDegrees(angleToSpeakerRadians);
      return angleToSpeakerDegrees;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getAngleToSpeaker(elevatorSubsystem, Robot.m_robotContainer.getLimelight());
    getTab().putInDashboard("hight Shooting To Speaker", hightShootingToSpeaker, false);
    getTab().putInDashboard("limelight to AprilTag", distanceFromLimelightToSpeaker, false);
    getTab().putInDashboard("shooting angle", getAngleToSpeaker(elevatorSubsystem, limelight), false);
    getTab().putInDashboard("get Vertical Limelight Hight From Pivot", getVerticalLimelightHightFromPivot(), false);

    // getTab().putInDashboard("Limelight Vertical Hight", getVerticalLimelightHightFromfloor(elevatorSubsystem), false);
  }
}
