package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;
import frc.robot.dashboard.SuperSystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.motor.SuperTalonFX;
import frc.util.vision.Limelight;

/**
  * a class contains all of the neccesary math for shooting
  */
public class ShootingMath extends SuperSystem {

    public Swerve swerve;
    public ElevatorSubsystem elevatorSubsystem;
    public PitchingSubsystem pitchingSubsystem;
    public double hightLimelightToApriltag;
    public double distanceFromLimelightToSpeaker;
    public double hightShootingToSpeaker;
    public double distanceFromShooterToSpeaker;
    public double angleToSpeakerRadians;
    public double angleToSpeakerDegrees;

        public ShootingMath(String nameSystem, Swerve swerve, ElevatorSubsystem elevatorSubsystem, PitchingSubsystem pitchingSubsystem) { 
        super("Shooting Math");
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.pitchingSubsystem = pitchingSubsystem;
    }

    public static double getEstematedSpeakerShootingDistance(Swerve swerve){
        return Math.sqrt(Math.pow(AllianceSpecs.speakerPos.getY() - swerve.getPose().getY(), 2) / Math.pow(AllianceSpecs.speakerPos.getX() - swerve.getPose().getX(),2));
    }

    public static double getEstematedSpeakerShootingAngle(Swerve swerve){
        double estematedAngle = Math.toDegrees(Math.atan(Math.abs(AllianceSpecs.speakerPos.getY() - swerve.getPose().getY()) / Math.abs(AllianceSpecs.speakerPos.getX() - swerve.getPose().getX())));
        if(AllianceSpecs.speakerPos.getY() - swerve.getPose().getY() < 0){
            estematedAngle *= -1;
        }
        return estematedAngle;
    }

    public static boolean isShootingRange(Swerve swerve){
        return getEstematedSpeakerShootingDistance(swerve) < Constants.MAX_SHOOTING_RANGE;
    }

      public double getAngleToSpeaker(ElevatorSubsystem elevatorSubsystem, Limelight limelight){
    // if(!limelight.isValid()){
    //   if(getAbsolutePosition() < -5 || getAbsolutePosition() > 15 || Math.abs(Robot.m_robotContainer.getSwerve().getYaw().getDegrees() - Robot.m_robotContainer.getSwerve().getEstematedSpeakerShootingAngle()) > Constants.ESTEMATED_ANGLE_TRESHOLD){ // TODO: fix it
    //     setPosition(5);
    //   }else {
    //     elevatorSubsystem.setPosition(elevatorSubsystem.getElevatorHight() + 5);  
    //   }
    // }
    hightLimelightToApriltag = Constants.SPEAKER_APRILTAG_HIGHT - getVerticalLimelightHightFromfloor(elevatorSubsystem);
    distanceFromLimelightToSpeaker = limelight.getDistanceToTarget(hightLimelightToApriltag, getAbsolutePosition());
    hightShootingToSpeaker = Constants.SPEAKER_HIGHT - (getVerticalLimelightHightFromfloor(elevatorSubsystem) + Constants.VERTICAL_LIMELIGHT_TO_CENTER_SHOOTER);
    distanceFromShooterToSpeaker = distanceFromLimelightToSpeaker + Constants.HORIZONTAL_LIMELIGHT_TO_CENTER_SHOOTER;
    angleToSpeakerRadians = Math.atan(hightShootingToSpeaker / distanceFromShooterToSpeaker);
    angleToSpeakerDegrees = Math.toDegrees(angleToSpeakerRadians);
    return angleToSpeakerDegrees;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getAngleToSpeaker(elevatorSubsystem, Robot.m_robotContainer.getLimelight());
    // if((getAbsolutePosition() > Constants.MAX_PITCHING_ANGLE) ||(getAbsolutePosition() < Constants.MIN_PITCHING_ANGLE)) //TODO: 
    // {
    //   masterPitchingMotor.set (ControlMode.PercentOutput, 0);
    // }
    getTab().putInDashboard("limelight to AprilTag", distanceFromLimelightToSpeaker, false);
    getTab().putInDashboard("Limelight Vertical Hight", getVerticalLimelightHightFromfloor(elevatorSubsystem), false);
  }


}
