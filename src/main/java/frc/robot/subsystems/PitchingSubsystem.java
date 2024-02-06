// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperTalonFX;
import frc.util.vision.Limelight;

public class PitchingSubsystem extends SuperSystem {
  /** Creates a new PitchingSubsystem. */
  public SuperTalonFX masterPitchingMotor;
  public SuperTalonFX slavePitchingMotor;
  public CANcoder angleEncoder;
  public Gains pitchingGains;

  public PitchingSubsystem() {
    super("Pitching Subsystem");
    pitchingGains = new Gains("pitchingGains", 0.383, 0, 0.0001);
    masterPitchingMotor = new SuperTalonFX(Constants.MASTER_PITCHING_MOTOR_ID, 40, false, false, NeutralMode.Brake, pitchingGains, TalonFXControlMode.MotionMagic,8000, 5000,5); 
    slavePitchingMotor = new SuperTalonFX(masterPitchingMotor, Constants.SLAVE_PITCHING_MOTOR_ID, 40, false);
    angleEncoder = new CANcoder(Constants.PITCHING_ENCODER_ID);
    configAngleEncoder();
    resetFalconEncoder();
    getTab().addCommandToDashboard("reset falcon encoder", new InstantCommand( () -> resetFalconEncoder()));
  }

  public void SetOutput(double output){ 
    masterPitchingMotor.set(ControlMode.PercentOutput, output);
  }
  /**
   * Set the position of the motor.
   *
   * @param position position in degrees.
   */
  public void setPosition(double position){
    masterPitchingMotor.set(ControlMode.MotionMagic, degreesToFalconEncoder(position));
  }
/**
 * config the angle encoder 356 - 140
 */
  private void configAngleEncoder(){        
angleEncoder.getPosition().setUpdateFrequency(4);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.pitchingCanCoderConfig);
  }

  /**
   * Return the current angle - the angle offset 0.227051 
   *
   * @return the absolute angle of the Cancoder in degrees.
   */
  public double getAbsolutePosition(){
    double absolutePosition;
    if(Constants.PITCHING_ENCODER_OFFSET < 85 && Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) > 200){
      absolutePosition = Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) - (Constants.PITCHING_ENCODER_OFFSET + 360);
    }
    else if(Constants.PITCHING_ENCODER_OFFSET > 305 && Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) < 80){
      absolutePosition = Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) - Constants.PITCHING_ENCODER_OFFSET + 360;
    }
    else{
      absolutePosition = Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) - Constants.PITCHING_ENCODER_OFFSET;
    }
    return absolutePosition;
  }

  /**
 * reset the falcon integrated encoder
 */
  public void resetFalconEncoder(){


    masterPitchingMotor.reset(degreesToFalconEncoder(getAbsolutePosition())); 
  }
  
  /**
   * Converts given degrees to raw sensor units.
   * devide degrees by 360 to get rotatins and multiply by encoder steps per revolution amount.
   *
   * @param degrees The degrees to convert.
   * @return the amount of falcon raw sensor units.
   */
  public double degreesToFalconEncoder(double degrees){
    return (degrees / 360) * Constants.PITCHING_GEAR_RATIO * 2048;
  }

  /**
   * get the vertical hight of the Limelight lens from the pivot point of the shooting system
   * @return the limelight vertical hight in Millimeters
   */
  public double getVerticalLimelightHightFromPivot(){
    return Math.sin(getAbsolutePosition() - Constants.LIMELIGHT_OFFSET_ANGLE_FROM_PIVOT) * Constants.LIMELIGHT_TO_PIVOT;
  }

  public double getVerticalLimelightHightFromfloor(ElevatorSubsystem eleavatorSubsystem){
    return Constants.FLOOR_TO_CLOSE_ELEAVATOR + eleavatorSubsystem.getElevatorHight() + Constants.RIDER_BOTTOM_TO_PITCH_PIVOT_VERTICAL + getVerticalLimelightHightFromPivot();
  }

  public double getAngleToSpeaker(ElevatorSubsystem elevatorSubsystem, Limelight limelight){
    if(!limelight.isValid()){
      if(getAbsolutePosition() < -5 || getAbsolutePosition() > 15 || Math.abs(Robot.m_robotContainer.getSwerve().getYaw().getDegrees() - Robot.m_robotContainer.getSwerve().getEstematedSpeakerShootingAngle()) > Constants.ESTEMATED_ANGLE_TRESHOLD){ // TODO: fix it
        setPosition(5);
      }else {
        elevatorSubsystem.setPosition(elevatorSubsystem.getElevatorHight() + 5);
          
      }
    }
    double hightLimelightToApriltag = Constants.SPEAKER_APRILTAG_HIGHT - getVerticalLimelightHightFromfloor(elevatorSubsystem);
    double distanceFromLimelightToSpeaker = limelight.getDistanceToTarget(hightLimelightToApriltag, getAbsolutePosition());
    double hightShootingToSpeaker = Constants.SPEAKER_HIGHT - (getVerticalLimelightHightFromfloor(elevatorSubsystem) + Constants.VERTICAL_LIMELIGHT_TO_CENTER_SHOOTER);
    double distanceFromShooterToSpeaker = distanceFromLimelightToSpeaker + Constants.HORIZONTAL_LIMELIGHT_TO_CENTER_SHOOTER;
    double angleToSpeakerRadians = Math.atan(hightShootingToSpeaker / distanceFromShooterToSpeaker);
    double angleToSpeakerDegrees = Math.toDegrees(angleToSpeakerRadians);
    return angleToSpeakerDegrees;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if((getAbsolutePosition() > Constants.MAX_PITCHING_ANGLE) ||(getAbsolutePosition() < Constants.MIN_PITCHING_ANGLE)) //TODO: 
    // {
    //   masterPitchingMotor.set(ControlMode.PercentOutput, 0);
    // }
    getTab().putInDashboard("CANcoder ", Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()), false);
    getTab().putInDashboard("integrated encoder ", masterPitchingMotor.getPosition(), false);
    getTab().putInDashboard("absolute position", getAbsolutePosition(), false);

  }
}

