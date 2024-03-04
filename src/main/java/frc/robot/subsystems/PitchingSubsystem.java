// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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
  public double hightLimelightToApriltag;
  public double distanceFromLimelightToSpeaker;
  public double hightShootingToSpeaker;
  public double distanceFromShooterToSpeaker;
  public double angleToSpeakerRadians;
  public double angleToSpeakerDegrees;
  // public CANCoder angleEncoder;
  

  public PitchingSubsystem() {
    super("Pitching Subsystem");
    pitchingGains = new Gains("pitchingGains", 0.75, 0, 0.008);
    masterPitchingMotor = new SuperTalonFX(Constants.MASTER_PITCHING_MOTOR_ID, Constants.CanBus.RIO, 40, true, false, NeutralMode.Brake, pitchingGains, TalonFXControlMode.MotionMagic,10000, 11000,0); 
    slavePitchingMotor = new SuperTalonFX(masterPitchingMotor, Constants.SLAVE_PITCHING_MOTOR_ID, Constants.CanBus.RIO, 40, true);
    angleEncoder = new CANcoder(Constants.PITCHING_ENCODER_ID);
    configAngleEncoder();
    resetFalconEncoder();
    getTab().addCommandToDashboard("reset falcon encoder", new InstantCommand( () -> resetFalconEncoder()));
  }

  public void SetOutput(double output){ 
    masterPitchingMotor.set(ControlMode.PercentOutput, output);
  }
  /**
   * Set the position of the motor via magic motion control.
   *
   * @param position position in degrees.
   */
  public void setPosition(double position){
    masterPitchingMotor.set(ControlMode.MotionMagic, degreesToFalconEncoder(position));
    // masterPitchingMotor.set(ControlMode.MotionMagic, position);
  }
/**
 * config the angle encoder
 */
  private void configAngleEncoder(){        
// angleEncoder.getPosition().setUpdateFrequency(4);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.pitchingCanCoderConfig);
    // angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    // angleEncoder.configSensorDirection(true);
    // angleEncoder.configMagnetOffset(Constants.PITCHING_ENCODER_OFFSET);
    // masterPitchingMotor.configRemoteFeedbackFilter(angleEncoder, 1);
    // masterPitchingMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1);
  }

  /**
   * Return the current angle - the angle offset
   *
   * @return the absolute angle of the Cancoder in degrees.
   */
  public double getAbsolutePosition(){
    double absolutePosition;
    if(Constants.PITCHING_ENCODER_OFFSET < 85 && Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) > 200){
      absolutePosition = Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) - (Constants.PITCHING_ENCODER_OFFSET + 360);
    }
    else if(Constants.PITCHING_ENCODER_OFFSET > 300 && Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) < 80  ){
      absolutePosition = Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) - Constants.PITCHING_ENCODER_OFFSET + 360;
    }
    else{
      absolutePosition = Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()) - Constants.PITCHING_ENCODER_OFFSET;
    }
    return absolutePosition;
    // return angleEncoder.getAbsolutePosition();
  }

  /**
 * reset the falcon integrated encoder
 */
  public void resetFalconEncoder(){
    masterPitchingMotor.reset(degreesToFalconEncoder(getAbsolutePosition()));
    System.out.println("reset: " + degreesToFalconEncoder(getAbsolutePosition())); 
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
    // return degrees * (2048 / 180);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // getAngleToSpeaker(elevatorSubsystem, Robot.m_robotContainer.getLimelight());
    if((getAbsolutePosition() > Constants.MAX_PITCHING_ANGLE)) //TODO: needs to be changed to the correct values in constants
    {
      masterPitchingMotor.set (ControlMode.PercentOutput, 0);
    }
    getTab().putInDashboard("CANcoder ", Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()), false);
    getTab().putInDashboard("integrated encoder ", masterPitchingMotor.getPosition(), false);
    getTab().putInDashboard("absolute position", getAbsolutePosition(), false);
  }
}

