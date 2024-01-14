// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.util.PID.Gains;
import frc.util.motor.SuperTalonFX;

public class PitchingSubsystem extends SubsystemBase {
  /** Creates a new PitchingSubsystem. */
  public SuperTalonFX masterPitchingMotor;
  public SuperTalonFX slavePitchingMotor;
  public CANCoder angleEncoder;
  public Gains pitchingGains;

  public PitchingSubsystem() {
    pitchingGains = new Gains("pitchingGains", 0, 0, 0);
    masterPitchingMotor = new SuperTalonFX(Constants.MASTER_PITCHING_MOTOR, 40, false, false, NeutralMode.Coast, pitchingGains, null); //queen dont forget control mode
    slavePitchingMotor = new SuperTalonFX(masterPitchingMotor, Constants.SLAVE_PITCHING_MOTOR, 40, false);
    angleEncoder = new CANCoder(Constants.PITCHING_ENCODER);
  }

  public void setPosition(double position){
    masterPitchingMotor.set(ControlMode.Position, getAbsolutePosition(position));
  }

  public double getAbsolutePosition(double position){
    return Conversions.degreesToFalcon(position - Constants.PITCHING_ENCODER_OFFSET,1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
