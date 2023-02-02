// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.ShootingLow;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperSparkMax;


// Yteam Example Subsystem
public class ShootingSubsystem extends SuperSystem {
 public SuperSparkMax ShooingMotor;
 public Gains shootingGains;
  // Motors, Selenoid and Sensors declaration
  public ShootingSubsystem() {
    super("ShootingSubsystem");
    shootingGains = new Gains("_shootingGains",0.01, 0,0);
    // ShooingMotor = new SuperSparkMax(14,false);
    ShooingMotor = new SuperSparkMax(14, MotorType.kBrushless, 30, false, 1 ,1 , IdleMode.kBrake, ControlType.kPosition, shootingGains, 0, 0, 0);
    setDefaultCommand(new ShootingLow(this, 0));
  }

  /** Creates a new ExampleSubsystem. */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOutput(double output){
    ShooingMotor.setMode(ControlMode.PercentOutput);
    ShooingMotor.set(output);
  }

  public void setPosition(double position){
    ShooingMotor.setMode(ControlMode.Position);
    ShooingMotor.set(position);
  }

}

