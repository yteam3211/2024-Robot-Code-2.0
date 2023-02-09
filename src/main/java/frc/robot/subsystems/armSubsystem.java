// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperTalonFX;


// Yteam Example Subsystem
public class armSubsystem extends SuperSystem {
 public SuperSparkMax ArmgMotor;
 public Gains ArmgGains;

  // Motors, Selenoid and Sensors declaration
  public armSubsystem() {
    super("ShootingSubsystem");
    ArmgGains = new Gains("_shootingGains",0, 0,0,0,0,0,0);
    ArmgMotor = new SuperSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless, 30, false, 1 ,1 , IdleMode.kBrake, ControlType.kPosition, ArmgGains, 0, 0, 0);
    // setDefaultCommand(new ShootingOutput(this, 0));
  }

  /** Creates a new ExampleSubsystem. */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoder(){
    ArmgMotor.reset(0);
  }

  public void setPosition(double position){

    // ShooingMotor.setMode(ControlMode.Position);
    // ShooingMotor.getPIDController().setReference(position, ControlType.kPosition);
    // getTab().putInDashboard("encoder_live", ShooingMotor.getPosition(), false);

  }

}

