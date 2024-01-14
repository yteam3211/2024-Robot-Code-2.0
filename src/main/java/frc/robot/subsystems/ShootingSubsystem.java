// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperCanFlex;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperTalonFX;

public class ShootingSubsystem extends SuperSystem {
  /** Creates a new ShootingSubsystem. */
  public SuperSparkMax kickerMotor;
  public SuperTalonFX masterShooterMotor;
  public SuperTalonFX slaveShooterMotor;
  public Gains shooterGains;
  public ShootingSubsystem() {
    super("shooting");
    shooterGains = new Gains("shooterGains", 0, 0, 0);
    kickerMotor = new SuperSparkMax(0, MotorType.kBrushless, 40, false, 1, 1, IdleMode.kCoast, ControlType.kDutyCycle, null, 0, 0, 0);
    masterShooterMotor = new SuperTalonFX(0, 40, false, false, NeutralMode.Coast, shooterGains, null); //queen dont forget control mode
    slaveShooterMotor = new SuperTalonFX(masterShooterMotor, 0, 40, false);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterVelocity(double velocity)
  {
     masterShooterMotor.set(ControlMode.Velocity, velocity);
  }

  public void setKickerOutput(double Output)
  {
    kickerMotor.setMode(ControlMode.PercentOutput);
    kickerMotor.set(Output);
  }

  public double getVelocity()
  {
    return masterShooterMotor.getVelocity();
  }

}
