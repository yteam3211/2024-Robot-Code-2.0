// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShootingCommands.KickerCommand;
import frc.robot.commands.ShootingCommands.ShootingOutput;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperCanFlex;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperTalonFX;

public class ShootingSubsystem extends SuperSystem {
  /** Creates a new ShootingSubsystem. */
  public SuperTalonFX masterShooterMotor;
  public SuperTalonFX slaveShooterMotor;
  public Gains shooterGains;
  public ShootingSubsystem() {
    super("shooting");
    shooterGains = new Gains("shooterGains", 0, 0, 5,0,0.5,0.5,0); //TODO: need to add kf
    masterShooterMotor = new SuperTalonFX(Constants.MASTER_SHOOTER_MOTOR_ID, 40, false, false, NeutralMode.Coast, shooterGains, TalonFXControlMode.Velocity); 
    slaveShooterMotor = new SuperTalonFX(masterShooterMotor, Constants.SLAVE_SHOOTER_MOTOR_ID, 40, false);
    setDefaultCommand( new ShootingOutput(this, 0)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTab().putInDashboard("shooting velocity", masterShooterMotor.getVelocity(), false);
  }

  public void setShooterVelocity(double velocity)
  {
     masterShooterMotor.set(ControlMode.Velocity, velocity);
  }



  public double getVelocity()
  {
    return masterShooterMotor.getVelocity();
  }

   public void setShooterOutput(double output)
  {
     masterShooterMotor.set(ControlMode.PercentOutput, output);
  }

}
