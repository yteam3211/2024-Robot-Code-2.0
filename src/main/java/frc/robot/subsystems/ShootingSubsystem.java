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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  public Gains shooterGainsPos;

  public ShootingSubsystem() {
    super("shooting subsystem");
    shooterGains = new Gains("shooterGains", 0, 0, 2,0.00015,4,0,0);     
    shooterGainsPos = new Gains("shooterGains", 0.08, 0, 0); 


    masterShooterMotor = new SuperTalonFX(Constants.MASTER_SHOOTER_MOTOR_ID, Constants.CanBus.RIO, 40, true, false, NeutralMode.Brake, shooterGains, TalonFXControlMode.Velocity, 0, 0,0); 
    slaveShooterMotor = new SuperTalonFX(masterShooterMotor, Constants.SLAVE_SHOOTER_MOTOR_ID, Constants.CanBus.RIO, 40, true);

    masterShooterMotor.config_kP(1, shooterGainsPos.kp);
    masterShooterMotor.config_kI(1, shooterGainsPos.ki);
    masterShooterMotor.config_kD(1, shooterGainsPos.kd);
  }

  
  public void setShooterVelocity(double velocity)
  {
    System.out.println("set shooter velocity:" + velocity);
    masterShooterMotor.set(ControlMode.Velocity, velocity);
  }
  
  
  public double getVelocity()
  {
    return masterShooterMotor.getVelocity();
  }
    public double getPos()
  {
    return masterShooterMotor.getPosition();
  }

  public double getOutput(){
    return masterShooterMotor.getOutput();
  }
  
  public void setShooterOutput(double output)
  {
    System.out.println("set shooter output:" + output);
    masterShooterMotor.set(ControlMode.PercentOutput, output);
  }

    public void setShooterPos(double pos)
  {
    masterShooterMotor.selectProfileSlot(1, 0);
    masterShooterMotor.set(ControlMode.Position, pos);
  }

  public void setMode(NeutralMode mode){
      // masterShooterMotor.setNeutralMode(mode);          
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTab().putInDashboard("shooting velocity", masterShooterMotor.getVelocity(), false);   
    getTab().putInDashboard("shooting position", masterShooterMotor.getPosition(), false);
    SmartDashboard.putBoolean(getName(), ( Constants.SHOOTING_SPEAKER_VELCITY- masterShooterMotor.getVelocity()) < Constants.SHOOTING_VELOCITY_TRESHOLD);
  }
}
