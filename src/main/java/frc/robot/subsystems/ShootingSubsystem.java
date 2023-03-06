// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.dashboard.SuperSystem;
import frc.util.motor.SuperTalonSRX;

public class shootingSubsystem extends SuperSystem {
 private SuperTalonSRX shootingWheels;
 private SuperTalonSRX shootingWheelsMaster;
  /** Creates a new shootingSubsystem. */
  public shootingSubsystem() {
    super("shootingSubsystem");
    shootingWheelsMaster = new SuperTalonSRX(21, 40, false, false, 0, 1, 1, null, ControlMode.Velocity);
    shootingWheels = new SuperTalonSRX(shootingWheelsMaster, 22, 40, true);
    // setDefaultCommand();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setVelocity(double Velocity){
    shootingWheelsMaster.set(ControlMode.Velocity, Velocity);
  }

  public double GetVelocity(){
    return shootingWheelsMaster.getVelocity();
    }

  public void resetEncoder(){
    shootingWheelsMaster.reset(0);
  }


}
