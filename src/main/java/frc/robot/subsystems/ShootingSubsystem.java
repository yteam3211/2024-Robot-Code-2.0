// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.commands.ShootingLow;
import frc.util.SuperSystem;
import frc.util.motor.SuperSparkMax;


// Yteam Example Subsystem
public class ShootingSubsystem extends SuperSystem {
 public SuperSparkMax ShooingMotor;
  // Motors, Selenoid and Sensors declaration
  public ShootingSubsystem() {
    super("ShootingSubsystem");
    ShooingMotor = new SuperSparkMax(0, null, 0, false, 0, 0, null, null, null, 0, 0, 0);
    setDefaultCommand(new ShootingLow(this, 0));
  }

  /** Creates a new ExampleSubsystem. */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOutput( double output){
    ShooingMotor.set( output);
  }

}

