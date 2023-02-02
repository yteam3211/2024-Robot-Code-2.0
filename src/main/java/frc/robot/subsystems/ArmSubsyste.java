// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.commands.ShootingOutput;
import frc.util.SuperSystem;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperVictorSP;


// Yteam Example Subsystem
public class ArmSubsyste extends SuperSystem {
   public SuperVictorSP armMotor;
   private DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(0);
  // Motors, Selenoid and Sensors declaration
  public ArmSubsyste() {
    super("ShootingSubsystem");
    armMotor = new SuperVictorSP(0,dutyCycleEncoder, 0);
    // setDefaultCommand(new ShootingLow(this, 0));
  }

  /** Creates a new ExampleSubsystem. */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOutput( double output){
    
  }

}

