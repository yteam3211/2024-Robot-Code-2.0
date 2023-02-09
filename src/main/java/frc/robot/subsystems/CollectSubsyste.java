// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.commands.ShootingOutput;
import frc.robot.commands.collectCommand;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperTalonSRX;
import frc.util.motor.SuperVictorSP;


// Yteam Example Subsystem
public class CollectSubsyste extends SuperSystem {
  //  public SuperVictorSP armMotor;
  //  private DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(0);
      public SuperTalonSRX leaderCollectMotor;
      public SuperTalonSRX collectMotor;
      public DigitalInput closeMicroSwitch;
      public Gains collectGains;
      
  // Motors, Selenoid and Sensors declaration
  public CollectSubsyste() {
    super("collectSubsystem");
    collectGains = new Gains("collectGains",0.03, 0,0);
    leaderCollectMotor = new SuperTalonSRX(Constants.RIGHT_LEADER_COLLECT_MOTOR, 30, false, false, 0, 1, 0, collectGains, ControlMode.Position);
    // closeMicroSwitch = new DigitalInput(Constants.CLOSE_MICROSWITCH);
    // setDefaultCommand(new collectCommand(this, 0));
  }

  /** Creates a new ExampleSubsystem. */
  
  @Override
  public void periodic() {
    System.out.println(leaderCollectMotor.getPosition());
    // This method will be called once per scheduler run
  }

  public void setPosition( double Position){
    leaderCollectMotor.set(ControlMode.Position, Position);
    // if (isCollectClose() == true) {
    //   resetEncoder();
    // };
  }
  
  // public boolean isCollectClose(){
    
  //   return closeMicroSwitch.get();
   
  // }

  public void reSetEncoder(){
    leaderCollectMotor.reset(0);
  }

  public void setOutput(double output){
 
    leaderCollectMotor.set(ControlMode.PercentOutput, output);

  
  }
}

