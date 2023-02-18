// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      public DigitalInput closeMicroSwitch;
      public Gains collectGains;
      public VictorSP collectMotor ;
      public VictorSP motor;
      int counter = 0;
      double point = 0;
      
  // Motors, Selenoid and Sensors declaration
  /**
   * 
   */
  public CollectSubsyste() {
    super("collectSubsystem");
    collectGains = new Gains("collectGains",4, 0,0);
    leaderCollectMotor = new SuperTalonSRX(Constants.RIGHT_LEADER_COLLECT_MOTOR, 30, false, false, 0, 1, 0, collectGains, ControlMode.Position);
    collectMotor = new VictorSP(1);
    motor = new VictorSP(0);
    this.reSetEncoder();
    // closeMicroSwitch = new DigitalInput(Constants.CLOSE_MICROSWITCH);
    // setDefaultCommand(new collectCommand(this));
  }

  /** Creates a new ExampleSubsystem. */
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("position", leaderCollectMotor.getPosition());
    SmartDashboard.putNumber("output", leaderCollectMotor.getOutput());

    // System.out.println("p" + getPosition());
  }

  public void setPosition(double position){
    // System.out.println("ppp" + this.point);
    SmartDashboard.putNumber("target", position);
    leaderCollectMotor.set(ControlMode.Position, position);

    // if (isCollectClose() == true) {
    //   resetEncoder();
    // };
  }

  public void setPoint(double point){
    this.point = point;
  }
  
  // public boolean isCollectClose(){
    
  //   return closeMicroSwitch.get();
   
  // }

  public void reSetEncoder(){
    leaderCollectMotor.reset(0);
  }

  public void setOutput(double output){

 
    // leaderCollectMotor.set(ControlMode.PercentOutput, output);
    collectMotor.set(output);
  }

  public void output(double Output){
    motor.set(Output);
  }
  public double getPosition(){
    return leaderCollectMotor.getPosition();
  }
}

