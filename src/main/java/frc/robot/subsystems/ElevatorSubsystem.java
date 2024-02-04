// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperTalonFX;

public class ElevatorSubsystem extends SuperSystem {
  public enum gains{
      EleavatorUp(0),
      EleavatorDown(1),
      EleavatorClimbUp(2),
      EleavatorClimbDown(4);
      public final int value;
      gains(int index) {
        value = index;
     }
  }
  private SuperTalonFX masterEleavatorMotor;
  private SuperTalonFX slave1EleavatorMotor;
  private SuperTalonFX slave2EleavatorMotor;
  private Gains eleavatorUpGains;
  private Gains eleavatorDownGains;    
  private Gains eleavatorClimbUpGains;  
  private Gains eleavatorClimbDownGains;
  public gains mode;

  private DigitalInput EleavatorMicrowSwitch;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    super("ElevatorSubsystem");
    eleavatorUpGains = new Gains("eleavator up Gains", 0.03, 0, 0);    
    eleavatorDownGains = new Gains("eleavator up Gains", 0.03, 0, 0);
    eleavatorClimbUpGains = new Gains("eleavator up Gains", 0.03, 0, 0);
    eleavatorClimbDownGains = new Gains("eleavator up Gains", 0.03, 0, 0);
    masterEleavatorMotor = new SuperTalonFX(Constants.MASTER_ELEAVATOR_MOTOR_ID, 40, false, false, NeutralMode.Brake, eleavatorUpGains, TalonFXControlMode.MotionMagic, 0, 0,0);
    slave1EleavatorMotor = new SuperTalonFX(masterEleavatorMotor, Constants.SLAVE1_ELEAVATOR_MOTOR_ID, 40, false);
    slave2EleavatorMotor = new SuperTalonFX(masterEleavatorMotor, Constants.SLAVE2_ELEAVATOR_MOTOR_ID, 40, false);
    EleavatorMicrowSwitch = new DigitalInput(Constants.MICROSWITCH_ELEAVATOR_ID);

    
  }
  
  /**
   * get the master motor of the eleavator
   * @return the SuperTalonFX master motor
   */
  public double getMasterPosition()
  {
    return masterEleavatorMotor.getPosition();
  }
  
  public boolean isEleavatorDown()
  {
    return !EleavatorMicrowSwitch.get();
  }
  
  public void resetEncoder()
  {
    masterEleavatorMotor.reset(0);
    masterEleavatorMotor.set(ControlMode.PercentOutput, 0);
  }
  
  /**
   * set the hight that the elevator should go
   * @param hight hight im milimeters
   */
  public void setPosition(double hight)
  {
    double falconPos = (hight / Constants.ELEAVATOR_WINCH_CIRCUMFERENCE) * Constants.ELEAVATOR_GEAR_RATIO * 2048;
    masterEleavatorMotor.set(ControlMode.Position, falconPos);
  }

  public void setOutput(double Output){
    masterEleavatorMotor.set(ControlMode.PercentOutput, Output);
  }
  
  /**
   * get the hight that the elevator got up. 
   * @return the hight from the base of the elevator to the bottom of the elevator's rider in milimeters
   */
  public double getElevatorHight(){
    return ((getMasterPosition() / 2048) / Constants.ELEAVATOR_GEAR_RATIO) * Constants.ELEAVATOR_WINCH_CIRCUMFERENCE;
  }

  @Override
  public void periodic() {
    getTab().putInDashboard("elevator hight", getElevatorHight(), false);
    getTab().putInDashboard("elevator master integrated encoder", masterEleavatorMotor.getPosition(), false);
    getTab().putInDashboard("is elevator down", isEleavatorDown(), false);
    if (this.isEleavatorDown())
    {
      resetEncoder();
    }
    
    if((getMasterPosition() < Constants.MIN_ELEAVATOR_POS) ||(getMasterPosition() > Constants.MAX_ELEAVATOR_POS))
    {
      masterEleavatorMotor.set(ControlMode.PercentOutput, 0);
    }
    // This method will be called once per scheduler run
  }
}
