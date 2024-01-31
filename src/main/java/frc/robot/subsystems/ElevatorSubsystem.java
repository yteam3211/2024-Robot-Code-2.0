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
  private SuperTalonFX masterEleavatorMotor;
  private SuperTalonFX slave1rEleavatorMotor;
  private SuperTalonFX slave2rEleavatorMotor;
  private Gains eleavatorGains;
  private DigitalInput EleavatorMicrowSwitch;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    super("ElevatorSubsystem");
    eleavatorGains = new Gains("eleavatorGains", 0, 0, 0);
    masterEleavatorMotor = new SuperTalonFX(Constants.MASTER_ELEAVATOR_MOTOR_ID, 40, false, false, NeutralMode.Brake, eleavatorGains, TalonFXControlMode.Position);
    slave1rEleavatorMotor = new SuperTalonFX(masterEleavatorMotor, Constants.SLAVE1_ELEAVATOR_MOTOR_ID, 40, false);
    slave2rEleavatorMotor = new SuperTalonFX(masterEleavatorMotor, Constants.SLAVE2_ELEAVATOR_MOTOR_ID, 40, false);
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
    return EleavatorMicrowSwitch.get();
  }
  
  public void resetEncoder()
  {
    masterEleavatorMotor.reset(0);
    masterEleavatorMotor.set(ControlMode.PercentOutput, 0);
  }
  
  public void setPosition(double position)
  {
    masterEleavatorMotor.set(ControlMode.Position, position);
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
    if (this.isEleavatorDown()) 
    {
      resetEncoder();
    }

    if((getMasterPosition() < Constants.MIN_ELEAVATOR_POS) ||(getMasterPosition() < Constants.MAX_ELEAVATOR_POS))
    {
      masterEleavatorMotor.set(ControlMode.PercentOutput, 0);
    }
    // This method will be called once per scheduler run
  }
}
