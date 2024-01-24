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

public class EleavatorSubsystem extends SuperSystem {
  private SuperTalonFX masterEleavatorMotor;
  private SuperTalonFX slave1rEleavatorMotor;
  private SuperTalonFX slave2rEleavatorMotor;
  private Gains eleavatorGains;
  private DigitalInput EleavatorMicrowSwitch;
  /** Creates a new ElevatorSubsystem. */
  public EleavatorSubsystem() {
    super("ElevatorSubsystem");
    eleavatorGains = new Gains("eleavatorGains", 0, 0, 0);
    masterEleavatorMotor = new SuperTalonFX(Constants.MASTER_ELEAVATOR_MOTOR, 40, false, false, NeutralMode.Brake, eleavatorGains, TalonFXControlMode.Position);
    slave1rEleavatorMotor = new SuperTalonFX(masterEleavatorMotor, Constants.SLAVE1_ELEAVATOR_MOTOR, 40, false);
    slave2rEleavatorMotor = new SuperTalonFX(masterEleavatorMotor, Constants.SLAVE2_ELEAVATOR_MOTOR, 40, false);
    EleavatorMicrowSwitch = new DigitalInput(Constants.MICROWSWITCH_ELEAVATOR);

  }

  @Override
  public void periodic() {
    if (this.isEleavatorDown()) 
    {

    }
    // This method will be called once per scheduler run
  }

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
  }

  public void setPosition(double position)
  {
    masterEleavatorMotor.set(ControlMode.Position, position);
  }
}
