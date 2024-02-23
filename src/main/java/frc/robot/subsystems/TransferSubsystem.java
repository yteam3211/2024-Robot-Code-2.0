// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.util.SuperSystem;
import frc.util.motor.SuperSparkMax;

public class TransferSubsystem extends SuperSystem {
  private SuperSparkMax transferMotor;
  private DigitalInput noteIn; 
  /** Creates a new TransferSubsystem. */
  public TransferSubsystem() {
    super("Transfer Subsystem");
    transferMotor = new SuperSparkMax(Constants.TRANSFER_MOTOR_ID, MotorType.kBrushless, 30, false, IdleMode.kCoast);
    noteIn = new DigitalInput(3);
  }

  public void setOutput(double output){
    transferMotor.setMode(ControlMode.PercentOutput);
    transferMotor.set(output);
  }

  public double getOutput(){
    return transferMotor.getOutput();
  }

  public boolean isNoteIn(){
    return noteIn.get();
  }
  @Override
  public void periodic() {
    getTab().putInDashboard("motor output", getOutput(), false);
    getTab().putInDashboard("is note in", isNoteIn(), false);
    // This method will be called once per scheduler run
  }
}
