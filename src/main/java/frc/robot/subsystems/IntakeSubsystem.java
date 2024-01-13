// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperCanFlex;
import frc.util.motor.SuperSparkMax;

public class IntakeSubsystem extends SuperSystem {
  public SuperSparkMax intakeMotor;
  public SuperCanFlex intakeWheelMotor;
  public DigitalInput closeIntakeMicroSwitch;
  public Gains intakeGains;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    super("Intake Subsystem");
    intakeGains = new Gains("intakeGains", 0, 0, 0);
    intakeWheelMotor = new SuperCanFlex(Constants.INTAKE_WHEEL_MOTOR, MotorType.kBrushless, 40, false, 1, 1, IdleMode.kCoast, ControlType.kDutyCycle, null, 0, 0, 0);
    intakeMotor = new SuperSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless, 40, false, 1, 1, IdleMode.kBrake, ControlType.kPosition, intakeGains, 0, 0, 0);
    closeIntakeMicroSwitch = new DigitalInput(Constants.INTAKE_MICROW);
    getTab().addCommandToDashboard("Reset Intake pos", new InstantCommand(() -> this.resetEncoder()));
    }

  @Override
  public void periodic() 
  {
    if(this.isIntakeOn())
    {
      this.resetEncoder();
    }
    // This method will be called once per scheduler run
  }
  //intakeMotor functions
  public void resetEncoder(){
    intakeMotor.reset(0);
  } 

  public double getPosition(){
    return intakeMotor.getPosition();
  }

    public  void setIntakeMotorPos(double position) {
    intakeMotor.setMode(ControlMode.Position);
    intakeMotor.getPIDController().setReference(position, ControlType.kPosition);

  }

  //intakeWheelMotor functions

  public void resetWheelEncoder(){
    intakeWheelMotor.reset(0);
  } 

  public double getwheelPosition(){
    return intakeWheelMotor.getPosition();
  }

    public  void setwheelsMotorPos(double position) {
    intakeWheelMotor.setMode(ControlMode.Position);
    intakeWheelMotor.getPIDController().setReference(position, ControlType.kPosition);

  }

  public void setWheeksOutput(double Output) {
    intakeWheelMotor.setMode(ControlMode.PercentOutput);
    intakeWheelMotor.set(Output);
  }

    public boolean isIntakeOn()
    {
      return closeIntakeMicroSwitch.get();
    }

}
