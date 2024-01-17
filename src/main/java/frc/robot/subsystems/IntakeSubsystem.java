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
  public SuperSparkMax intakeOpenMotor;
  public SuperCanFlex intakeWheelsMotor;
  public DigitalInput closeIntakeMicroSwitch;
  public Gains intakeGains;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    super("Intake Subsystem");
    intakeGains = new Gains("intakeGains", 0, 0, 0);
    // intakeWheelsMotor = new SuperCanFlex(Constants.INTAKE_WHEELS_MOTOR, MotorType.kBrushless, 40, false, 1, 1, IdleMode.kCoast, ControlType.kDutyCycle, null, 0, 0, 0);
    // intakeOpenMotor = new SuperSparkMax(Constants.INTAKE_OPEN_MOTOR, MotorType.kBrushless, 40, false, 1, 1, IdleMode.kBrake, ControlType.kPosition, intakeGains, 0, 0, 0);
    // closeIntakeMicroSwitch = new DigitalInput(Constants.INTAKE_MICROW);
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
    intakeOpenMotor.reset(0);
  } 

  public double getPosition(){
    return intakeOpenMotor.getPosition();
  }

    public  void setIntakeMotorPos(double position) {
    intakeOpenMotor.setMode(ControlMode.Position);
    intakeOpenMotor.getPIDController().setReference(position, ControlType.kPosition);

  }

  //intakeWheelMotor functions

  public void resetWheelEncoder(){
    intakeWheelsMotor.reset(0);
  } 

  public double getwheelPosition(){
    return intakeWheelsMotor.getPosition();
  }

    public  void setwheelsMotorPos(double position) {
    intakeWheelsMotor.setMode(ControlMode.Position);
    intakeWheelsMotor.getPIDController().setReference(position, ControlType.kPosition);

  }

  public void setWheeksOutput(double Output) {
    intakeWheelsMotor.setMode(ControlMode.PercentOutput);
    intakeWheelsMotor.set(Output);
  }

    public boolean isIntakeOn()
    {
      return false;//closeIntakeMicroSwitch.get();
    }

}
