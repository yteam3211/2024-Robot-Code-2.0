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
  public SuperSparkMax intakeWheelsMotor;
  public DigitalInput closeIntakeMicroSwitch;
  public Gains intakeOpenGains;
  public Gains intakeWheelsGains;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    super("Intake Subsystem");
    intakeOpenGains = new Gains("intake Open Gains", 0.035, 0, 0.001);
    intakeWheelsGains = new Gains("intake Wheels Gains", 0.001, 0, 0);
    intakeWheelsMotor = new SuperSparkMax(Constants.INTAKE_WHEELS_MOTOR_ID, MotorType.kBrushless, 30, false, IdleMode.kCoast);
    intakeOpenMotor = new SuperSparkMax(Constants.INTAKE_OPEN_MOTOR_ID, MotorType.kBrushless, 40, false, 1, 1, IdleMode.kCoast, ControlType.kPosition, intakeOpenGains, 0, 0, 0);
    closeIntakeMicroSwitch = new DigitalInput(Constants.INTAKE_MICROSWITCH_ID);
    getTab().addCommandToDashboard("Reset Intake pos", new InstantCommand(() -> resetEncoder()));
    }
    //intakeMotor functions
    
    /**
     * reset the integrated encoder to 0.
     */
    public void resetEncoder(){
        setIntakeOpenMotorPosition(0);
      intakeOpenMotor.reset(0);
    } 
    
    /**
     * get the positon of the integrated encoder.
     * @return the number of rotations of the motor
   */
  public double getPosition(){
    return intakeOpenMotor.getPosition();
  }
  
  /**
   * set the intake open motor position
   * @param position the number of rotation to set the motor
   */
  public  void setIntakeOpenMotorPosition(double position) {
    intakeOpenMotor.setMode(ControlMode.Position);
    intakeOpenMotor.getPIDController().setReference(position, ControlType.kPosition);
    
  }
    public  void setIntakeOpenMotorOutput(double output) {
    intakeOpenMotor.setMode(ControlMode.PercentOutput);
    intakeOpenMotor.set(output);
    
  }
  
  //intakeWheelMotor functions
  
  /**
   * set the intake wheels motor output
   * @param Output the output to set the motor [-1, 1]
   */
  public void setWheelsMotorOutput(double Output) {
    intakeWheelsMotor.setMode(ControlMode.PercentOutput);
    intakeWheelsMotor.set(Output);
  }

  public void setWheelsMotorVelocity(double velocity){
    intakeWheelsMotor.setMode(ControlMode.Velocity);
    intakeWheelsMotor.set(velocity);
  }
  
  /**
   * check if the microswitch is preesed or not
   * @return true if pressed, false if not pressed
   */
  public boolean isIntakeClose()
  {
    return !closeIntakeMicroSwitch.get();
  }
  
    @Override
    // This method will be called once per scheduler run
    public void periodic() 
    {
      getTab().putInDashboard("intake pos", intakeOpenMotor.getPosition(), 3, 1, false);
      getTab().putInDashboard("intake velocity", intakeWheelsMotor.getVelocity(), 4, 1, false);      
      getTab().putInDashboard("intake micro",isIntakeClose(), 5, 1, false);      
      getTab().putInDashboard("intake volt",intakeOpenMotor.getBusVoltage(), 6, 1, false);
      getTab().putInDashboard("intake amp",intakeOpenMotor.getAmper(), 7, 1, false);



      // System.out.println(intakeOpenMotor.getPosition());
      if(this.isIntakeClose())
      {
        resetEncoder();
      }
    }
}
