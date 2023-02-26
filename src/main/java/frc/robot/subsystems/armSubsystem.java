// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ArmOutputCommand;
import frc.robot.commands.armPosition;
import frc.robot.commands.zeroArm;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperTalonFX;
import frc.robot.RobotButtons;


// Yteam Example Subsystem
public class armSubsystem extends SuperSystem {
  public static SuperSparkMax ArmgMotor;
  public Gains armgGains;

  // Motors, Selenoid and Sensors declaration
  public armSubsystem() {
    super("ShootingSubsystem");
    // armgGains = new Gains("armGains",0.04,0.0001,0.2); // human
    armgGains = new Gains("armGains", 0, 0, 0.000003, 0, 0.6, 0.04, 0); // second
    // armgGains = new Gains("armGains",0.22,0,0);
    ArmgMotor = new SuperSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless, 30, false, 1, 1, IdleMode.kBrake,
        ControlType.kSmartMotion, armgGains, 7, 10, 1);
    setDefaultCommand(new armPosition(this, -10));
    this.resetArmEncoder();
  }

  /** Creates a new ExampleSubsystem. */

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm position", ArmgMotor.getPosition());
    SmartDashboard.putNumber("arm velocity", ArmgMotor.getVelocity());

    // This method will be called once per scheduler run
  }

  public void resetArmEncoder() {
    ArmgMotor.reset(0);
  }


  public void setPosition(double position) {
    ArmgMotor.setMode(ControlMode.Position);
    ArmgMotor.getPIDController().setReference(position, ControlType.kSmartMotion);
    SmartDashboard.putNumber("armTarget", position);

  }


  public double getPosition() {
    return ArmgMotor.getPosition();
  }

  public void setArmOutput(double output) {
    ArmgMotor.setMode(ControlMode.PercentOutput);
    ArmgMotor.set(output);
  }



  public double velocity() {
    return ArmgMotor.getVelocity();
  }

  public void SetDisableDefault(){
    setDefaultCommand(new zeroArm(this));
  }
  
  public void SetTeleopDefault(){
    // setDefaultCommand(null);
  }
}


