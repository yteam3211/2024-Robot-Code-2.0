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
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.armPosition;
import frc.robot.commands.zeroArm;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperTalonFX;

// Yteam Example Subsystem
public class GripperSubsystem extends SuperSystem {
  public SuperSparkMax gripperMotor;
  public Gains grippergGains;

  // Motors, Selenoid and Sensors declaration
  public GripperSubsystem() {
    super("ShootingSubsystem");
    grippergGains = new Gains("grippergGains", 1.5, 0, 0);

    gripperMotor = new SuperSparkMax(Constants.GRIPPER_MOTOR, MotorType.kBrushless, 30, false, 1, 1, IdleMode.kBrake,
        ControlType.kPosition, grippergGains, 0, 0, 0);
    // setDefaultCommand();
    this.resetGriperEncoder();
  }

  /** Creates a new ExampleSubsystem. */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  public void resetGriperEncoder(){
    gripperMotor.reset(0);
  }


  public void setGripperPosition(double position) {
    gripperMotor.setMode(ControlMode.Position);
    gripperMotor.getPIDController().setReference(position, ControlType.kPosition);
    SmartDashboard.putNumber("gripper target", position);


  }



  public void setGriperOutput(double output) {
    gripperMotor.setMode(ControlMode.PercentOutput);
    gripperMotor.set(output);
  }



  
  public void SetTeleopDefault(){
    // setDefaultCommand(null);
  }
}



