// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.chrono.ThaiBuddhistChronology;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.armPosition;
import frc.robot.commands.zeroArm;
import frc.util.dashboard.SuperSubSystemTab;
import frc.util.SuperSystem;
import frc.util.PID.Gains;
import frc.util.motor.SuperSparkMax;
import frc.util.motor.SuperTalonFX;

// Yteam Example Subsystem
public class armCollectSubsystem extends SuperSystem {
  public SuperSparkMax armCollectMotor;
  public CollectSubsystem collectSubsystem;
  public Gains armCollectgGains;
  public DigitalInput downMicroSwitch;

  private SuperSubSystemTab shuffleBoardTab = new SuperSubSystemTab("armCollect", this);



  // Motors, Selenoid and Sensors declaration
  public armCollectSubsystem() {
    super("armCollect");
    armCollectgGains = new Gains("grippergGains", 0.1, 0, 0);

     armCollectMotor = new SuperSparkMax(Constants.ARM_COLLECT_MOTOR, MotorType.kBrushless, 30, false, 1, 1, IdleMode.kBrake,
        ControlType.kPosition, armCollectgGains, 0, 0, 0);
        shuffleBoardTab.addCommandToDashboard("Reset", new InstantCommand(() -> this.resetArmCollectEncoder()));
    // setDefaultCommand();
    this.resetArmCollectEncoder();
  }

  /** Creates a new ExampleSubsystem. */

  @Override
  public void periodic() {
    shuffleBoardTab.putInDashboard("collect position", armCollectMotor.getPosition(), false);
    shuffleBoardTab.putInDashboard("collect output", armCollectMotor.getOutput(), false);
    // System.out.println("collect position" + armCollectMotor.getPosition());
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Gripper Position", gripperMotor.getPosition());
  }



  public void resetArmCollectEncoder(){
    armCollectMotor.reset(0);
  }


  public  void setArmCollectPosition(double position) {
    armCollectMotor.setMode(ControlMode.Position);
    armCollectMotor.getPIDController().setReference(position, ControlType.kPosition);
    // SmartDashboard.putNumber("gripper target", position);


  }



  public void setArmCollectOutput(double output) {
    armCollectMotor.setMode(ControlMode.PercentOutput);
    armCollectMotor.set(output);
  //   if (((output <= 0 && !isShootingDown()))){
  //     armCollectMotor.set(output);
  //   }
  }


  public boolean isShootingDown(){
    
    // return downMicroSwitch.get();
    return false;
  }
}
