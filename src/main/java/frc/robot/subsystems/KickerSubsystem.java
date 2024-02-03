// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ShootingCommands.KickerCommand;
import frc.util.motor.SuperSparkMax;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */
    public SuperSparkMax kickerMotor;

  public KickerSubsystem() {
   kickerMotor = new SuperSparkMax(Constants.KICKER_SHOOTER_MOTOR_ID, MotorType.kBrushless, 40, false, IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // test( 1 > 2 ? 3 : 2, 2);
    // This method will be called once per scheduler run
  }

    public void setKickerOutput(double Output)
  {
    kickerMotor.setMode(ControlMode.PercentOutput);
    kickerMotor.set(Output);
  }
  public void test(double a, double b){}
  }

