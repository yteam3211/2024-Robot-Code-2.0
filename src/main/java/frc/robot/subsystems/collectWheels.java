// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class collectWheels extends SubsystemBase {
  public VictorSP collectWheelsMotor ;
  public VictorSP centeringMotor;
  /** Creates a new collectWheels. */
  public collectWheels() {
    collectWheelsMotor = new VictorSP(1);
    centeringMotor = new VictorSP(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void WheelsSetOutput(double output){
    // leaderCollectMotor.set(ControlMode.PercentOutput, output);
    collectWheelsMotor.set(output);
  }

  public void centeringSetOutput(double Output){
    centeringMotor.set(Output);
  }
}
