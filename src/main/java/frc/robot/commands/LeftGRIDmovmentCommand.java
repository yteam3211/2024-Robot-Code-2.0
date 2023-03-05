// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotButtons;
import frc.robot.autos.GRIDmovement.Left1GRID;
import frc.robot.autos.GRIDmovement.Left2GRID;
import frc.robot.autos.GRIDmovement.Left3GRID;
import frc.robot.autos.GRIDmovement.Right1GRID;
import frc.robot.autos.GRIDmovement.Right2GRID;
import frc.robot.autos.GRIDmovement.Right3GRID;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.collectWheels;

public class LeftGRIDmovmentCommand extends CommandBase {
  /** Creates a new rightGRIDmovmentCommand. */
  private Swerve swerve;
  private int presses = 0;
  private boolean pressed = false;
  private Timer pressesTimer = new Timer();
  private Timer finishTimer = new Timer();
  public LeftGRIDmovmentCommand(Swerve swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotButtons.GRIDmovmentHelper = (() -> false);
    pressed = false;
    presses = 0;
    pressesTimer.reset();
    pressesTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("presses: " + presses + "time: " + pressesTimer.get());
    if(!pressesTimer.hasElapsed(0.2) && RobotButtons.leftGRIDmovment.getAsBoolean() && !pressed){
      presses++;
      pressed = true;
    }
    else if(!RobotButtons.leftGRIDmovment.getAsBoolean() && pressed){
      pressesTimer.reset();
      pressed = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotButtons.GRIDmovmentHelper = (() -> true);
    switch(presses){
    case 1:
      System.out.println("chosen: 1");
      Left1GRID.getAutoCommand(swerve).schedule();
      break;
    case 2:
      System.out.println("chosen: 2");
      Left2GRID.getAutoCommand(swerve).schedule();
      break;
    case 3:
      System.out.println("chosen: 3");
      Left3GRID.getAutoCommand(swerve).schedule();
      break;
  }
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pressesTimer.hasElapsed(0.25);
  }
}
