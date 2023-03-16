// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwereCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotButtons;
import frc.robot.autos.AutoCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.collectWheels;

public class rightGRIDmovmentCommand extends CommandBase {
  /** Creates a new rightGRIDmovmentCommand. */
  private Swerve swerve;
  private int presses = 0;
  private String name;
  private boolean pressed = false;
  private Timer pressesTimer = new Timer();
  private Timer finishTimer = new Timer();
  public rightGRIDmovmentCommand(Swerve swerve) {
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
    if(!pressesTimer.hasElapsed(0.25) && RobotButtons.rightGRIDmovment.getAsBoolean() && !pressed){
      presses++;
      pressed = true;
    }
    else if(!RobotButtons.rightGRIDmovment.getAsBoolean() && pressed){
      pressesTimer.reset();
      pressed = false;
    }
  }

  // Called o nce the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotButtons.GRIDmovmentHelper = (() -> true);
    switch(presses){
    case 1:
      System.out.println("chosen: 1");
      name = "1 right GRID";
      break;
    case 2:
      System.out.println("chosen: 2");
      name = "2 right GRID";
      break;
    case 3:
      System.out.println("chosen: 3");
      name = "3 right GRID";
      break;
  }
  new GRIDmovmentGroupCommand(swerve, name).schedule();
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pressesTimer.hasElapsed(0.3);
  }
}
