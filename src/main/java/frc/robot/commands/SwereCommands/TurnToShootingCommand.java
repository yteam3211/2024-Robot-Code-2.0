// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwereCommands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShootingMath;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;
import frc.util.vision.Limelight;

public class TurnToShootingCommand extends Command {
  private Swerve swerve;
  private Limelight limelight;
  private ShootingMath shootingMath;
  double output;
  protected Gains gains = new Gains("rotation gains", 0.03, 0, 0.4);

  protected PIDController pid = new PIDController(gains);

  /** Creates a new TurnToZeroCommand. */
  public TurnToShootingCommand(Swerve swerve, Limelight limelight, ShootingMath shootingMath) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    this.shootingMath = shootingMath;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setMaxOutput(Constants.SwerveConstant.maxSpeed * 0.6);
    System.out.println("******** inside TurnToShootingCommand");
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!limelight.isValid()){
      pid.setTargetPosition(ShootingMath.getEstematedSpeakerShootingAngle(swerve));
      output = pid.getOutput(Swerve.gyro.getYaw());
    }
    else{
      pid.setTargetPosition(0);
      output = pid.getOutput(limelight.getX());
    }
    output += 0.1 * Constants.SwerveConstant.maxAngularVelocity * Math.signum(output);
    swerve.drive(new Translation2d(0.046, 0.046), output, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0, true);
    System.out.println("********exit TurnToShootingCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.isValid() && Math.abs(limelight.getX()) < Constants.SHOOTING_ANGLE_TRESHOLD;
    };
  }

  

