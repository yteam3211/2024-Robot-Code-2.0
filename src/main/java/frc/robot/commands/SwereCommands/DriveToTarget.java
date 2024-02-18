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

public class DriveToTarget extends Command {
  private Swerve swerve;
  private Limelight limelight;
  private ShootingMath shootingMath;
  protected double desiredAngle;
  protected double desiredXpos;
  protected double desiredYpos;
  protected double ROutput;
  protected double XOutput;
  protected double YOutput;
  protected Gains RGains = new Gains("rotation gains", 0, 0, 0);
  protected Gains XGains = new Gains("rotation gains", 0, 0, 0);
  protected Gains YGains = new Gains("rotation gains", 0, 0, 0);

  protected PIDController Rpid = new PIDController(RGains);
  protected PIDController Xpid = new PIDController(XGains);
  protected PIDController Ypid = new PIDController(YGains);

  /** Creates a new TurnToZeroCommand. */
  public DriveToTarget(Swerve swerve, Limelight limelight, ShootingMath shootingMath, double desiredAngle, double desiredXpos, double desiredYpos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    this.shootingMath = shootingMath;
    this.desiredAngle = desiredAngle;
    this.desiredXpos = desiredXpos;
    this.desiredYpos = desiredYpos;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rpid.setMaxOutput(Constants.SwerveConstant.maxSpeed * 0.6);
    Xpid.setMaxOutput(Constants.SwerveConstant.maxSpeed * 0.6);
    Ypid.setMaxOutput(Constants.SwerveConstant.maxSpeed * 0.6);
    Rpid.setTargetPosition(desiredAngle);
    Xpid.setTargetPosition(desiredXpos);
    Ypid.setTargetPosition(desiredYpos);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      ROutput = Rpid.getOutput(Swerve.gyro.getYaw());
      XOutput = Xpid.getOutput(swerve.getPose().getX());
      YOutput = Ypid.getOutput(swerve.getPose().getY());
    
    ROutput += 0.1 * Constants.SwerveConstant.maxAngularVelocity * Math.signum(ROutput);
    XOutput += 0.1 * Constants.SwerveConstant.maxAngularVelocity * Math.signum(XOutput);
    YOutput += 0.1 * Constants.SwerveConstant.maxAngularVelocity * Math.signum(YOutput);
    swerve.drive(new Translation2d(XOutput, YOutput), ROutput, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.isValid() && Math.abs(limelight.getX()) < Constants.SHOOTING_ANGLE_TRESHOLD;
    };
  }

  

