package frc.robot.commands.timercommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;
import frc.util.vision.Limelight;

public class TimerSideToSide extends CommandBase {
  /** Creates a new SideToSideApriltag. */
  protected Limelight limelight;
  protected Swerve swerve;
  protected boolean AprilTag;
  protected Gains gainsX = new Gains("gains x", 0.03, 0, 0);
  protected Gains gainsY = new Gains("gains y", 0.45, 0, 0.045);
  protected Gains gainsR = new Gains("gains r", 0.05, 0, 0);
  protected PIDController pidX = new PIDController(gainsX);
  protected PIDController pidY = new PIDController(gainsY);
  protected PIDController pidR = new PIDController(gainsR);
  private double seconds;
  private Timer timer = new Timer();

  public TimerSideToSide(Limelight limelight, Swerve swerve, boolean AprilTag, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    this.AprilTag = AprilTag;
    this.seconds = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if (AprilTag){
      pidY.setTargetPosition(0);
      limelight.setPipeline(0);
    }
    else{
      pidY.setTargetPosition(5);
      limelight.setPipeline(1);
    }
    pidX.setTargetPosition(0);
    pidR.setTargetPosition(0);
    pidX.setMaxOutput(Constants.SwerveConst.maxSpeed * 0.6);
    pidY.setMaxOutput(Constants.SwerveConst.maxSpeed * 0.6);
    pidR.setMaxOutput(Constants.SwerveConst.maxAngularVelocity * 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOutput = pidX.getOutput(limelight.getX());
    double yOutput = pidY.getOutput(limelight.getY());
    double rOutput = pidR.getOutput(Swerve.gyro.getYaw());

    xOutput += 0.02 * Constants.SwerveConst.maxSpeed * Math.signum(xOutput);
    yOutput += 0.02 * Constants.SwerveConst.maxSpeed * Math.signum(yOutput);
    rOutput += 0.02 * Constants.SwerveConst.maxAngularVelocity * Math.signum(rOutput);

    swerve.drive(new Translation2d(yOutput, xOutput), rOutput, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(seconds); 
  }
}
