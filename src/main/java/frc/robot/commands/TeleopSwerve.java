package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import frc.robot.subsystems.Swerve;
import frc.util.PID.Gains;
import frc.util.PID.PIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    protected double seconds;
    private Timer timer = new Timer();    
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, double seconds) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.seconds = seconds;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if(!timer.hasElapsed(seconds) & seconds != 0){
            translationVal = 0;
            strafeVal = 0.2;
            rotationVal = 0;
        }
        /* Drive */
        if (RobotButtons.Balance.getAsBoolean() && Math.abs(s_Swerve.gyro.getPitch()) > 2){
            s_Swerve.drive(     
                new Translation2d(0.2 * Math.signum(s_Swerve.gyro.getPitch()), 0).times(Constants.Swerve.maxSpeed), 
                0,
             !robotCentricSup.getAsBoolean(), //Field oriented by the controller switch
             true
            );
        }
        else{
            s_Swerve.drive(            
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), //Field oriented by the controller switch
             true
            );
        }
       }
    }

