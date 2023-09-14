package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.dashboard.SuperSystem;
import frc.util.PID.Gains;
import frc.robot.Constants;
import frc.robot.RobotButtons;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Swerve extends SuperSystem {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public static final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public Swerve() {
        super("Swerve");
        zeroGyro();
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstant.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstant.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstant.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstant.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstant.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstant.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstant.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        // mSwerveMods[0].setDesiredState(swerveModuleStates[0], isOpenLoop); // | if you want a specific module to work.
        // mSwerveMods[1].setDesiredState(swerveModuleStates[1], isOpenLoop); // | 
        // mSwerveMods[2].setDesiredState(swerveModuleStates[2], isOpenLoop); // | must put in comment the command in the for loop above (line 67 for now).
        // mSwerveMods[3].setDesiredState(swerveModuleStates[3], isOpenLoop); // |

    }    

    public void setStop(){
        mSwerveMods[0].forceSetAngle(Rotation2d.fromDegrees(45));
        mSwerveMods[1].forceSetAngle(Rotation2d.fromDegrees(-45));
        mSwerveMods[2].forceSetAngle(Rotation2d.fromDegrees(-45));
        mSwerveMods[3].forceSetAngle(Rotation2d.fromDegrees(45));
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstant.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return ((Constants.SwerveConstant.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw()));
    }


    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        getTab().putInDashboard("pose y", getPose().getX(), false);
        getTab().putInDashboard("pose x", getPose().getY(), false);
        getTab().putInDashboard("yaw", gyro.getYaw(), false);
        getTab().putInDashboard("roll", gyro.getRoll(), false);
        getTab().putInDashboard("pitch", gyro.getPitch(), false);
        swerveOdometry.update(getYaw(), getModulePositions());  
        for(SwerveModule mod : mSwerveMods){
            getTab().putInDashboard("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees(), false);
            getTab().putInDashboard("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees(), false);
            getTab().putInDashboard("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond, false);    
        }
    }
}