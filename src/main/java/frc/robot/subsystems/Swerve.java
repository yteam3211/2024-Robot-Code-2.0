package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.dashboard.SuperSystem;
import frc.util.PID.Gains;
import frc.util.vision.Limelight;
import frc.robot.AllianceSpecs;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

// import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class Swerve extends SuperSystem {
    // public SwerveDriveOdometry swerveOdometry;
    public static DigitalOutput redLED = new DigitalOutput(Constants.RED_LED_ID);
    public static DigitalOutput greenLED = new DigitalOutput(Constants.GREEN_LED_ID);
    public static DigitalOutput blueLED = new DigitalOutput(Constants.BLUE_LED_ID);
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Limelight limelight;
 

    public static final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    public Swerve(Limelight limelight) {
        // Gains RotationGains
        super("Swerve");

        this.limelight = limelight;
        zeroGyro();
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstant.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstant.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstant.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstant.Mod3.constants)
        };
        // getTab().getTab().add("field", 0).withWidget(BuiltInWidgets.kField).getEntry(); 
            AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.2, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        
        

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstant.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
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

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = 
            Constants.SwerveConstant.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstant.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }    


    public void lockWheels(){
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
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.SwerveConstant.swerveKinematics.toChassisSpeeds(getModuleStates());
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
    
    public void setLEDS(boolean red, boolean green, boolean blue){
        redLED.set(red);
        greenLED.set(green);
        blueLED.set(blue);
    }



    @Override
    public void periodic(){
        getTab().putInDashboard("pose x", getPose().getX(), false);
        getTab().putInDashboard("pose y", getPose().getY(), false);
        getTab().putInDashboard("pose Rotation", getPose().getRotation().getDegrees(), false);
        
        // getTab().putInDashboard("Cancoder position", SwerveModule.angleEncoder.getAbsolutePosition(), false);
        getTab().putInDashboard("yaw", gyro.getYaw(), false);
        getTab().putInDashboard("alliance", AllianceSpecs.isRed, false);
        SmartDashboard.putNumber("gyro", gyro.getYaw());
        // getTab().putInDashboard("roll", gyro.getRol l(), false);
        // getTab().putInDashboard("pitch", gyro.getPitch(), false);
        // swerveOdometry.update(getYaw(), getModulePositions());
        poseEstimator.update(getYaw(), getModulePositions());
        getTab().putInDashboard("is autonomus", !Robot.isAutonomous, false);
        if(limelight.isValid() && !Robot.isAutonomous){
            Pose2d camPose = new Pose2d(AllianceSpecs.poseX.getAsDouble(), AllianceSpecs.poseY.getAsDouble(), getYaw());
            
            poseEstimator.addVisionMeasurement(camPose, limelight.getLatency());
            
            poseEstimator.resetPosition(getYaw(), getModulePositions(), camPose);
            getTab().putInDashboard("LL x pos", AllianceSpecs.poseX.getAsDouble(), false);
            getTab().putInDashboard("LL y pos", AllianceSpecs.poseY.getAsDouble(), false);
        }
        for(SwerveModule mod : mSwerveMods){
            // getTab().putInDashboard("Mod " + mod.moduleNumber + " CANcoder", mod.getCanCoder().getDegrees(), false);
            // getTab().putInDashboard("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees(), false);
            getTab().putInDashboard("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond, false);
        }
    }
}