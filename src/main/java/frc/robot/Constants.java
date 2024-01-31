package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.Swerve;
import frc.util.vision.Limelight;

public final class Constants {

    // General constants
    public static final double SHOOTING_ANGLE_TRESHOLD = 2; //TODO: set when the robot is build
    public static final double SHOOTING_VELOCITY_TRESHOLD = 10; //TODO: set when the robot is build 
    public static final double SHOOTING_VELCITY = 0; //TODO: set when the robot is build
    
    public static final double PITCHING_ENCODER_OFFSET = Units.rotationsToDegrees(0); //TODO: set when the robot is build
    public static final double LIMELIGHT_OFFSET_ANGLE_FROM_PIVOT = -8.60890684614;
    public static final double LIMELIGHT_TO_PIVOT = 347.32;
    public static final double SPEAKER_APRILTAG_HIGHT = 1450; //in millimeters
    public static final double SPEAKER_HIGHT = 2150; //in millimeters
    public static final double VERTICAL_LIMELIGHT_TO_CENTER_SHOOTER = 150; //in millimeters
    public static final double HORIZONTAL_LIMELIGHT_TO_CENTER_SHOOTER = 150; //in millimeters


    public static final double ELEAVATOR_TRESHOLD = 10; //TODO: set when the robot is build
    public static final double ELEAVATOR_GEAR_RATIO = 15.31 / 1;
    private static final double ELEAVATOR_WINCH_DIAMETER = 50; // diameter in millimeters
    public static final double ELEAVATOR_WINCH_CIRCUMFERENCE = ELEAVATOR_WINCH_DIAMETER * Math.PI;
    public static final double FLOOR_TO_CLOSE_ELEAVATOR = 0; //TODO: check on robot after mechanical stop has added
    public static final double RIDER_BOTTOM_TO_PITCH_PIVOT_VERTICAL = 259.84; //in millimeters

    public static final double  TURN_SWERVE_TRESHOLD = 0;//TODO: set when the robot is build

    public static final double INTAKE_OPEN_POSITION = 0; //TODO: set when the robot is build
    public static final double INTAKE_WHEELS_OUTPUT = 0; //TODO: set when the robot is build

    public static final double MIN_ELEAVATOR_POS = 0;    
    public static final double MAX_ELEAVATOR_POS = 0;

    

    // ID constants
    public static final int INTAKE_OPEN_MOTOR_ID = 0;
    public static final int INTAKE_WHEELS_MOTOR_ID = 25;
    public static final int INTAKE_MICROSWITCH_ID = 0;
    
    public static final int TRANSFER_MOTOR_ID = 29;
    

    public static final int MASTER_SHOOTER_MOTOR_ID = 21;
    public static final int SLAVE_SHOOTER_MOTOR_ID = 22;
    public static final int KICKER_SHOOTER_MOTOR_ID = 23;
    
    public static final int MASTER_PITCHING_MOTOR_ID = 0;
    public static final int SLAVE_PITCHING_MOTOR_ID = 0;
    public static final int PITCHING_ENCODER_ID = 0;

    public static final int MASTER_ELEAVATOR_MOTOR_ID = 13;
    public static final int SLAVE1_ELEAVATOR_MOTOR_ID = 14;
    public static final int SLAVE2_ELEAVATOR_MOTOR_ID = 15;
    public static final int MICROSWITCH_ELEAVATOR_ID = 0;


    




    public static final double stickDeadband = 0.01;

    public static final class SwerveConstant {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.59655; //TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.59655; //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.01;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); 
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 7.0; 

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Coast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Units.rotationsToDegrees(0.518311));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Units.rotationsToDegrees(0.603027));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Units.rotationsToDegrees(0.593262)); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Units.rotationsToDegrees(0.555908));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
