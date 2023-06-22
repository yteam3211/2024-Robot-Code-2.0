package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;
import frc.robot.subsystems.shootingSubsystem;
import frc.util.vision.Limelight;

public final class Constants {
        // subsystems

        // motors ID constants
        public static final int SHOOTING_MOTOR = 15;
        public static final int ARM_DOWN_MICROSWITCH = 0; 
        public static final int UP_MICROSWITCH = 0;
        public static final int RIGHT_LEADER_COLLECT_MOTOR = 16;
        public static final int COLLECT_CLOSE_MICROSWITCH = 1;
        public static final int ARM_MOTOR = 17;
        public static final int ARM_COLLECT_MOTOR = 20;
        public static final int GRIPPER_MOTOR = 18; 
        public static final int RIGHT_SHOOTING_M0TOR = 21; 
        public static final int LEFT_SHOOTIN_MOTOR = 22; 

        // positions constants
        public static final double ARM_OPEN_POSITION = 5.2;
        public static final double ARM_CLOSE_POSITION = 0.3;
        public static final double COLLECT_OPEN_POSITION = 250;
        
        // shooting constsnts
        public static final ShootingConstants SHOOTING_HIGH = new ShootingConstants(0.3, 0.75);
        public static final ShootingConstants SHOOTING_AUTO_HIGH = new ShootingConstants(0.3, 0.82);
        public static final ShootingConstants SHOOTING_MID = new ShootingConstants(0.4, 0.51);
        public static final ShootingConstants SHOOTING_LOW = new ShootingConstants(0.4, 0.29);

        // collect constants
        public static final double COLLECT_WHEELS_OUTPUT = -0.7;
        public static final double CENTERING_WHEELS_OUTPUT = -0.15;
        public static final double CLOSE_COLLECT_OUTPUT = -0.3;

        // subsystems constants
        // public static final SubsystemsConstants SUBSYSTEMS = new SubsystemsConstants(
        // Robot.getm_robotContainer().getM_armCollectSubsystem(),
        //  Robot.getm_robotContainer().getM_CartridgeSubsystem(),
        //  Robot.getm_robotContainer().getM_CollectSubsystem(),
        //  Robot.getm_robotContainer().getM_CollectWheels(),
        //  Robot.getm_robotContainer().getM_ShootingSubsystem(),
        //  Robot.getm_robotContainer().getS_Swerve(),
        //  Robot.getm_robotContainer().getLimelight()
        // );

    public static final class ShootingConstants{ 
        private  final double ArmPosition = ARM_OPEN_POSITION;
        private final double ArmSeconds = 0;
        private final double CartridgeOutput;
        private final double ShootingWheelsOutput;
        public ShootingConstants(double CartridgeOutput, double ShootingWheelsOutput){
            this.CartridgeOutput = CartridgeOutput;
            this.ShootingWheelsOutput  = ShootingWheelsOutput;
        }
        
        public double getArmPosition() {
            return ArmPosition;
        }
        public double getArmSeconds() {
            return ArmSeconds;
        }
        public double getCartridgeOutput() {
            return CartridgeOutput;
        }
        public double getShootingWheelsOutput() {
            return ShootingWheelsOutput;
        }
    }
    public static final class SubsystemsConstants{
        private final armCollectSubsystem armCollectSubsystem;
        private final CartridgeSubsystem cartridgeSubsystem;
        private final CollectSubsystem collectSubsystem;
        private final collectWheelsSubsystem collectWheelsSubsystem;
        private final shootingSubsystem shootingSubsystem;
        private final Swerve swerve;
        private final Limelight limelight;

        public SubsystemsConstants(armCollectSubsystem armCollectSubsystem,
                CartridgeSubsystem cartridgeSubsystem, CollectSubsystem collectSubsystem,
                collectWheelsSubsystem collectWheelsSubsystem,
                shootingSubsystem shootingSubsystem, Swerve swerve, Limelight limelight) 
            {
            this.armCollectSubsystem = armCollectSubsystem;
            this.cartridgeSubsystem = cartridgeSubsystem;
            this.collectSubsystem = collectSubsystem;
            this.collectWheelsSubsystem = collectWheelsSubsystem;
            this.shootingSubsystem = shootingSubsystem;
            this.swerve = swerve;
            this.limelight = limelight;
            }

        public armCollectSubsystem getArmCollectSubsystem() {
            return armCollectSubsystem;
        }

        public CartridgeSubsystem getCartridgeSubsystem() {
            return cartridgeSubsystem;
        }

        public CollectSubsystem getCollectSubsystem() {
            return collectSubsystem;
        }

        public collectWheelsSubsystem getCollectWheelsSubsystem() {
            return collectWheelsSubsystem;
        }

        public shootingSubsystem getShootingSubsystem() {
            return shootingSubsystem;
        }

        public Swerve getSwerve() {
            return swerve;
        }
        public Limelight getLimelight() {
            return limelight;
        }
    }
    
        


    public static final double stickDeadband = 0.01;

    public static final class SwerveConstant {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.34);; //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(20.34); //TODO: This must be tuned to specific robot
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
        public static final double driveKP = 0.01; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 7.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Coast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(35.859);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(186.240);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108.721);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees( 32.871);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
