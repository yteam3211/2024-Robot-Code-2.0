package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;  if you use talon SRX mag Encoder
import com.ctre.phoenix.sensors.AbsoluteSensorRange;            
import com.ctre.phoenix.sensors.SensorInitializationStrategy;   
import com.ctre.phoenix.sensors.SensorTimeBase;                 
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConst.angleEnableCurrentLimit, 
            Constants.SwerveConst.angleContinuousCurrentLimit, 
            Constants.SwerveConst.anglePeakCurrentLimit, 
            Constants.SwerveConst.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveConst.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveConst.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveConst.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveConst.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConst.driveEnableCurrentLimit, 
            Constants.SwerveConst.driveContinuousCurrentLimit, 
            Constants.SwerveConst.drivePeakCurrentLimit, 
            Constants.SwerveConst.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveConst.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveConst.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveConst.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveConst.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveConst.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveConst.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveConst.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
    
}