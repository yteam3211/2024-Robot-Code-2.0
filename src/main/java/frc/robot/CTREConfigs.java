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
            Constants.SwerveConstant.angleEnableCurrentLimit, 
            Constants.SwerveConstant.angleContinuousCurrentLimit, 
            Constants.SwerveConstant.anglePeakCurrentLimit, 
            Constants.SwerveConstant.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveConstant.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveConstant.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveConstant.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveConstant.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConstant.driveEnableCurrentLimit, 
            Constants.SwerveConstant.driveContinuousCurrentLimit, 
            Constants.SwerveConstant.drivePeakCurrentLimit, 
            Constants.SwerveConstant.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveConstant.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveConstant.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveConstant.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveConstant.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveConstant.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveConstant.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveConstant.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
    
}