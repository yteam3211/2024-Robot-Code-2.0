package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstant;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfigurator;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private TalonFX mDriveMotor;
    // private TalonSRX angleEncoder;
    private CANcoder angleEncoder;

    private RelativeEncoder integratedAngleEncoder;
    private final SparkPIDController angleController;
    

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstant.driveKS, Constants.SwerveConstant.driveKV, Constants.SwerveConstant.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.CanBus.CANivore);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.CanBus.CANivore);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);

        setSpeed(desiredState, isOpenLoop);
    }

    public void testSpeed(boolean isOpenLoop, double percentOutput){
        mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        double desiredSpeed = desiredState.speedMetersPerSecond;
        double breakDesiredSpeed = desiredSpeed;
        if (RobotButtons.BreakValue.getAsDouble() > 0.01){
        breakDesiredSpeed *= ((1.15 - RobotButtons.BreakValue.getAsDouble()));
        if(breakDesiredSpeed > 1)
            breakDesiredSpeed -= 0.15;
        desiredSpeed = breakDesiredSpeed;
        }

        if(isOpenLoop){
            double percentOutput = desiredSpeed / Constants.SwerveConstant.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredSpeed, Constants.SwerveConstant.wheelCircumference, Constants.SwerveConstant.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredSpeed));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstant.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public void forceSetAngle(Rotation2d angle){
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        integratedAngleEncoder.setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }


    private void configAngleEncoder(){      
        angleEncoder.getPosition().setUpdateFrequency(5);  
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Constants.SwerveConstant.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.SwerveConstant.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.SwerveConstant.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstant.angleConversionFactor);
        angleController.setP(Constants.SwerveConstant.angleKP);
        angleController.setI(Constants.SwerveConstant.angleKI);
        angleController.setD(Constants.SwerveConstant.angleKD);
        angleController.setFF(Constants.SwerveConstant.angleKF);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.SwerveConstant.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.SwerveConstant.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.SwerveConstant.wheelCircumference, Constants.SwerveConstant.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.SwerveConstant.wheelCircumference, Constants.SwerveConstant.driveGearRatio), 
            getAngle()
        );
    }
}