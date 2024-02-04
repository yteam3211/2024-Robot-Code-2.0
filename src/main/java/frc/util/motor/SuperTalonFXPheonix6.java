//            /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.util.motor;

// import com.ctre.phoenix.motorcontrol.IMotorController;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix6.configs.Slot0Configs;
// // import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// // import com.ctre.phoenix.motorcontrol.IMotorController;
// // import com.ctre.phoenix.motorcontrol.NeutralMode;
// // import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
// // import com.ctre.phoenix.motorcontrol.
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.ControlRequest;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.hardware.CANcoder;
// // import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// // import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

// import frc.util.PID.Gains;

// /**
//  * This class is SuperMotor of TalonFX
//  * 
//  * @author Amitai Algom
//  */
// public class SuperTalonFXPheonix6 extends TalonFX implements SuperMotor {
//     private TalonFXConfiguration talonFXConfiguration;
//     private ControlRequest controlRequest;
//     private FeedbackConfigs feedbackConfigs;
//     private double positionMultiply = 1, velocityMultiply = 1;
//     /** 
//      * This constractor is to master motor
//      * 
//      * @param deviceNumber can id
//      * @param amps         amper limitation
//      * @param inverted     when side motor move
//      * @param PhaseSensor  when side sensor move
//      * @param mNeutralMode mode of motor brake or coast
//      * @param gains
//      */
//     public SuperTalonFXPheonix6(int deviceNumber, int amps, boolean inverted, boolean PhaseSensor, NeutralMode mNeutralMode,
//             Gains gains, ControlRequest controlRequest, int canconderID) {
//         super(deviceNumber);
//         this.controlRequest = controlRequest;
//         talonFXConfiguration = new TalonFXConfiguration();
//         feedbackConfigs = new FeedbackConfigs();
//         feedbackConfigs.FeedbackRemoteSensorID = canconderID;
//         feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        
//         setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
//         getPosition().setUpdateFrequency(50);
        
//         setInverted(inverted);
//         setSensorPhase(PhaseSensor);
//         setNeutralMode(mNeutralMode);
//         configNominalOutputForward(0, 0);
//         configNominalOutputReverse(0, 0);
//         configPeakOutputForward(1, 0);
//         configPeakOutputReverse(-1, 0);
//         selectProfileSlot(0, 0);
//         config_kF(0, gains.Kf);
//         config_kP(0, gains.kp);
//         config_kI(0, gains.ki);
//         config_kD(0, gains.kd);
//         setSelectedSensorPosition(0, 0, 0);
//         getConfigurator().apply(feedbackConfigs);
//     }

//     /**
//      * This constructor is to slave motor
//      * 
//      * @param leader       The master that motor follow
//      * @param deviceNumber can id
//      * @param amps         amper limitation
//      * @param inverted     when side motor move
//      */
//     public SuperTalonFXPheonix6(IMotorController leader, int deviceNumber, int amps, boolean inverted) {
//         super(deviceNumber);

//         selectProfileSlot(0, 0);
//         setInverted(inverted);
//         follow(leader);
//     }

//     @Override
//     public void setOutput(double setOutput) {
//         if (mode == TalonFXControlMode.Position || mode == TalonFXControlMode.MotionMagic)
//             setOutput *= positionMultiply;
//         else if (mode == TalonFXControlMode.Velocity)
//             setOutput *= velocityMultiply;
//         super.set(mode, setOutput);
//     }

//     @Override
//     public double getOutput() {
//         return super.getMotorOutputPercent();
//     }

//     @Override
//     public double getAmper() {
//        return super.getMotorOutputVoltage();
//     }

//     @Override
//     public void setMode(Object mode) {
//         if (mode instanceof TalonFXControlMode)
//             mode = (TalonFXControlMode) mode;
//     }

//     @Override
//     public double getVelocity() {
//         return super.getSelectedSensorVelocity();
//     }

//     @Override
//     // public double getPosition() {
//     //     return super.getSelectedSensorPosition();
//     // }

//     @Override
//     public void reset(double pos) {
//         super.setSelectedSensorPosition(pos);
//     }
// }
