package frc.robot;

import java.util.function.BooleanSupplier;

import javax.swing.GroupLayout.Group;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.resetCommand;
import frc.robot.commands.rightGRIDmovmentCommand;
import frc.robot.commands.ArmCollectCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.LeftGRIDmovmentCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.OpenIntakeAndArm;
import frc.robot.commands.ShootingCommnads.ShootingCommand;
import frc.robot.commands.ShootingCommnads.ShootingDownGroupCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToZeroCommand;
import frc.robot.commands.armCollectOutput;
import frc.robot.commands.armPosition;
// import frc.robot.commands.ClosingCollectGroupCommand;
// import frc.robot.commands.IntakeAndArm;
import frc.robot.commands.collectWheelsCommand;
import frc.robot.commands.lockWheelsCommnad;
import frc.robot.commands.ShootingCommnads.ShootingGroupCommand;
import frc.robot.commands.setPointCollectCommand;
import frc.robot.commands.ShootingCommnads.CartridgeOutputCommand;
import frc.robot.commands.ShootingCommnads.CubeFixtureGroupCommand;
import frc.robot.commands.resetCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.robot.subsystems.shootingSubsystem;
import frc.util.vision.Limelight;
// import frc.robot.commands.Balance;


// Yteam loadButtons
public class RobotButtons {
    public static BooleanSupplier GRIDmovmentHelper = (() -> true);
    

    public static Joystick systems = new Joystick(1);
    public static Joystick driver = new Joystick(0);
    // driver jpoystick buttons
    public Trigger resetGyro = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public static Trigger Balance = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.3);
    public Trigger stop = new Trigger(() -> driver.getRawButton(XboxController.Button.kBack.value));
    public Trigger LimelightAprilTag = new Trigger(() -> driver.getRawButton(XboxController.Button.kB.value));
    public Trigger LimelightRetroReflective = new Trigger(() -> driver.getRawButton(XboxController.Button.kX.value));
    public static Trigger halfSpeed = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.3);
    public Trigger robotFCentric = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public static Trigger forwardJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kLeftY.value)) > 0.1);
    public static Trigger sidesJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kLeftX.value)) > 0.1);
    public static Trigger rotationJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kRightX.value)) > 0.1);
    public static Trigger rightGRIDmovment = new Trigger(() -> driver.getPOV() == 90);
    public static Trigger leftGRIDmovment = new Trigger(() -> driver.getPOV() == 270);
    public Trigger TurnToZero = new Trigger(() -> driver.getPOV() == 0);
    public Trigger WheelsLock = new Trigger(() -> driver.getRawButton(XboxController.Button.kStart.value));

    // systems joystick buttons
    public Trigger OpenCollect = new Trigger(() -> systems.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.3);
    public Trigger collectWheelsBack = new Trigger(() -> systems.getRawButton(XboxController.Button.kStart.value));
    public Trigger shootingLow = new Trigger(() -> systems.getPOV() == 180);
    public Trigger shootingHigh = new Trigger(() -> systems.getPOV() == 0);
    public Trigger shootingFixture = new Trigger(() -> systems.getPOV() == 270);
    public Trigger shootinghMid = new Trigger(() -> systems.getPOV() == 90);
    public Trigger openArmCollect = new Trigger(() -> systems.getRawButton(XboxController.Button.kY.value));
    public Trigger closeArmCollect = new Trigger(() -> systems.getRawButton(XboxController.Button.kX.value));
    public Trigger resetArmCollect = new Trigger(() -> systems.getRawButton(XboxController.Button.kA.value));
    public Trigger reverseShooterTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kRightBumper.value));
    public static Trigger armForwardTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger resetTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kB.value));
    public Trigger SeconderyResetTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kBack.value));
    

    /**
     * @param shootingSubsystem
     * @param collectSubsyste
     * @param armSubsystem
     * @param swerve
     */
    public void loadButtons(shootingSubsystem shootingSubsystem, CollectSubsystem collectSubsystem,
            armSubsystem armSubsystem, Swerve swerve,collectWheels collectWheels, Limelight limelight, armCollectSubsystem armCollectSubsystem,CartridgeSubsystem cartridgeSubsystem) {
        // driver joystick commands
        swerve.setDefaultCommand(
            new TeleopSwerve(
                    swerve,
                    () -> driver.getRawAxis(XboxController.Axis.kLeftY.value),
                    () -> driver.getRawAxis(XboxController.Axis.kLeftX.value),
                    () -> driver.getRawAxis(XboxController.Axis.kRightX.value),
                    () -> false,
                    0));
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        LimelightAprilTag.whileTrue(new LimelightCommand(limelight, swerve, true, -1, 1));
        LimelightRetroReflective.whileTrue(new LimelightCommand(limelight, swerve, false, 14, 1));
        Balance.onTrue(new BalanceCommand(swerve));
        rightGRIDmovment.and(GRIDmovmentHelper).onTrue(new rightGRIDmovmentCommand(swerve));
        leftGRIDmovment.and(GRIDmovmentHelper).onTrue(new LeftGRIDmovmentCommand(swerve));
        TurnToZero.whileTrue(new TurnToZeroCommand(swerve));
        WheelsLock.onTrue(new lockWheelsCommnad(swerve));
        // LimelightRetroReflectiveFloor.whileTrue(new LimelightCommand(limelight, swerve, false, 8));

        // systems joystick commands
        OpenCollect.whileTrue(new OpenIntakeAndArm(collectSubsystem, collectWheels, armCollectSubsystem, -0.5, -0.15, 250, 5.2));
        collectWheelsBack.whileTrue(new collectWheelsCommand(collectWheels, 0.6, 0.5));
        
        resetArmCollect.onTrue(new armCollectOutput(armCollectSubsystem, -0.2, 0));

        shootingHigh.onTrue(new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , 5.2, 0 , 0.3, 0.75));
        shootinghMid.onTrue(new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , 5.2, 0 , 0.4, 0.51));
        shootingLow.onTrue(new ShootingGroupCommand(shootingSubsystem, armCollectSubsystem, cartridgeSubsystem , 5.2, 0 , 0.4, 0.29));
        shootingFixture.onTrue(new CubeFixtureGroupCommand(cartridgeSubsystem, 0.3, 700, -0.2, 20));
        reverseShooterTrigger.onTrue(new ShootingCommand(shootingSubsystem, cartridgeSubsystem, armCollectSubsystem,-0.3, 0));


        openArmCollect.onTrue(new InstantCommand(() -> armCollectSubsystem.setArmCollectPosition(5.2)));
        closeArmCollect.onTrue(new InstantCommand(() -> armCollectSubsystem.setArmCollectPosition(0)));

        
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        resetTrigger.and(SeconderyResetTrigger).onTrue(new resetCommand(shootingSubsystem, collectSubsystem, armCollectSubsystem, cartridgeSubsystem));
        
        // OpenCollect.whileFalse(new IntakeAndArm(collectSubsystem, collectWheels, armCollectSubsystem, 0, 0, 0, 0));
        
        // shootingLow.onTrue(new ShootingGroupCommand(cartridgeSubsystem, shootingSubsystem,1850, 0.19, -0.1 , 3000 ));
        // shootingHigh.onTrue(new ShootingGroupCommand(cartridgeSubsystem, shootingSubsystem, 6000, 0.4, -0.1,3000));
        // shootinghTrigger.onTrue(new ShootingGroupCommand(cartridgeSubsystem ,shootingSubsystem,3000, 0.4, -0.1 ,300));
        
        // humanArm.onTrue(new armPosition(armSubsystem, -20.7));
        // midArm.onTrue(new armPosition(armSubsystem, -65));  
        // floorArm.onTrue(new armPosition(armSubsystem, -70.7));
        // resetTrigger.onTrue(new armPosition(armSubsystem, 0));   
    }
}
