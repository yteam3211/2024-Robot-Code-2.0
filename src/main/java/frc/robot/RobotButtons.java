package frc.robot;

import java.util.function.BooleanSupplier;

import javax.swing.GroupLayout.Group;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

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
import frc.robot.commands.ShootingCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToZeroCommand;
import frc.robot.commands.armPosition;
// import frc.robot.commands.ClosingCollectGroupCommand;
// import frc.robot.commands.IntakeAndArm;
import frc.robot.commands.collectWheelsCommand;
import frc.robot.commands.setPointCollectCommand;
import frc.robot.commands.shootingOutputCommand;
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
    public Trigger LimelightRetroReflectiveMid = new Trigger(() -> driver.getRawButton(XboxController.Button.kX.value));
    public Trigger LimelightRetroReflectiveFloor = new Trigger(() -> driver.getRawButton(XboxController.Button.kA.value));
    public static Trigger halfSpeed = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.3);
    public Trigger robotFCentric = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public static Trigger forwardJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kLeftY.value)) > 0.1);
    public static Trigger sidesJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kLeftX.value)) > 0.1);
    public static Trigger rotationJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kRightX.value)) > 0.1);
    public static Trigger rightGRIDmovment = new Trigger(() -> driver.getPOV() == 90);
    public static Trigger leftGRIDmovment = new Trigger(() -> driver.getPOV() == 270);
    public Trigger TurnToZero = new Trigger(() -> driver.getPOV() == 0);

    // systems jpoystick buttons
    public Trigger OpenCollect = new Trigger(() -> systems.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.3);
    public Trigger collectWheelsBack = new Trigger(() -> systems.getRawButton(XboxController.Button.kStart.value));
    public Trigger shootingLow = new Trigger(() -> systems.getPOV() == 180);
    public Trigger shootingHigh = new Trigger(() -> systems.getPOV() == 0);
    public Trigger shootingMiddle = new Trigger(() -> systems.getPOV() == 270);
    public Trigger shootinghTrigger = new Trigger(() -> systems.getPOV() == 90);
    public Trigger ArmCollect1 = new Trigger(() -> systems.getRawButton(XboxController.Button.kY.value));
    public Trigger ArmCollect2 = new Trigger(() -> systems.getRawButton(XboxController.Button.kX.value));
    public Trigger ArmCollect3 = new Trigger(() -> systems.getRawButton(XboxController.Button.kA.value));
    // public Trigger driveArm = new Trigger(() -> coPilotJoystick.getRawButton(1));
    public static Trigger armBackTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kRightBumper.value));
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
        
        LimelightAprilTag.whileTrue(new LimelightCommand(limelight, swerve, true, -1, 0));
        // LimelightRetroReflectiveFloor.whileTrue(new LimelightCommand(limelight, swerve, false, 8));
        LimelightRetroReflectiveMid.whileTrue(new LimelightCommand(limelight, swerve, false, 16.5, 0));
        Balance.onTrue(new BalanceCommand(swerve));
        rightGRIDmovment.and(GRIDmovmentHelper).onTrue(new rightGRIDmovmentCommand(swerve));
        leftGRIDmovment.and(GRIDmovmentHelper).onTrue(new LeftGRIDmovmentCommand(swerve));
        TurnToZero.whileTrue(new TurnToZeroCommand(swerve));

        // systems joystick commands
        OpenCollect.whileTrue(new OpenIntakeAndArm(collectSubsystem, collectWheels, armCollectSubsystem, -0.5, -0.15, 300, 6.11));
        // OpenCollect.whileFalse(new IntakeAndArm(collectSubsystem, collectWheels, armCollectSubsystem, 0, 0, 0, 0));
        collectWheelsBack.whileTrue(new collectWheelsCommand(collectWheels, 0.6, 0.5));
        shootingLow.onTrue(new ShootingCommand(shootingSubsystem, cartridgeSubsystem,1850, 0.19));
        shootingHigh.onTrue(new ShootingCommand(shootingSubsystem, cartridgeSubsystem,6000, 0.4));
        shootinghTrigger.onTrue(new ShootingCommand(shootingSubsystem, cartridgeSubsystem,3000, 0.4));
        shootingMiddle.onTrue(new shootingOutputCommand( cartridgeSubsystem,0.2, 1000));
        // shootingMiddle.onTrue(new shootingOutputCommand(shootingSubsystem, 0.2, 1000));
        // shootinghTrigger.onTrue(new shootingOutputCommand(shootingSubsystem, 0.8, 6930));
        // humanArm.onTrue(new armPosition(armSubsystem, -20.7));
        // midArm.onTrue(new armPosition(armSubsystem, -65));  
        // floorArm.onTrue(new armPosition(armSubsystem, -70.7));
        ArmCollect1.onTrue(new ArmCollectCommand(armCollectSubsystem, 5.2, 1000));
        ArmCollect2.onTrue(new ArmCollectCommand(armCollectSubsystem, 0, 1000));
        // ArmCollect3.
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        resetTrigger.and(SeconderyResetTrigger).onTrue(new resetCommand(shootingSubsystem, collectSubsystem, armCollectSubsystem, cartridgeSubsystem));
        resetTrigger.onTrue(new armPosition(armSubsystem, 0));   
        
      
    }
}
