package frc.robot;

import javax.swing.GroupLayout.Group;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.resetCommand;
import frc.robot.commands.ArmOutputCommand;
import frc.robot.commands.ArmTriggerCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.armPosition;
import frc.robot.commands.collectGroupCommand;
import frc.robot.commands.collectOutput;
import frc.robot.commands.gripperCommand;
import frc.robot.commands.setPointCollectCommand;
import frc.robot.commands.shootingOutputCommand;
import frc.robot.commands.resetCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.collectWheels;
import frc.util.vision.Limelight;


// Yteam loadButtons
public class RobotButtons {

    public static Joystick systems = new Joystick(1);
    public static Joystick driver = new Joystick(0);
    // driver jpoystick buttons
    public Trigger resetGyro = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger LimelightAprilTag = new Trigger(() -> driver.getRawButton(XboxController.Button.kB.value));
    public Trigger LimelightRetroReflectiveMid = new Trigger(() -> driver.getRawButton(XboxController.Button.kX.value));
    public Trigger LimelightRetroReflectiveFloor = new Trigger(() -> driver.getRawButton(XboxController.Button.kA.value));
    // public static Trigger halfSpeed = new Trigger(() -> driver.getRawButton(XboxController.Button.kRightBumper.value));
    public static Trigger halfSpeed = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.3);
    public Trigger robotFCentric = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    // systems jpoystick buttons
    public Trigger OpenCollect = new Trigger(() -> systems.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.3);
    public Trigger collectWheelsBack = new Trigger(() -> systems.getRawButton(XboxController.Button.kStart.value));
    public Trigger shootingLow = new Trigger(() -> systems.getPOV() == 180);
    public Trigger shootingHigh = new Trigger(() -> systems.getPOV() == 0);
    public Trigger shootingMiddle = new Trigger(() -> systems.getPOV() == 270);
    public Trigger humanArm = new Trigger(() -> systems.getRawButton(XboxController.Button.kY.value));
    public Trigger floorArm = new Trigger(() -> systems.getRawButton(XboxController.Button.kX.value));
    public Trigger midArm = new Trigger(() -> systems.getRawButton(XboxController.Button.kA.value));
    // public Trigger driveArm = new Trigger(() -> coPilotJoystick.getRawButton(1));
    public Trigger openGripper = new Trigger(() -> systems.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.3);
    public Trigger armBackTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kRightBumper.value));
    public Trigger armForwardTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger resetTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kB.value));
    public Trigger SeconderyResetTrigger = new Trigger(() -> systems.getRawButton(XboxController.Button.kBack.value));

    /**
     * @param shootingSubsystem
     * @param collectSubsyste
     * @param armSubsystem
     * @param swerve
     */
    public void loadButtons(ShootingSubsystem shootingSubsystem, CollectSubsystem collectSubsystem,
            armSubsystem armSubsystem, Swerve swerve,collectWheels collectWheels, Limelight limelight, GripperSubsystem gripperSubsystem) {
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
        LimelightAprilTag.whileTrue(new LimelightCommand(limelight, swerve, true, 0));
        LimelightRetroReflectiveFloor.whileTrue(new LimelightCommand(limelight, swerve, false, 8));
        LimelightRetroReflectiveMid.whileTrue(new LimelightCommand(limelight, swerve, false, 14.3));
        // systems joystick commands
        OpenCollect.whileFalse(new collectGroupCommand(collectSubsystem,collectWheels, 0, 0, 0));
        OpenCollect.whileTrue(new collectGroupCommand(collectSubsystem, collectWheels, -0.5, -0.15, 250));
        collectWheelsBack.whileTrue(new collectOutput(collectWheels, 0.6, 0.5));
        shootingLow.onTrue(new shootingOutputCommand(shootingSubsystem, 0.45, 6850));
        shootingHigh.onTrue(new shootingOutputCommand(shootingSubsystem, 0.55, 6850));
        shootingMiddle.onTrue(new shootingOutputCommand(shootingSubsystem, 0.2, 1000));
        humanArm.onTrue(new armPosition(armSubsystem, -18));
        midArm.onTrue(new armPosition(armSubsystem, -63));  
        floorArm.onTrue(new armPosition(armSubsystem, -70.7));
        openGripper.whileTrue(new gripperCommand(gripperSubsystem, -43.475));
        openGripper.whileFalse(new gripperCommand(gripperSubsystem, 0.333));
        // armBackTrigger.whileTrue(new ArmTriggerCommand(armSubsystem, 1.5));
        armBackTrigger.whileTrue(new ArmOutputCommand(armSubsystem, 0.1));
        armForwardTrigger.whileTrue(new ArmOutputCommand(armSubsystem, -0.05));
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        resetTrigger.and(SeconderyResetTrigger).onTrue(new resetCommand(shootingSubsystem, collectSubsystem, armSubsystem, gripperSubsystem));
        resetTrigger.onTrue(new armPosition(armSubsystem, 0));   
      
    }
}
