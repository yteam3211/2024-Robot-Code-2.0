package frc.robot;

import javax.swing.GroupLayout.Group;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.resetCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.armPosition;
import frc.robot.commands.collectGroupCommand;
import frc.robot.commands.collectOutput;
import frc.robot.commands.gripperCommand;
import frc.robot.commands.setPointCollectCommand;
import frc.robot.commands.shootingOutputCommand;
import frc.robot.commands.resetCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.commands.LimelightPID.ForwardAndBack;
import frc.robot.commands.LimelightPID.SideToSide;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.collectWheels;import frc.util.vision.Limelight;


// Yteam loadButtons
public class RobotButtons {

    public static Joystick coPilotJoystick = new Joystick(1);
    public static Joystick driver = new Joystick(0);

    public Trigger resetGyro = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public static Trigger halfSpeed = new Trigger(() -> driver.getRawButton(XboxController.Button.kRightBumper.value));
    public Trigger robotFCentric = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger OpenCollect = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger collectWheelsBack = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kStart.value));
    public Trigger shootingLow = new Trigger(() -> coPilotJoystick.getPOV() == 180);
    public Trigger shootingHigh = new Trigger(() -> coPilotJoystick.getPOV() == 0);
    public Trigger shootingMiddle = new Trigger(() -> coPilotJoystick.getPOV() == 270);
    public Trigger humanArm = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kY.value));
    public Trigger middleArm = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kX.value));
    public Trigger lowArm = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kA.value));
    // public Trigger driveArm = new Trigger(() -> coPilotJoystick.getRawButton(1));
    public Trigger openGripper = new Trigger(() -> coPilotJoystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.3);
    public Trigger closeGripper = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kRightBumper.value));
    public Trigger resetTrigger = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kB.value));
    public Trigger SeconderyResetTrigger = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kBack.value));
    public final Trigger LimelightAprilTag = new Trigger(() -> driver.getRawButton(XboxController.Button.kB.value));

    /**
     * @param shootingSubsystem
     * @param collectSubsyste
     * @param armSubsystem
     * @param swerve
     */
    public void loadButtons(ShootingSubsystem shootingSubsystem, CollectSubsystem collectSubsystem,
            armSubsystem armSubsystem, Swerve swerve,collectWheels collectWheels, Limelight limelight) {
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> driver.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> driver.getRawAxis(XboxController.Axis.kLeftX.value),
                        () -> driver.getRawAxis(XboxController.Axis.kRightX.value),
                        () -> false));
        OpenCollect.whileFalse(new collectGroupCommand(collectSubsystem,collectWheels, 0, 0, 0));
        OpenCollect.whileTrue(new collectGroupCommand(collectSubsystem,collectWheels, 0.8, 0.3, 250));
        collectWheelsBack.whileTrue(new collectOutput(collectWheels, -0.6, -0.5));
        shootingLow.onTrue(new shootingOutputCommand(shootingSubsystem, 0.45));
        shootingMiddle.onTrue(new shootingOutputCommand(shootingSubsystem, 0.7));
        // shootingHigh.onTrue(new shootingOutputCommand(shootingSubsystem, 0.96));
        humanArm.onTrue(new armPosition(armSubsystem, -19));
        lowArm.onTrue(new armPosition(armSubsystem, -63.5));  
        middleArm.onTrue(new armPosition(armSubsystem, 0));
        // openGripper.whileTrue(new gripperCommand(armSubsystem, -6.5));
        closeGripper.whileTrue(new gripperCommand(armSubsystem, -12.1));
        closeGripper.whileFalse(new gripperCommand(armSubsystem, 0.333));
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        resetTrigger.and(SeconderyResetTrigger).onTrue(new resetCommand(shootingSubsystem, collectSubsystem, armSubsystem));
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        LimelightAprilTag.whileTrue(new SideToSide(limelight, swerve, true));
    }
}
