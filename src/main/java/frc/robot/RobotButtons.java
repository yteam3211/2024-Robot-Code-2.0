package frc.robot;

import javax.swing.GroupLayout.Group;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootingOutput;
import frc.robot.commands.ShootingPosition;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.armPosition;
import frc.robot.commands.collectCommand;
import frc.robot.commands.collectOutput;
import frc.robot.commands.gtipperCommand;
import frc.robot.commands.setPoitCollectCommand;
import frc.robot.commands.shootingCommandGroup;
import frc.robot.commands.simpleOutputCommand;
import frc.robot.commands.resetCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.commands.TurnToZero;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick coPilotJoystick = new Joystick(0);
    public static Joystick driver = new Joystick(1);
    private final Trigger turnToZero = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
 

    
    public Trigger resetGyro = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger halfSpeed = new Trigger(() -> driver.getRawButton(XboxController.Button.kRightBumper.value));
    public Trigger robotCentric = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger OpenCollect = new Trigger(() -> coPilotJoystick.getRawButton(5)); 
    public Trigger collectWheels = new Trigger(() -> coPilotJoystick.getRawAxis(2)>0.3);
    public Trigger collectWheelsBack = new Trigger(() -> coPilotJoystick.getRawButton(8)); 
    public Trigger shootingVelocityLow = new Trigger(() -> coPilotJoystick.getPOV () == 180);
    public Trigger shootingVelocityHigh = new Trigger(() -> coPilotJoystick.getPOV () == 0);
    public Trigger shootingVelocityMiddle = new Trigger(() -> coPilotJoystick.getPOV () == 270);
    public Trigger humanArm = new Trigger(() -> coPilotJoystick.getRawButton(4)); 
    public Trigger middleArm = new Trigger(() -> coPilotJoystick.getRawButton(3)); 
    public Trigger lowArm = new Trigger(() -> coPilotJoystick.getRawButton(1)); 
    // public Trigger driveArm = new Trigger(() -> coPilotJoystick.getRawButton(1)); 
    public Trigger openGripper = new Trigger(() -> coPilotJoystick.getRawAxis(3)>0.3);
    public Trigger closeGripper = new Trigger(() -> coPilotJoystick.getRawButton(6)); 
    public Trigger resetTrigger = new Trigger(() -> coPilotJoystick.getRawButton(2)); 



    public void loadButtons(ShootingSubsystem shootingSubsystem, CollectSubsystem collectSubsyste, armSubsystem armSubsystem, Swerve swerve) {
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> driver.getRawAxis(XboxController.Axis.kLeftY.value), 
                () -> driver.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> driver.getRawAxis(XboxController.Axis.kRightX.value), 
                () -> false
            )
        );
        OpenCollect.whileFalse(new setPoitCollectCommand(collectSubsyste, 0));
        OpenCollect.whileTrue(new setPoitCollectCommand(collectSubsyste, -1000));
        collectWheels.whileTrue(new collectOutput(collectSubsyste, 0.6, 0.1));
        collectWheelsBack.whileTrue(new collectOutput(collectSubsyste, -0.6, -0.5));
        shootingVelocityLow.onTrue(new simpleOutputCommand(shootingSubsystem, 0));
        shootingVelocityHigh.onTrue(new simpleOutputCommand(shootingSubsystem, 0.5));
        shootingVelocityMiddle.onTrue(new simpleOutputCommand(shootingSubsystem, 0));
        humanArm.onTrue(new armPosition(armSubsystem, -7));
        lowArm.onTrue(new armPosition(armSubsystem, -25.4));
        middleArm.onTrue(new armPosition(armSubsystem, 0));
        openGripper.onTrue(new gtipperCommand(armSubsystem, -5));
        closeGripper.onTrue(new gtipperCommand(armSubsystem, 3.3));
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        resetTrigger.whileTrue(new resetCommand(shootingSubsystem, collectSubsyste, armSubsystem));
        turnToZero.onTrue(new TurnToZero(swerve));
    }
}










