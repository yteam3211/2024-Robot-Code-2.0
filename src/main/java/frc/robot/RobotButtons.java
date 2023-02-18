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
import frc.robot.subsystems.CollectSubsyste;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick coPilotJoystick = new Joystick(1);
    public static Joystick driver = new Joystick(0);

    
    public Trigger resetGyro = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger halfSpeed = new Trigger(() -> driver.getRawButton(XboxController.Button.kRightBumper.value));
    public Trigger robotCentric = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public Trigger OpenCollect = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kLeftBumper.value)); 
    public Trigger collectWheels = new Trigger(() -> coPilotJoystick.getRawAxis(XboxController.Axis.kLeftTrigger.value)>0.3);
    public Trigger collectWheelsBack = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kStart.value)); 
    public Trigger shootingVelocityLow = new Trigger(() -> coPilotJoystick.getPOV () == 180);
    public Trigger shootingVelocityHigh = new Trigger(() -> coPilotJoystick.getPOV () == 0);
    public Trigger shootingVelocityMiddle = new Trigger(() -> coPilotJoystick.getPOV () == 270);
    public Trigger humanArm = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kY.value)); 
    public Trigger middleArm = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kX.value)); 
    public Trigger lowArm = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kA.value)); 
    // public Trigger driveArm = new Trigger(() -> coPilotJoystick.getRawButton(1)); 
    public Trigger openGripper = new Trigger(() -> coPilotJoystick.getRawAxis(XboxController.Axis.kRightTrigger.value)>0.3);
    public Trigger closeGripper = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kRightBumper.value)); 
    public Trigger resetTrigger = new Trigger(() -> coPilotJoystick.getRawButton(XboxController.Button.kB.value)); 



    public void loadButtons(ShootingSubsystem shootingSubsystem, CollectSubsyste collectSubsyste, armSubsystem armSubsystem, Swerve swerve) {
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
        OpenCollect.whileTrue(new setPoitCollectCommand(collectSubsyste, 400));
        collectWheels.whileTrue(new collectOutput(collectSubsyste, 0.6, 0.1));
        collectWheelsBack.whileTrue(new collectOutput(collectSubsyste, -0.6, -0.5));
        shootingVelocityLow.onTrue(new simpleOutputCommand(shootingSubsystem, 0.2));
        shootingVelocityMiddle.onTrue(new simpleOutputCommand(shootingSubsystem, 0.5));
        shootingVelocityHigh.onTrue(new simpleOutputCommand(shootingSubsystem, 0.6));
        humanArm.onTrue(new armPosition(armSubsystem, -7));
        lowArm.onTrue(new armPosition(armSubsystem, -56));
        // middleArm.onTrue(new armPosition(armSubsystem, 0));
        openGripper.onTrue(new gtipperCommand(armSubsystem, -5));
        closeGripper.onTrue(new gtipperCommand(armSubsystem, 3.3));
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        resetTrigger.onTrue(new resetCommand(shootingSubsystem, collectSubsyste, armSubsystem));




        

        // load buttons
    }
}










