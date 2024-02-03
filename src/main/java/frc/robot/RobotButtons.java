package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.GroupLayout.Group;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwereCommands.TeleopSwerve;
import frc.robot.commands.SwereCommands.TurnToShootingCommand;
import frc.robot.commands.Eleavator.EleavatorCommand;
import frc.robot.commands.IntakeCommands.IntakeAndTransferCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.ShootingCommands.CompleteShootingCommand;
import frc.robot.commands.ShootingCommands.KickerCommand;
import frc.robot.commands.ShootingCommands.PitchPos;
import frc.robot.commands.ShootingCommands.PitchCommand;
import frc.robot.commands.ShootingCommands.ShootingSpeedCommand;
import frc.robot.commands.SwereCommands.LockWheelsCommnad;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TransferSubsystem;
import frc.util.vision.Limelight;
// import frc.robot.commands.Balance;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick systems = new Joystick(1);
    public static Joystick driver = new Joystick(0);

    // driver jpoystick buttons
    public static DoubleSupplier BreakValue = () -> driver.getRawAxis(PS5Controller.Button.kR2.value);
    
    // public static Trigger forwardJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kLeftY.value)) > 0.1);
    // public static Trigger sidesJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kLeftX.value)) > 0.1);
    // public static Trigger rotationJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kRightX.value)) > 0.1);
    // public static Trigger rotationJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kRightX.value)) > 0.1);
    // systems joystick buttons
    
    public static Trigger intakeTrigger = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kL2.value));
    public static Trigger speakerShootingTrigger = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kTriangle.value));
    public static Trigger apmShootingTrigger = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kCross.value));
    public static Trigger climbTrigger = new Trigger(() -> systems.getPOV() == 0);

    public static Trigger pitchTrigger = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kR2.value));
    public static Trigger kicker = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kCircle.value));
    

    /**
     * @param shootingSubsystem
     * @param collectSubsyste
     * @param armSubsystem
     * @param swerve
     */
    public void loadButtons(Swerve swerve, Limelight limelight, ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,ElevatorSubsystem eleavatorSubsystem,KickerSubsystem kickerSubsystem) {
        // driver joystick commands
        swerve.setDefaultCommand(
            new TeleopSwerve(
                    swerve,
                    () -> driver.getRawAxis(PS5Controller.Axis.kLeftY.value),
                    () -> driver.getRawAxis(PS5Controller.Axis.kLeftX.value),
                    () -> driver.getRawAxis(PS5Controller.Axis.kRightX.value)
                    ));

                // systems joystick commands
        PitchTest.whileTrue(new PitchCommand(limelight, pitchingSubsystem, eleavatorSubsystem, 10));
        kicker.whileTrue(new KickerCommand(kickerSubsystem,shootingSubsystem,0.4));
        
        // speakerShootingTrigger.onTrue(new CompleteShootingCommand( swerve,  limelight,  shootingSubsystem,  pitchingSubsystem, eleavatorSubsystem, kickerSubsystem));                    climbTrigger.onTrue(new EleavatorCommand(eleavatorSubsystem, 0));
        speakerShootingTrigger.whileTrue(new ShootingSpeedCommand(shootingSubsystem, 17000));                    
        climbTrigger.whileTrue(new EleavatorCommand(eleavatorSubsystem, 200));
        // intakeTrigger.whileTrue(new IntakeCommand(intakeSubsystem, Constants.INTAKE_OPEN_POSITION, -1000));

        pitchTrigger.onTrue(new PitchPos(pitchingSubsystem, 10));
    }
}

