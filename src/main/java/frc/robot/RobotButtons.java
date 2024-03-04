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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwereCommands.DriveToTarget;
import frc.robot.commands.SwereCommands.TeleopSwerve;
import frc.robot.commands.SwereCommands.TurnToShootingCommand;
import frc.robot.commands.AutoCommands.AutoShooingWheels;
import frc.robot.commands.Eleavator.EleavatorClimbDown;
import frc.robot.commands.Eleavator.EleavatorUpCommand;
import frc.robot.commands.Eleavator.OpenElevatorCommanGroup;
import frc.robot.commands.Eleavator.EleavatorDown;
import frc.robot.commands.Eleavator.EleavatorOutput;
import frc.robot.commands.Eleavator.PitchAndEleavator;
import frc.robot.commands.IntakeCommands.IntakeAndTransferCommand;
import frc.robot.commands.IntakeCommands.IntakeBackwordsCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeWheels;
import frc.robot.commands.ShootingCommands.CompleteAMPShootingCommand;
import frc.robot.commands.ShootingCommands.CompleteSpeakerShootingCommand;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.PitchCommands.SpeakerPitchCommand;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingSpeedCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TransferSubsystem;
import frc.util.vision.Limelight;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingVelocity;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerShootingCommand;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerIntakeCommand;
import frc.robot.commands.IntakeCommands.TransferCommands.TransferCommand;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerOutput;
// import frc.robot.commands.Balance;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick systems = new Joystick(1);
    public static Joystick driver = new Joystick(0);
    
    // driver jpoystick buttons
    public static DoubleSupplier BreakValue = () -> driver.getRawAxis(PS5Controller.Axis.kR2.value);
    public static Trigger resetGyro = new Trigger(() -> driver.getRawButton(PS5Controller.Button.kL1.value));
    public Trigger turnToShooting = new Trigger(() -> driver.getRawButton(PS5Controller.Button.kR1.value));
    public static Trigger forwardJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(PS5Controller.Axis.kLeftY.value)) > 0.1);
    public static Trigger sidesJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(PS5Controller.Axis.kLeftX.value)) > 0.1);
    public static Trigger rotationJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(PS5Controller.Axis.kRightX.value)) > 0.1);
    
    // systems joystick buttons
    public static Trigger intakeTrigger = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kR2.value)); 
    public static Trigger completeSpeakerShootingTrigger = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kTriangle.value));
    public static Trigger apmShootingTrigger = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kCross.value));
    public static Trigger pitchDown = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kCircle.value)); 
     public static Trigger pitch = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kSquare.value));
    public static Trigger climbUpTrigger = new Trigger(() ->  systems.getPOV() == 0);
    public static Trigger  elevstorDown = new Trigger(() ->  systems.getPOV() == 180);
    public static Trigger shoot = new Trigger(() ->  systems.getPOV() == 270);
    public static Trigger shootTest = new Trigger(() ->  systems.getPOV() == 90);
    


    public static Trigger PitchTrigger = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kOptions.value));
    public static Trigger shootingReverse = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kL1.value));
    public static Trigger intakeReverse = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kR1.value));
    
    public static Trigger kicker = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kL2.value));

    

    /**
     * @param shootingSubsystem
     * @param collectSubsyste
     * @param armSubsystem
     * @param swerve
     */
    public void loadButtons(Swerve swerve, Limelight limelight, ShootingSubsystem shootingSubsystem, PitchingSubsystem pitchingSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,ElevatorSubsystem elevatorSubsystem,KickerSubsystem kickerSubsystem, ShootingMath shootingMath) {
        // driver joystick commands
        swerve.setDefaultCommand(
            new TeleopSwerve(
                    swerve,
                    () -> -driver.getRawAxis(PS5Controller.Axis.kLeftY.value),
                    () -> -driver.getRawAxis(PS5Controller.Axis.kLeftX.value),
                    () -> -driver.getRawAxis(PS5Controller.Axis.kRightX.value)
                    ));
        resetGyro.onTrue(new InstantCommand(() -> Robot.m_robotContainer.getSwerve().zeroGyro()));

        // systems joystick commands
        completeSpeakerShootingTrigger.whileTrue(new CompleteSpeakerShootingCommand(swerve, limelight, shootingSubsystem, pitchingSubsystem, elevatorSubsystem, kickerSubsystem, shootingMath)); 
        // completeSpeakerShootingTrigger.onFalse(new InstantCommand(() -> shootingSubsystem.setShooterOutput(0)));
        shootingReverse.whileTrue(new ShootingSpeedCommand(shootingSubsystem, kickerSubsystem, 2000, 0.4));
        shoot.whileTrue(new ParallelCommandGroup(new PitchPos(pitchingSubsystem, 52),new ShootingVelocity(shootingSubsystem, Constants.SHOOTING_SPEAKER_VELCITY)));
        apmShootingTrigger.whileTrue(new CompleteAMPShootingCommand(shootingSubsystem, pitchingSubsystem, elevatorSubsystem));
        // .whileTrue(new CompleteAMPShootingCommand(shootingSubsystem, pitchingSubsystem));
        apmShootingTrigger.onFalse(new KickerShootingCommand(kickerSubsystem, shootingSubsystem, Constants.KICKER_OUTPUT));
        
        shootTest.onTrue(new PitchPos(pitchingSubsystem, 32));

        intakeTrigger.whileTrue(new IntakeAndTransferCommand( intakeSubsystem, transferSubsystem, shootingSubsystem, kickerSubsystem,pitchingSubsystem).onlyWhile(()-> elevatorSubsystem.getElevatorHight() < 10));
        // intakeTrigger.onFalse(new PitchPos(pitchingSubsystem, 0));
        intakeReverse.whileTrue(new ParallelCommandGroup( new IntakeBackwordsCommand(intakeSubsystem, 0.4), new TransferCommand(transferSubsystem, -0.5),new KickerIntakeCommand(kickerSubsystem, shootingSubsystem, -0.3)));

        kicker.whileTrue(new KickerOutput(kickerSubsystem, shootingSubsystem, Constants.KICKER_OUTPUT));

        climbUpTrigger.onTrue(new OpenElevatorCommanGroup(elevatorSubsystem, pitchingSubsystem, Constants.CLIMB_ELEVATOR_HIGHT));
        // climb.onTrue(new EleavatorClimbDown(elevatorSubsystem, -90));
        elevstorDown.onTrue(new EleavatorClimbDown(elevatorSubsystem, -90));

        PitchTrigger.onTrue(new SpeakerPitchCommand(limelight, pitchingSubsystem, elevatorSubsystem, shootingMath, shootingSubsystem));
        pitchDown.onTrue(new PitchPos(pitchingSubsystem, 0));//TODO: ANGLE = 0!!
        pitch.onTrue(new PitchPos(pitchingSubsystem,-40));

        turnToShooting.onTrue(new TurnToShootingCommand(swerve, limelight, shootingMath));

    }
}

