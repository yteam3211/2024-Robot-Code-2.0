package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwereCommands.SwerveForward;
import frc.robot.commands.SwereCommands.TeleopSwerve;
import frc.robot.commands.SwereCommands.TrapGroupCommand;
import frc.robot.commands.SwereCommands.TurnSwerveCommand;
import frc.robot.commands.Elevator.ElevatorClimbDown;
import frc.robot.commands.Elevator.ElevatorClimbUpGroupCommand;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.TrapOpenElevator;
import frc.robot.commands.Elevator.ElevatorSlow;
import frc.robot.commands.IntakeCommands.IntakeAndTransferCommand;
import frc.robot.commands.IntakeCommands.IntakeBackwordsCommand;
import frc.robot.commands.ShootingCommands.CompleteAMPShootingCommand;
import frc.robot.commands.ShootingCommands.CompleteSpeakerShootingCommand;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchPos;
import frc.robot.commands.ShootingCommands.PitchCommands.PitchSlow;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingOutput;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PitchingSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TransferSubsystem;
import frc.util.vision.Limelight;
import frc.robot.commands.ShootingCommands.ShootingWheelsCommands.ShootingVelocity;
import frc.robot.commands.ShootingCommands.KickerCommands.KickerOutput;
// import frc.robot.commands.Balance;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick systems = new Joystick(1);
    public static Joystick driver = new Joystick(0);
    
    // driver jpoystick buttons
    public static DoubleSupplier BreakValue = () -> driver.getRawAxis(PS5Controller.Axis.kR2.value);
    public static Trigger resetGyro = new Trigger(() -> driver.getRawButton(PS5Controller.Button.kL1.value));
    public static Trigger forwardJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(PS5Controller.Axis.kLeftY.value)) > 0.1);
    public static Trigger sidesJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(PS5Controller.Axis.kLeftX.value)) > 0.1);
    public static Trigger rotationJoystick = new Trigger(() -> Math.abs(driver.getRawAxis(PS5Controller.Axis.kRightX.value)) > 0.1);
    public static Trigger TurnForward = new Trigger(() ->  driver.getPOV() == 0);
    public static Trigger TurnRight = new Trigger(() ->  driver.getPOV() == 90);
    public static Trigger TurnBackward = new Trigger(() ->  driver.getPOV() == 180);
    public static Trigger TurnLeft = new Trigger(() ->  driver.getPOV() == 270);
    public static Trigger CenterToTrap = new Trigger(() -> driver.getRawButton(PS5Controller.Button.kCross.value));
    public static Trigger SlowForward = new Trigger(() -> driver.getRawButton(PS5Controller.Button.kTriangle.value));    
    public static Trigger test = new Trigger(() -> driver.getRawButton(PS5Controller.Button.kCircle.value));

    
    // systems joystick buttons
    public static Trigger intakeTrigger = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kR2.value)); 
    public static Trigger intakeReverse = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kR1.value));
    public static Trigger shoot = new Trigger(() ->  systems.getPOV() == 270); 
    public static Trigger kicker = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kL2.value));    
    public static Trigger pitchDown = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kCircle.value));     
    
    
    public static Trigger completeSpeakerShootingTrigger = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kTriangle.value));
    public static Trigger apmShootingTrigger = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kCross.value));
    public static Trigger hoohup = new Trigger(() ->  systems.getRawButton(PS5Controller.Button.kSquare.value)); 
    public static Trigger ElevatorSlowUp = new Trigger(() ->  systems.getRawAxis(PS5Controller.Axis.kLeftY.value) < -0.8);
    public static Trigger ElevatorSlowDown = new Trigger(() ->  systems.getRawAxis(PS5Controller.Axis.kLeftY.value) > 0.8);
    public static Trigger EnableElevator = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kL3.value));
    public static Trigger PitchSlowUp = new Trigger(() -> systems.getRawAxis(PS5Controller.Axis.kRightY.value) < -0.8);
    public static Trigger PitchSlowDown = new Trigger(() -> systems.getRawAxis(PS5Controller.Axis.kRightY.value) > 0.8);
    public static Trigger EnablePitch = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kR3.value));
    public static Trigger elevatorUpTrigger = new Trigger(() ->  systems.getPOV() == 0);
    public static Trigger  elevstorDown = new Trigger(() ->  systems.getPOV() == 180);
    public static Trigger ClimbTrigger = new Trigger(() ->  systems.getPOV() == 90);
    // public static Trigger defenseShooting = new Trigger(() ->  systems.getPOV() == 90);

    public static Trigger TrapElevator = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kOptions.value));
    public static Trigger shootingAMPkickeer = new Trigger(() -> systems.getRawButton(PS5Controller.Button.kL1.value));
    
    public static Trigger hook = new Trigger(() -> systems.getRawButton(13));



    

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
        resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));        
        test.whileTrue(new SwerveForward(swerve));
        TurnForward.onTrue(new TurnSwerveCommand(swerve, 0));
        TurnRight.onTrue(new TurnSwerveCommand(swerve, -90));
        TurnBackward.onTrue(new TurnSwerveCommand(swerve, 180));
        TurnLeft.onTrue(new TurnSwerveCommand(swerve, 90));
        CenterToTrap.onTrue(new TrapGroupCommand(swerve, limelight));
        SlowForward.whileTrue(new SwerveForward(swerve));

        // systems joystick commands
        intakeTrigger.whileTrue(new IntakeAndTransferCommand(intakeSubsystem, transferSubsystem, shootingSubsystem, kickerSubsystem,pitchingSubsystem));
        intakeReverse.whileTrue(new IntakeBackwordsCommand(intakeSubsystem, 0.9));
        shoot.whileTrue(new ParallelCommandGroup(new PitchPos(pitchingSubsystem, 54),new ShootingVelocity(shootingSubsystem, Constants.SHOOTING_SPEAKER_VELCITY)));
        kicker.whileTrue(new KickerOutput(kickerSubsystem, shootingSubsystem, Constants.KICKER_OUTPUT));
        pitchDown.onTrue(new PitchPos(pitchingSubsystem, 0));
        

        elevstorDown.onTrue(new ParallelCommandGroup(new ElevatorDown(elevatorSubsystem, -40), new PitchPos(pitchingSubsystem, 0)));
        shootingAMPkickeer.whileTrue(new ParallelCommandGroup(new ShootingOutput(shootingSubsystem, 0.4), new KickerOutput(kickerSubsystem, shootingSubsystem, 0.4)));      
        apmShootingTrigger.onTrue(new CompleteAMPShootingCommand(shootingSubsystem, pitchingSubsystem, elevatorSubsystem));
        
        // completeSpeakerShootingTrigger.onTrue(new CompleteSpeakerShootingCommand(swerve, limelight, shootingSubsystem, pitchingSubsystem, elevatorSubsystem, kickerSubsystem, shootingMath)); 
        
        ClimbTrigger.onTrue(new ElevatorClimbDown(elevatorSubsystem, -140));

        elevatorUpTrigger.onTrue(new ElevatorClimbUpGroupCommand(elevatorSubsystem, shootingSubsystem, pitchingSubsystem)); //new OpenElevatorCommanGroup(elevatorSubsystem, pitchingSubsystem, Constants.CLIMB_ELEVATOR_HIGHT)
        ElevatorSlowUp.and(EnableElevator).whileTrue(new ElevatorSlow(elevatorSubsystem, true));
        ElevatorSlowDown.and(EnableElevator).whileTrue(new ElevatorSlow(elevatorSubsystem, false).unless(() -> ElevatorSubsystem.isEleavatorDown()));

        TrapElevator.onTrue(new TrapOpenElevator(elevatorSubsystem, pitchingSubsystem, kickerSubsystem, shootingSubsystem));

        hoohup.whileTrue(new ShootingOutput(shootingSubsystem, -0.1));
        PitchSlowDown.and(EnablePitch).whileTrue(new PitchSlow(pitchingSubsystem, false));
        PitchSlowUp.and(EnablePitch).whileTrue(new PitchSlow(pitchingSubsystem, true));
        // hook.onTrue(new PitchPos(pitchingSubsystem, -20));

    }
}

