package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommands.AutoIntakeGroupCommand;
import frc.robot.commands.AutoCommands.AutoKickerCommand;
import frc.robot.commands.AutoCommands.AutoShooingWheels;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.IntakePos;
import frc.robot.commands.ShootingCommands.CompleteSpeakerShootingCommand;
import frc.robot.commands.ShootingCommands.PitchPos;
import frc.robot.commands.SwereCommands.LockWheelsCommand;
import frc.robot.commands.SwereCommands.TurnToShootingCommand;
import frc.robot.subsystems.*;
import frc.util.vision.Limelight;
import frc.util.vision.Limelight.limelightStreamMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
    public static boolean isRed;
    private final RobotButtons robotButtons = new RobotButtons();
    
    /* Subsystems */
    private final Limelight limelight = new Limelight.Builder().build();
    private final Swerve swerve = new Swerve(limelight);
    private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final PitchingSubsystem pitchingSubsystem = new PitchingSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final TransferSubsystem transferSubsystem = new TransferSubsystem();
    private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
    // private final SendableChooser<Command> autoChooser;
    
    private final AllianceSpecs allianceSpecs = new AllianceSpecs(limelight);
    private final ShootingMath shootingMath = new ShootingMath(swerve, elevatorSubsystem, pitchingSubsystem, limelight);

    //auto commands register
    public RobotContainer() {

    NamedCommands.registerCommand("Open Intake Command", new AutoIntakeGroupCommand(intakeSubsystem, transferSubsystem, shootingSubsystem, kickerSubsystem, pitchingSubsystem));
    NamedCommands.registerCommand("Start Intake Command", new IntakePos(intakeSubsystem, Constants.INTAKE_OPEN_POSITION));
    NamedCommands.registerCommand("Start pitch", new PitchPos(pitchingSubsystem, 54));
    NamedCommands.registerCommand("Close 1 pitch", new PitchPos(pitchingSubsystem,  19));
    NamedCommands.registerCommand("Kicker", new AutoKickerCommand(kickerSubsystem, shootingSubsystem, Constants.KICKER_OUTPUT));
        
        // Register Named Commands // Need to put right commands and right subsystems
        // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
        // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
        // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

        // Configure the button bindings
        configureButtonBindings();
        
        // Build an auto chooser. This will use Commands.none() as the default option.
        // autoChooser = AutoBuilder.buildAutoChooser("3 MA");
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        robotButtons.loadButtons(swerve, limelight, shootingSubsystem, pitchingSubsystem, intakeSubsystem, transferSubsystem, elevatorSubsystem, kickerSubsystem, shootingMath);
    }

    public void SetOutputToZero(){
        shootingSubsystem.setShooterOutput(0);
        intakeSubsystem.setIntakeOpenMotorUotput(0);
        intakeSubsystem.setWheelsMotorOutput(0);
        transferSubsystem.setOutput(0);
        kickerSubsystem.setKickerOutput(0);
        pitchingSubsystem.SetOutput(0);
        elevatorSubsystem.setOutput(0); 
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto("collecting first ring and shotting it");
        // return autoChooser.getSelected();
    }

    // gets & sets 
    public Limelight getLimelight() {
        return limelight;
    }

    public Swerve getSwerve() {
        return swerve;
    }

    public RobotButtons getRobotButtons() {
        return robotButtons;
    }
        
    public AllianceSpecs getAllianceSpecs() {
        return allianceSpecs;
    }

    public ShootingSubsystem getShootingSubsystem() {
        return shootingSubsystem;
    }

    public PitchingSubsystem getPitchingSubsystem() {
        return pitchingSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public TransferSubsystem getTransferSubsystem() {
        return transferSubsystem;
    }

    public KickerSubsystem getKickerSubsystem() {
        return kickerSubsystem;
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }
        public ShootingMath getShootingMath() {
        return shootingMath;
    }
}