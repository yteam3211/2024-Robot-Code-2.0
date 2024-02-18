package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShootingCommands.CompleteSpeakerShootingCommand;
import frc.robot.commands.SwereCommands.LockWheelsCommnad;
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
    private final SendableChooser<Command> autoChooser;
    
    private final AllianceSpecs allianceSpecs = new AllianceSpecs(limelight);
    private final ShootingMath shootingMath = new ShootingMath(swerve, elevatorSubsystem, pitchingSubsystem, limelight);

    //auto commands register
    public RobotContainer() {

    // NamedCommands.registerCommand("Turn Command", new TurnToShootingCommand(swerve, limelight, Constants.SHOOTING_ANGLE_TRESHOLD));
    // NamedCommands.registerCommand("Shooting Command", new CompleteShootingCommand(swerve, limelight, shootingSubsystem, pitchingSubsystem));
    // NamedCommands.registerCommand("Open Intake Command", new IntakeCommand(intakeSubsystem, Constants.INTAKE_OPEN_POSITION, Constants.INTAKE_WHEELS_OUTPUT));
    // NamedCommands.registerCommand("Close Intake Command", new IntakeCommand(intakeSubsystem, Constants.INTAKE_OPEN_POSITION, 0));

        // Configure the button bindings
        configureButtonBindings();
        
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        return autoChooser.getSelected();
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