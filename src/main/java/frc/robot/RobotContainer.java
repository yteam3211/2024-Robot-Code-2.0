package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.ShootingCommnads.CartridgeOutputCommand;
import frc.robot.commands.ShootingCommnads.ShootingGroupCommand;
import frc.robot.commands.autoCommands.FarFromHumanCube;
import frc.robot.commands.autoCommands.Next2Human3Cubes;
import frc.robot.commands.autoCommands.Next2HumanCommand;
import frc.robot.commands.autoCommands.Center3Cubes;
import frc.robot.commands.autoCommands.CenterCloseToHumanCube;
import frc.robot.commands.autoCommands.CenterFarFromHumanCube;
import frc.robot.commands.timercommand.collectAtuoCommand;
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
    /* Controllers */
    private RobotButtons robotButtons = new RobotButtons();
    public static final Limelight limelight = new Limelight.Builder().build();
    /* Drive Controls */


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final collectWheelsSubsystem m_CollectWheels = new collectWheelsSubsystem();
    private final shootingSubsystem  m_ShootingSubsystem = new shootingSubsystem();
    private final CollectSubsystem m_CollectSubsystem = new CollectSubsystem();
    private final armCollectSubsystem m_armCollectSubsystem = new armCollectSubsystem();
    private final CartridgeSubsystem m_cartridgeSubsystem = new CartridgeSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public final CenterFarFromHumanCube CenterFarFromHumanCube = new CenterFarFromHumanCube(s_Swerve,  m_CollectSubsystem, m_cartridgeSubsystem,
     m_CollectWheels, m_ShootingSubsystem, m_armCollectSubsystem);
    public final CenterCloseToHumanCube centerCloseToHumanCube = new CenterCloseToHumanCube(s_Swerve,  m_CollectSubsystem, m_cartridgeSubsystem, m_CollectWheels, m_ShootingSubsystem, m_armCollectSubsystem);
    public final Center3Cubes center3Cubes = new Center3Cubes(s_Swerve, m_CollectSubsystem, m_cartridgeSubsystem, m_CollectWheels, m_ShootingSubsystem, m_armCollectSubsystem);
    
    public final Next2HumanCommand next2Human = new Next2HumanCommand(s_Swerve, m_CollectSubsystem, m_cartridgeSubsystem,
     m_CollectWheels, m_armCollectSubsystem, limelight, m_ShootingSubsystem);
    public final Next2Human3Cubes next2Human3Cubes = new Next2Human3Cubes(s_Swerve, m_CollectSubsystem, m_cartridgeSubsystem, m_CollectWheels, m_armCollectSubsystem, m_ShootingSubsystem);
     
    public final FarFromHumanCube farFromHumanCube = new FarFromHumanCube(s_Swerve, m_CollectSubsystem, m_cartridgeSubsystem,
     m_CollectWheels, m_armCollectSubsystem, limelight, m_ShootingSubsystem);
    
    public final ShootingGroupCommand justShoot = new ShootingGroupCommand(m_ShootingSubsystem, m_armCollectSubsystem, m_cartridgeSubsystem , Constants.SHOOTING_HIGH);
     
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();
        
    }
    
    private void configureSwerveButtons() {
        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        configureSwerveButtons();
        robotButtons.loadButtons(m_ShootingSubsystem, m_CollectSubsystem,  s_Swerve, m_CollectWheels, limelight, m_armCollectSubsystem, m_cartridgeSubsystem);
    }
 
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null; // atuo;
    }

    // gets & sets 
    public Swerve getS_Swerve() {
        return s_Swerve;
    }

    public shootingSubsystem getM_ShootingSubsystem() {
        return m_ShootingSubsystem;
    }

    public CollectSubsystem getM_CollectSubsyste() {
        return m_CollectSubsystem;
    }

    public CartridgeSubsystem getM_CartridgeSubsystem(){
        return m_cartridgeSubsystem;
    }
    public RobotButtons getRobotButtons() {
        return robotButtons;
    }
    public void setRobotButtons(RobotButtons robotButtons) {
        this.robotButtons = robotButtons;
    }
    public static Limelight getLimelight() {
        return limelight;
    }
    public collectWheelsSubsystem getM_CollectWheels() {
        return m_CollectWheels;
    }
    public CollectSubsystem getM_CollectSubsystem() {
        return m_CollectSubsystem;
    }
    public armCollectSubsystem getM_armCollectSubsystem() {
        return m_armCollectSubsystem;
    }
    public CartridgeSubsystem getM_cartridgeSubsystem() {
        return m_cartridgeSubsystem;
    }
    public CenterCloseToHumanCube getCenterCloseToHumanCube() {
        return centerCloseToHumanCube;
    }
    public Center3Cubes getCenter3Cubes() {
        return center3Cubes;
    }
    public Next2Human3Cubes getNext2Human3Cubes() {
        return next2Human3Cubes;
    }
    public ShootingGroupCommand getJustShoot() {
        return justShoot;
    }
    public CenterFarFromHumanCube getCenterFarFromHumanCube(){
        return CenterFarFromHumanCube;
    }
    public ShootingGroupCommand getJustShootAtuo(){
        return justShoot;
    }
    public Next2HumanCommand getNext2Human() {
        return next2Human;
    }
    public FarFromHumanCube getFarFromHumanCube(){
        return farFromHumanCube;
    }
    public Command getTest(){
       return AutoCommand.getAutoCommand(s_Swerve, "test", 3);
    }
}
 