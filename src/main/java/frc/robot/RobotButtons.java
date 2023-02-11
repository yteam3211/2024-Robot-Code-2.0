package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.armOutput;
import frc.robot.commands.armPosition;
import frc.robot.commands.resetCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick driver = new Joystick(1);
    public static Joystick systems = new Joystick(0);
    public final Trigger robotCentric = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public final Trigger halfSpeed = new Trigger(() -> driver.getRawButton(XboxController.Button.kX.value));
    private final Trigger zeroGyro = new Trigger(() -> driver.getRawButton(XboxController.Button.kY.value));
    public Trigger arm2 = new Trigger(() -> systems.getRawButton(4));
    public Trigger arm3 = new Trigger(() -> systems.getRawButton(3));
    public Trigger arm5 = new Trigger(() -> systems.getRawButton(2));
    public Trigger reset = new Trigger(() -> systems.getRawButton(6));
    public Trigger armcolose = new Trigger(() -> systems.getRawAxis(2)>0.3);



    public void loadButtons(Swerve swerve, armSubsystem arm ) {
        // swerve.setDefaultCommand(
            // new TeleopSwerve(
                // swerve, 
                // () -> driver.getRawAxis(XboxController.Axis.kLeftY.value), // up & down
                // () -> driver.getRawAxis(0), // up & down
                // // () -> driver.getRawAxis(XboxController.Axis.kLeftX.value),  // left & right 
                // () -> driver.getRawAxis(1),  // left & right 
                // () -> driver.getRawAxis(XboxController.Axis.kRightX.value), // rotation
                // () -> robotCentric.getAsBoolean()
            // )
        // );

        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        // arm2.whileTrue(new armPosition(arm, 6.2));
        // armcolose.onTrue(new armPosition(arm, 0));

        // // armcolose.whileTrue(new armOutput(arm, 1));
        // reset.whileTrue(new resetCommand(arm));
    }
}