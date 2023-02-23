package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.LimelightPID.ForwardAndBack;
import frc.robot.commands.LimelightPID.SideToSide;
import frc.robot.subsystems.Swerve;
import frc.util.vision.Limelight;


// Yteam loadButtons
public class RobotButtons {
    public static final Joystick driver = new Joystick(0);
    public static final Joystick systems = new Joystick(1);
    public final Trigger robotCentric = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.3);
    public static final Trigger halfSpeed = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.3);
    public final Trigger zeroGyro = new Trigger(() -> driver.getRawButton(XboxController.Button.kLeftBumper.value));
    public final Trigger SideToSideByLimelight = new Trigger(() -> driver.getRawButton(XboxController.Button.kB.value));
    public final Trigger BackAndForwardByLimelight = new Trigger(() -> driver.getRawButton(XboxController.Button.kX.value));

    public void loadButtons(Swerve swerve, Limelight limelight) {
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> driver.getRawAxis(XboxController.Axis.kLeftY.value), //  left & right
                () -> driver.getRawAxis(XboxController.Axis.kLeftX.value),  // up & down   
                () -> driver.getRawAxis(XboxController.Axis.kRightX.value), // rotation
                () -> robotCentric.getAsBoolean()
            )
        );

        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        SideToSideByLimelight.whileTrue(new SideToSide(limelight, swerve));
        BackAndForwardByLimelight.onTrue(new ForwardAndBack(limelight, swerve));
    }
}