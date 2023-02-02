package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootingOutput;
import frc.robot.subsystems.ShootingSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick coPilotJoystick = new Joystick(0);
    public final JoystickButton halfSpeed;

    public Trigger High = new Trigger(() -> coPilotJoystick.getPOV() == 90);
    public Trigger Low = new Trigger(() -> coPilotJoystick.getPOV() == 180);
    public Trigger Middle = new Trigger(() -> coPilotJoystick.getPOV() == 270);
    public Trigger shoot = new Trigger(() -> coPilotJoystick.getPOV() == 0);
   
    public RobotButtons(Joystick driver) {
        halfSpeed = new JoystickButton(driver, XboxController.Button.kX.value);
    }

    public void loadButtons(ShootingSubsystem shootingSubsystem) {
        shoot.whileTrue(new ShootingOutput(shootingSubsystem, 1));
        Low.whileTrue(new ShootingOutput(shootingSubsystem, -0.2));
        // High.onTrue(new Shootingcommand(shootingSubsystem, 0, null));
        // Low.onTrue(new Shootingcommand(shootingSubsystem, 0, null));
        // Middle.onTrue(new Shootingcommand(shootingSubsystem, 0, null));
        // load buttons
    }
}