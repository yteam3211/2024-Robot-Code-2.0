package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootingOutput;
import frc.robot.commands.ShootingPosition;
import frc.robot.subsystems.ShootingSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick coPilotJoystick = new Joystick(0);
    public final JoystickButton halfSpeed;

    public Trigger High = new Trigger(() -> coPilotJoystick.getPOV() == 0);
    public Trigger Low = new Trigger(() -> coPilotJoystick.getPOV() == 180);
    public Trigger Middle = new Trigger(() -> coPilotJoystick.getPOV() == 270);
    public Trigger shoot = new Trigger(() -> coPilotJoystick.getRawButton(5));
    public Trigger reset = new Trigger(() -> coPilotJoystick.getRawButton(3)); 
    public Trigger pid = new Trigger(() -> coPilotJoystick.getRawAxis(2)>0.3); 
    public Trigger down = new Trigger(() -> coPilotJoystick.getRawAxis(3)>0.3); 
   
    public RobotButtons(Joystick driver) {
        halfSpeed = new JoystickButton(driver, XboxController.Button.kX.value);
    }

    public void loadButtons(ShootingSubsystem shootingSubsystem) {
        shoot.whileTrue(new ShootingOutput(shootingSubsystem, -0.1));
        pid.whileTrue(new ShootingOutput(shootingSubsystem, 0.5));
        down.whileTrue(new ShootingOutput(shootingSubsystem, 1));
        reset.onTrue(new InstantCommand(() -> shootingSubsystem.resetEncoder()));
        High.onTrue(new ShootingPosition( shootingSubsystem,0));
        Low.onTrue(new ShootingPosition( shootingSubsystem,1.5));
        // Middle.onTrue(new ShootingPosition( shootingSubsystem,1.5));
        // load buttons
    }
}