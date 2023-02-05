package frc.robot;

import javax.swing.GroupLayout.Group;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootingOutput;
import frc.robot.commands.ShootingPosition;
import frc.robot.commands.shootingCommandGroup;
import frc.robot.commands.simpleOutputCommand;
import frc.robot.subsystems.ShootingSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick coPilotJoystick = new Joystick(0);
    public final JoystickButton halfSpeed;

    public Trigger High = new Trigger(() -> coPilotJoystick.getPOV() == 0);
    public Trigger Low = new Trigger(() -> coPilotJoystick.getPOV() == 180);
    public Trigger Group = new Trigger(() -> coPilotJoystick.getPOV() == 270);
    public Trigger back = new Trigger(() -> coPilotJoystick.getRawButton(5));
    public Trigger reset = new Trigger(() -> coPilotJoystick.getRawButton(3)); 
    public Trigger middle = new Trigger(() -> coPilotJoystick.getRawAxis(2)>0.3); 
    public Trigger down = new Trigger(() -> coPilotJoystick.getRawAxis(3)>0.3); 
   
    public RobotButtons(Joystick driver) {
        halfSpeed = new JoystickButton(driver, XboxController.Button.kX.value);
    }

    public void loadButtons(ShootingSubsystem shootingSubsystem) {
        back.whileTrue(new simpleOutputCommand(shootingSubsystem, -0.2));
        down.whileTrue(new simpleOutputCommand(shootingSubsystem, 0.4));
        middle.whileTrue(new simpleOutputCommand(shootingSubsystem, 0.6));
        reset.onTrue(new InstantCommand(() -> shootingSubsystem.resetEncoder()));
        // High.onTrue(new ShootingPosition( shootingSubsystem,0));
        // Low.onTrue(new ShootingPosition( shootingSubsystem,1.5));
        // Group.onTrue(new shootingCommandGroup(shootingSubsystem, 0, 0));
        // load buttons
    }
}