package frc.robot;

import javax.swing.GroupLayout.Group;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootingOutput;
import frc.robot.commands.ShootingPosition;
import frc.robot.commands.collectCommand;
import frc.robot.commands.collectOutput;
import frc.robot.commands.reSetCommand;
import frc.robot.commands.setPoitCollectCommand;
import frc.robot.commands.shootingCommandGroup;
import frc.robot.commands.simpleOutputCommand;
import frc.robot.subsystems.CollectSubsyste;
import frc.robot.subsystems.ShootingSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick coPilotJoystick = new Joystick(0);
    public final JoystickButton halfSpeed;

    public Trigger High = new Trigger(() -> coPilotJoystick.getPOV() == 270);
    public Trigger Low = new Trigger(() -> coPilotJoystick.getPOV() == 180);
    public Trigger Group = new Trigger(() -> coPilotJoystick.getPOV()== 0);
    public Trigger back = new Trigger(() -> coPilotJoystick.getRawButton(6));
    public Trigger collectClose = new Trigger(() -> coPilotJoystick.getRawButton(5));
    public Trigger reset = new Trigger(() -> coPilotJoystick.getRawButton(3)); 
    public Trigger middle = new Trigger(() -> coPilotJoystick.getRawAxis(2)>0.3); 
    public Trigger down = new Trigger(() -> coPilotJoystick.getRawAxis(3)>0.3); 

   
    public RobotButtons(Joystick driver) {
        halfSpeed = new JoystickButton(driver, XboxController.Button.kX.value);
    }

    public void loadButtons(ShootingSubsystem shootingSubsystem, CollectSubsyste collectSubsyste) {
        back.whileTrue(new simpleOutputCommand(shootingSubsystem, 0.4));
        middle.whileTrue(new collectOutput(collectSubsyste, 0.5,0.3));   
        // collectClose.onTrue(new ShootingOutput(shootingSubsystem, 0.4));
        reset.onTrue(new reSetCommand(shootingSubsystem,collectSubsyste));
        collectClose.onFalse(new setPoitCollectCommand(collectSubsyste, 0));
        collectClose.onTrue(new setPoitCollectCommand(collectSubsyste, -1000));
        // High.onTrue(new ShootingPosition( shootingSubsystem,0));
        // Low.onTrue(new ShootingPosition( shootingSubsystem,5));
        // load buttons
    }
}










