package frc.robot;

import javax.swing.GroupLayout.Group;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootingOutput;
import frc.robot.commands.ShootingPosition;
import frc.robot.commands.collectCommand;
import frc.robot.commands.collectOutput;
import frc.robot.commands.setPoitCollectCommand;
import frc.robot.commands.shootingCommandGroup;
import frc.robot.commands.simpleOutputCommand;
import frc.robot.subsystems.CollectSubsyste;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.armSubsystem;


// Yteam loadButtons
public class RobotButtons {
    public static Joystick coPilotJoystick = new Joystick(0);
    public static Joystick driver = new Joystick(1);
    public final JoystickButton halfSpeed;

    public Trigger collectOpen = new Trigger(() -> coPilotJoystick.getRawButton(5)); 
    public Trigger down = new Trigger(() -> coPilotJoystick.getRawAxis(3)>0.3); 

   
    public RobotButtons(Joystick driver) {
        halfSpeed = new JoystickButton(driver, XboxController.Button.kX.value);
    }

    public void loadButtons(ShootingSubsystem shootingSubsystem, CollectSubsyste collectSubsyste, armSubsystem armSubsystem) {
        collectOpen.onFalse(new setPoitCollectCommand(collectSubsyste, 0));
        collectOpen.onTrue(new setPoitCollectCommand(collectSubsyste, -1000));
        // load buttons
    }
}










