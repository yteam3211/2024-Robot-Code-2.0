package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


// Yteam loadButtons
public class RobotButtons {

    public RobotButtons(Joystick driver) {
        Trigger halfSpeed = new Trigger(() -> driver.getRawButton(XboxController.Button.kX.value));
    }

    public void loadButtons() {
        // load buttons
    }
}