package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShootingSubsystem;

public class shootingCommandGroup extends SequentialCommandGroup
{
    public shootingCommandGroup(ShootingSubsystem subsystem, double output, double position)
    {
        addCommands(
            new ShootingOutput(subsystem, output),
            new ShootingPosition(subsystem, position),
            new reSetCommand(subsystem)
            );
    }
}
