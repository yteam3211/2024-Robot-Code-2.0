package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.collectWheels;

public class collectGroupCommand extends ParallelCommandGroup
{
    public collectGroupCommand(CollectSubsystem collectSubsystem,collectWheels collectWheels, double wheelsOutput, double centeringOutput, double point)
    {
        addCommands(
            new setPointCollectCommand(collectSubsystem, point),
            new collectOutput(collectWheels, wheelsOutput, centeringOutput)
            );
    }
}
