package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CollectSubsyste;
import frc.robot.subsystems.collectWheels;

public class collectGroupCommand extends ParallelCommandGroup
{
    public collectGroupCommand(CollectSubsyste collectSubsyste,collectWheels collectWheels, double wheelsOutput, double centeringOutput, double point)
    {
        addCommands(
            new setPointCollectCommand(collectSubsyste, point),
            new collectOutput(collectWheels, wheelsOutput, centeringOutput)
            );
    }
}
