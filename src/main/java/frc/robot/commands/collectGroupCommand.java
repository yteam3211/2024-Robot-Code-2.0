package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.collectWheels;

public class collectGroupCommand extends ParallelCommandGroup
{
    
    public collectGroupCommand(CollectSubsystem collectSubsystem,collectWheels collectWheels,armCollectSubsystem armCollect, double wheelsOutput, double centeringOutput, double point,double position)
    {
        addCommands(
            new setPointCollectCommand(collectSubsystem, point),
            new collectWheelsCommand(collectWheels, wheelsOutput, centeringOutput),
            new ArmCollectCommand(armCollect, position, 200)
            );
    }
}
