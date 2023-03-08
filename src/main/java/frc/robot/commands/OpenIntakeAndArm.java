package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.collectWheels;

public class OpenIntakeAndArm extends ParallelCommandGroup
{
    
    public OpenIntakeAndArm(CollectSubsystem collectSubsystem,collectWheels collectWheels,armCollectSubsystem armCollect, double wheelsOutput, double centeringOutput, double collectPoint, double armCollectPoint)
    {
        addCommands(
            new setPointCollectCommand(collectSubsystem, collectPoint, armCollect, armCollectPoint),
            new collectWheelsCommand(collectWheels, wheelsOutput, centeringOutput)
            );
    }
}
