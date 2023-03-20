package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;

public class OpenIntakeAndArm extends ParallelCommandGroup
{
    
    public OpenIntakeAndArm(CollectSubsystem collectSubsystem,collectWheelsSubsystem collectWheels,armCollectSubsystem armCollect, double wheelsOutput, double centeringOutput, double collectPoint, double armCollectPoint)
    {
        addCommands(
            new setPointCollectCommand(collectSubsystem, collectPoint, armCollect, armCollectPoint),
            new collectWheelsCommand(collectWheels, wheelsOutput, centeringOutput)
            );
    }
}
