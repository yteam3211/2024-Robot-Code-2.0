package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.armCollectSubsystem;
import frc.robot.subsystems.collectWheelsSubsystem;

public class closeArmWithDelay extends SequentialCommandGroup
{
    
    public closeArmWithDelay(armCollectSubsystem armCollect,  double collectPoint, double armCollectPoint)
    {
        addCommands(
            new WaitCommand(1),
            new ArmCollectCommand(armCollect, collectPoint, armCollectPoint)
            );
    }
}
