package frc.robot.autos.GRIDmovement;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Right2GRID {
    public static Command getAutoCommand(Swerve driveSubsystem) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 right GRID", new PathConstraints(1.6, 3));
        
        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("intakeDown", new IntakeDown());

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose, // TODO ckack if it work 
            driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        return autoBuilder.fullAuto(pathGroup);
    }
}

