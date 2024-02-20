package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.vision.Limelight;

public class AllianceSpecs {
    public static boolean isRed;
    public static Pose2d speakerPos;
    public static Pose2d AMPPose;
    public static DoubleSupplier poseY;
    public static DoubleSupplier poseX;
    
  
    public AllianceSpecs(Limelight limelight) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            isRed = true;
            speakerPos = Constants.RED_SPEAKER_POS;
            poseX = () -> limelight.getBlueBotpose()[0];//limelight.getRedBotpose()[0];
            poseY = () -> limelight.getBlueBotpose()[1];//limelight.getRedBotpose()[1];
            
        }
        else{
            isRed = false;
            speakerPos = Constants.BLUE_SPEAKER_POS;
            poseX = () -> limelight.getBlueBotpose()[0];
            poseY = () -> limelight.getBlueBotpose()[1];
        }
    }
}