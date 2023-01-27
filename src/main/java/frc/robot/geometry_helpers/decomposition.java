package frc.robot.geometry_helpers;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;

public class decomposition {
    public static List<Double> DecomposePose3d(Pose3d pose){
        List<Double> tempList = new ArrayList<>();
        tempList.add(pose.getX());
        tempList.add(pose.getY());
        tempList.add(pose.getZ());
        return tempList;
      }
    
}
