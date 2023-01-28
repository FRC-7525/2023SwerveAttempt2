package frc.robot.geometry_helpers;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class decomposition {
    public static List<Double> DecomposePose3d(Pose3d pose){
        List<Double> tempList = new ArrayList<>();
        tempList.add(pose.getX());
        tempList.add(pose.getY());
        tempList.add(pose.getZ());
        return tempList;
      }
    public static List <Double> Decompose_Rotation_3d(Rotation3d rot) {
      List<Double> temp = new ArrayList<>();
      temp.add(rot.getX());
      temp.add(rot.getY());
      temp.add(rot.getZ());
      return temp;
    }

}

