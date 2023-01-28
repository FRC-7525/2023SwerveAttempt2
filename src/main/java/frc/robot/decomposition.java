package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class decomposition {
    public static Double[] DecomposePose3d(Pose3d pose){
        List<Double> temp = new ArrayList<>();
        temp.add(pose.getX());
        temp.add(pose.getY());
        temp.add(pose.getZ());

        Double[] arr = new Double[temp.size()];
        temp.toArray(arr);
        return arr;
    }

    public static Double[] Decompose_Rotation_3d(Rotation3d rot) {
      List<Double> temp = new ArrayList<>();
      temp.add(rot.getX());
      temp.add(rot.getY());
      temp.add(rot.getZ());

      Double[] arr = new Double[temp.size()];
      temp.toArray(arr);
      return arr;
    }
}

