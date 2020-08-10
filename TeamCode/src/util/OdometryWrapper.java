package util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Odometry;

public class OdometryWrapper {

    public Odometry odometry;
    private int[] lastOdometryValues = {0, 0, 0};

    public OdometryWrapper (HardwareMap odometryMap) {
        odometry = odometryMap.odometry.get("odometry");
    }

    public int[] calculateOdometryDeltas(){
        int[] wheelCounts = odometry.getAllWheelCounts();
        int[] deltas = {0, 0, 0};
        for (int i = 0; i < wheelCounts.length; i++) {
            deltas[i] = wheelCounts[i] - lastOdometryValues[i];
            lastOdometryValues[i] = wheelCounts[i];
        }
        return deltas;
    }

    public void resetOdometry() {
        odometry.reset();
    }

}
