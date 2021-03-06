package util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMU{
    BNO055IMU imu;
    private double imuBias = 0;
    private double calibration = 0;

    public IMU(HardwareMap theMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "util.IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = theMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void setImuBias(double vuforiaHeading){
        imuBias = util.Functions.normalizeAngle(vuforiaHeading - getHeading()); // -180 to 180 deg
    }
    public double getHeading(){
        double heading = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ).thirdAngle - calibration;
        heading = util.Functions.normalizeAngle(heading); // -180 to 180 deg
        return heading;
    }

    public double getImuBias() {
        return imuBias;
    }

}
