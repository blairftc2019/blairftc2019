package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotDrive{

    // Initialize Motor Variables
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor RL = null;
    public DcMotor RR = null;
    public DcMotor LL = null;
    public DcMotor SR = null;
    public DcMotor SL = null;
    
    // Variables to use IMU's
    public BNO055IMU imu;
    public double origAngle;
    public static Orientation angles;
    public Acceleration gravity;
    
    // Create ttelemetry object
    public static Telemetry telemetry;
    
    public RobotDrive(){
        // Constructor
    }
    
    public void initDrive(HardwareMap hMap) {
        FL = hMap.get(DcMotor.class, "FL");
        FR = hMap.get(DcMotor.class, "FR");
        RL = hMap.get(DcMotor.class, "RL");
        RR = hMap.get(DcMotor.class, "RR");
        LL = hMap.get(DcMotor.class, "LL");
        SL = hMap.get(DcMotor.class, "SL");
        SR = hMap.get(DcMotor.class, "SR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initIMU(HardwareMap hMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);
    }

    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void goLeft(double x, int y) {
        origAngle = getAngle();
        long time = System.currentTimeMillis();

        while (System.currentTimeMillis() < time + y) {
            double[] drivePower = {-x, x, x, -x};
            double angleError = getAngle() - origAngle;

            if (Math.abs(angleError) >= 0.3) {
                double[] powerDiff = {-angleError, angleError, -angleError, angleError};
                for (int i=0; i<4; i++) {
                    drivePower[i] -= Math.tanh(powerDiff[i])*0.3;
                }
            }
            motorDrive(drivePower);
        }
    }

    public void goRight(double x, int y) {
        origAngle = getAngle();
        long time = System.currentTimeMillis();
        
        while (getRuntime() < time + y) {
            double[] drivePower = {x, -x, -x, x};
            double angleError = getAngle() - origAngle;

            if (Math.abs(angleError) != 0) {
                double[] powerDiff = {-angleError, angleError, -angleError, angleError};
                for (int i=0; i<4; i++) {
                    drivePower[i] -= Math.tanh(powerDiff[i])*0.3;
                }
            }
            motorDrive(drivePower);
        }
    }

    public void goForward(double x, int y) {
        FL.setPower(x);
        FR.setPower(x);
        RL.setPower(x);
        RR.setPower(x);
        sleep(y);
        FL.setPower(0);
        FR.setPower(0);
        RL.setPower(0);
        RR.setPower(0);
    }

    public void goBackward(double x, int y) {
        FL.setPower(-x);
        FR.setPower(-x);
        RL.setPower(-x);
        RR.setPower(-x);
        sleep(y);
        FL.setPower(0);
        FR.setPower(0);
        RL.setPower(0);
        RR.setPower(0);
    }
    
    public void raiseArm(double x) {
        LL.setPower(-x);
    }

    public void lowerArm(double x) {
        LL.setPower(x);
    }

    public void raiseArmFor(double x, int y) {
        LL.setPower(-x);
        sleep(y);
        LL.setPower(0);
    }

    public void lowerArmFor(double x, int y) {
        LL.setPower(x);
        sleep(y);
        LL.setPower(0);
    }

    public void sweepIn(double x) {
        SL.setPower(-x);
        SR.setPower(x);
    }

    public void sweepOut(double x) {
        SL.setPower(x);
        SR.setPower(-x);
    }

    public void sweepInFor(double x, int y) {
        SL.setPower(-x);
        SR.setPower(x);
        sleep(y);
        SL.setPower(0);
        SR.setPower(0);
    }

    public void sweepOutFor(double x, int y) {
        SL.setPower(x);
        SR.setPower(-x);
        sleep(y);
        SL.setPower(0);
        SR.setPower(0);
    }

    public void motorDrive(double[] x) {
        FL.setPower(x[0]);
        FR.setPower(x[1]);
        RL.setPower(x[2]);
        RR.setPower(x[3]);
    }
}
