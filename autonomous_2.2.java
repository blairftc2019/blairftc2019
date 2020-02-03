package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.io.*; 
import java.util.*;
import java.util.List;
import java.lang.*;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Gyro_test_Hank", group = "Concept")

public class Gyro_test_Hank extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "ARmXU+X/////AAABmXYp8aIMsULAiUQVZNgKKLA/Zj/4XtslXMZEoS9O4wT7IZTvkZo2CuHMrIsJzLjwOyzh5Ewn4knwCGAbj+M+lW7S3CvYMIW6qzV14weT7WNEi3HkY3lvU5I8o9a3dXWXPKq8MdBlzknTlNsQBIJrWoSgjzV7wV0azxickaTp3hsYEKjOMsVVVGPX6ryCLqbuakRihTFz5FtXwpw1YLkAgawK65bzEfl7YoZ9+XvEBkmi0nBEitTybNsuXIf8heXALLwF/HrCnfgus9JJRZrUziaXlCs6vqd/lPa9tLyjdtWlPxnltzYkKXiFv/v1gnLyWoIFLk+avblo5HLZkNQty3wP3dA6Cz7VcNnDdvj0s6vw";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // Iniialte hardware variables
    
    static BNO055IMU imu;
    static Orientation angles = new Orientation();
    static Acceleration gravity;
    // static double origAngle;
    
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor RL = null;
    private DcMotor RR = null;
    private DcMotor LL = null;
    private DcMotor SR = null;
    private DcMotor SL = null;
    
    double[] error = {0, 0, 0, 0};
    double[] derivative = {0, 0, 0, 0};
    double[] integral = {0, 0, 0, 0};
    double[] previous_error = {0, 0 ,0 ,0};
    double[] setpoint = {0, 0 ,0 ,0};
    double[] drivePower = {0, 0, 0, 0};
    double[] resultPower = {0, 0, 0, 0};
    
    double[] position = {0, 0, 0, 0};
    double origAngle;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double origAngle = angles.firstAngle;

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        LL = hardwareMap.get(DcMotor.class, "LL");
        SL = hardwareMap.get(DcMotor.class, "SL");
        SR = hardwareMap.get(DcMotor.class, "SR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double sweeperPower = 0;

        // double topError = 0;
        // double rightError = 0;
        // double bottomError = 0;
        // double leftError = 0;

        boolean undetected = true;
        
        double topSetpoint = 0;
        double rightSetpoint = 0;
        double bottomSetpoint = 0;
        double leftSetpoint = 0;
        
        // Check pisition is for testing purpose
        // Comment it out in the game
        
        // checkPosition();
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            
            goLeft(0.7, 5000);


            while (opModeIsActive()) {

                // When Tensorflow is enabled
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {

                            if (recognition.getLabel() == "Skystone") {
                                // These variables are doubles
                                position[0] = recognition.getTop();
                                position[1] = recognition.getRight();
                                position[2] = recognition.getBottom();
                                position[3] = recognition.getLeft();
                                
                                setPoint(120, 640, 480, 0);
                                undetected = false;
                            }
                            
                            // Add the error information to the telemetry
                            // telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            // telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            //         position[3], position[0]);
                            // telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            //         position[1], position[2]);
                        }
                    //   telemetry.update();
                    } else if (undetected) {
                        goLeft(0.6, 5000);
                    }
                }
                
                // Keep the Sweeper running for block autonomous
                sweeperPower = 0.4;
                // SL.setPower(sweeperPower);
                // SR.setPower(-sweeperPower);

                PID(setpoint, position);
                motorDrive(drivePower);
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void motorDrive(double[] x) {
        FL.setPower(x[0]);
        FR.setPower(x[1]);
        RL.setPower(x[2]);
        RR.setPower(x[3]);
    }

    public void PID(double[] setpoint, double[] actual) {

        double kP = 0.015;
        double kI = 0.001;
        double kD = 0.005;

        for (int i=0; i<4; i++) {
            error[i] = Math.abs(setpoint[i] - actual[i]);
            integral[i] = error[i] * 0.02;
            derivative[i] = (error[i]-previous_error[i]) / 0.02;
            resultPower[i] =  kP*error[i] + kI*integral[i] + kD*derivative[i];
            previous_error[i] = error[i];
        }
        
        if (Math.abs((error[1]-error[3]))>10) {
            
            /** 
             * The right and the left is inversed here
             * Because when the robot has error on the right, it should go left,
             * so the error calculation has inversed left and right.
             */
            double[] forward = {resultPower[0], resultPower[0], resultPower[0], resultPower[0]};
            double[] right = {-resultPower[1], resultPower[1], resultPower[1], -resultPower[1]};
            double[] bottom = {resultPower[2], resultPower[2], resultPower[2], resultPower[2]};
            double[] left = {resultPower[3], -resultPower[3], -resultPower[3], resultPower[3]};
            
            for (int i=0; i<4; i++) {
                drivePower[i] = Math.tanh(left[i]+right[i]+bottom[i])*0.8;
            }
        } else {
            for (int i=0; i<4; i++) {
                drivePower[i] = 0;
            }
            // double drivePower[] = {0,0,0,0};
        }
        
        telemetry.addData("Error top", error[0]);
        telemetry.addData("Error right", error[1]);
        telemetry.addData("Error bottom", error[2]);
        telemetry.addData("Error left", error[3]);
        telemetry.update();
    }
    
    public void setPoint(double topSetpoint, double rightSetpoint, double bottomSetpoint, double leftSetpoint) {
        setpoint[0] = topSetpoint;
        setpoint[1] = rightSetpoint;
        setpoint[2] = bottomSetpoint;
        setpoint[3] = leftSetpoint;
    }

    public void goLeft(double x, int y) {
        double time = getRuntime();
        while (getRuntime() < time + y) {
            double[] drivePower = {-x, x, x, -x};
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angleError = angles.firstAngle - origAngle;
            if (Math.abs(angleError) != 0) {
                double[] powerDiff = {-angleError, angleError, -angleError, angleError};
                for (int i=0; i<4; i++) {
                    drivePower[i] -= Math.tanh(powerDiff[i])*0.3;
                }
            }
            motorDrive(drivePower);
        }
    }

    public void goRight(double x, int y) {
        FL.setPower(x);
        FR.setPower(-x);
        RL.setPower(-x);
        RR.setPower(x);
        sleep(y);
        FL.setPower(0);
        FR.setPower(0);
        RL.setPower(0);
        RR.setPower(0);
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

    // private void autoCorrect(){
    //     double angleError = 0;
    //     double angleIntegral = 0;
    //     double angleDerivative = 0;
    //     double anglePrevError = 0;

    //     double akP = 0.006;
    //     double akI = 0.005;
    //     double akD = 0.01;

    //     angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
    //     while(Math.abs(angles.firstAngle - origAngle > 3)){
    //         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //         angleError = origAngle - angles.firstAngle;
    //         angleIntegral = angleIntegral + angleError;
    //         angleDerivative = angleError - anglePrevError;
    //         anglePrevError = angleError;

    //         power = error*akP + integral*akI + derivative*akD;
    //         if(power > 1){
    //             power = 1;
    //         }
    //         turnL(power);
    //         telemetry.addData("GyroError", error);
    //         telemetry.addData("power", power);
    //         telemetry.update();
            
    //         sleep(50);
    //     }
    //     telemetry.addData("error", error);
    //     telemetry.addData("power", power);
    //     telemetry.update();
    //     stopRobot();
    // }
    
    public void checkPosition() {
        for (int j=0; j<10000000; j++) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {

                    if (recognition.getLabel() == "Skystone") {
                        // These variables are doubles
                        position[0] = recognition.getTop();
                        position[1] = recognition.getRight();
                        position[2] = recognition.getBottom();
                        position[3] = recognition.getLeft();
                    }
                    
                    // Add the error information to the telemetry
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            position[3], position[0]);
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            position[1], position[2]);
                }
              telemetry.update();
            }
            sleep(10);
        }
    }
}
