// TeleOp for Hank's bot
// Single Driver
// FTC 2020 Skystone
// 2020 / 1 / 27
// Hank Cui


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp_Single_Person", group="TeleOp")

public class TeleOp_Single_Person extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor RL = null;
    private DcMotor RR = null;
    private DcMotor LL = null;
    private DcMotor SR = null;
    private DcMotor SL = null;
    int reverse = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        LL = hardwareMap.get(DcMotor.class, "LL");
        SL = hardwareMap.get(DcMotor.class, "SL");
        SR = hardwareMap.get(DcMotor.class, "SR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Driving
            double leftstickx = 0;
            double leftsticky = 0;
            double rightstickx = 0;
            double wheelpower;
            double stickangleradians;
            double rightX;
            double leftfrontpower;
            double rightfrontpower;
            double leftrearpower;
            double rightrearpower;
            double dpadpower = .25;

            double liftPower = 0;
            double slPower = 0;
            double srPower = 0;

            if (gamepad1.dpad_up) {
                liftPower = -dpadpower*2;
            } else if (gamepad1.dpad_right) {
                leftstickx = dpadpower;
            } else if (gamepad1.dpad_down) {
                liftPower = dpadpower*2;
            } else if (gamepad1.dpad_left) {
                leftstickx = -dpadpower;
            } else if (gamepad1.right_trigger > 0.1) {
                slPower = -gamepad1.right_trigger;
                srPower = gamepad1.right_trigger;
                leftstickx = -gamepad1.left_stick_x;
                leftsticky = gamepad1.left_stick_y;
                rightstickx = gamepad1.right_stick_x;
            } else if (gamepad1.left_trigger > 0.1) {
                slPower = gamepad1.left_trigger;
                srPower = -gamepad1.left_trigger;
                leftstickx = -gamepad1.left_stick_x;
                leftsticky = gamepad1.left_stick_y;
                rightstickx = gamepad1.right_stick_x;
            } else {
                leftstickx = -gamepad1.left_stick_x;
                leftsticky = gamepad1.left_stick_y;
                rightstickx = -gamepad1.right_stick_x;
            }
            wheelpower = Math.hypot(leftstickx, leftsticky);
            stickangleradians = Math.atan2(leftsticky, leftstickx);

            stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

            rightX = rightstickx * .5 * reverse;
            leftfrontpower = (wheelpower * Math.cos(stickangleradians) + rightX);
            rightfrontpower = (wheelpower * Math.sin(stickangleradians) - rightX);
            leftrearpower = (wheelpower * Math.sin(stickangleradians) + rightX);
            rightrearpower = (wheelpower * Math.cos(stickangleradians) - rightX);

            FL.setPower(leftfrontpower);
            FR.setPower(rightfrontpower);
            RL.setPower(leftrearpower);
            RR.setPower(rightrearpower);

            LL.setPower(liftPower);
            SL.setPower(slPower);
            SR.setPower(srPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)", leftfrontpower, rightfrontpower, leftrearpower, rightrearpower);
            telemetry.update();
        }
    }
}
