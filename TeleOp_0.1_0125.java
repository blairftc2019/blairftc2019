// TeleOp for Hank's bot
// FTC 2020 Skystone
// 2020 / 1 / 25
// Hank Cui


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp_0.1_0125", group="TeleOp")

public class Hank_teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor RL = null;
    private DcMotor RR = null;
    private DcMotor LL = null;
    private DcMotor SR = null;
    private DcMotor SL = null;

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

            // Setup a variable for each drive wheel to save power level for telemetry
            double FLPower = 0;
            double FRPower = 0;
            double RLPower = 0;
            double RRPower = 0;
            double LLPower = 0;
            double SLPower = 0;
            double SRPower = 0;
            double drive = 0;
            double turn = 0;
            double dpadPower = 0.5;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            
            if (gamepad1.dpad_up) {
                LLPower = -dpadPower;
            } else if (gamepad1.dpad_down) {
                LLPower = dpadPower;
            } else if (gamepad1.left_trigger > 0) {
                drive = -gamepad1.left_stick_y;
                turn  =  gamepad1.right_stick_x;
                SLPower = gamepad1.left_trigger;
                SRPower = -gamepad1.left_trigger;
                FLPower = Range.clip(drive + turn, -1.0, 1.0) ;
                FRPower = Range.clip(drive - turn, -1.0, 1.0) ;
                RLPower = Range.clip(drive + turn, -1.0, 1.0) ;
                RRPower = Range.clip(drive - turn, -1.0, 1.0) ;
            } else if (gamepad1.right_trigger > 0) {
                drive = -gamepad1.left_stick_y;
                turn  =  gamepad1.right_stick_x;
                SLPower = -gamepad1.right_trigger;
                SRPower = gamepad1.right_trigger;
                FLPower = Range.clip(drive + turn, -1.0, 1.0) ;
                FRPower = Range.clip(drive - turn, -1.0, 1.0) ;
                RLPower = Range.clip(drive + turn, -1.0, 1.0) ;
                RRPower = Range.clip(drive - turn, -1.0, 1.0) ;
            } else {
                drive = -gamepad1.left_stick_y;
                turn  =  gamepad1.right_stick_x;
                FLPower = Range.clip(drive + turn, -1.0, 1.0) ;
                FRPower = Range.clip(drive - turn, -1.0, 1.0) ;
                RLPower = Range.clip(drive + turn, -1.0, 1.0) ;
                RRPower = Range.clip(drive - turn, -1.0, 1.0) ;
            }
            
            // Send calculated power to wheels
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            RL.setPower(RLPower);
            RR.setPower(RRPower);
            LL.setPower(LLPower);
            SL.setPower(SLPower);
            SR.setPower(SRPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)", FLPower, FRPower , RLPower, RRPower);
            telemetry.addData("Sweeper", "Lifter (%.2f), sweeper (%.2f)", LLPower, SRPower);
            telemetry.update();
        }
    }
}
