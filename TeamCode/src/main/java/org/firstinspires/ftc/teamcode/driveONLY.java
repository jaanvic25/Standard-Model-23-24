
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="driving TEST")
//@Disabled
public class driveONLY extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //    port 0
    private DcMotor leftFront = null;
    //    port 3
    private DcMotor leftBack = null;
    //    port 1
    private DcMotor rightFront = null;
    //    port 2
    private DcMotor rightBack = null;

    private DcMotor liftMotorL = null; //left
    private DcMotor liftMotorR = null; //right

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront"); //xr odometry
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront"); //xl odometry
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //y odometry

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorR = hardwareMap.get(DcMotor.class, "liftMotorR");
        liftMotorL = hardwareMap.get(DcMotor.class, "liftMotorL");

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorR.setDirection(DcMotor.Direction.FORWARD);
        liftMotorL.setDirection(DcMotor.Direction.REVERSE);


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//
//        servo_claw = hardwareMap.servo.get("servo_claw");
//        servo_claw_pos = SERVO_CLAW_INIT;
//
//        servo_claw.setPosition(servo_claw_pos);
//
//


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        final double JOYSTICK_SEN = .007;
        final double JOYSTICK_SENTEST = .005; //made really low might not work

        double liftPower;

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        // if mathabs < joystick -> (?) 0 else (:) set to leftstick

        double lx = Math.abs(gamepad1.left_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.left_stick_x;
        //lx is turning
        double rx = Math.abs(gamepad1.right_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_x;
        //rx is strafing
        double ry = Math.abs(gamepad1.right_stick_y)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_y;
        //llx
        // Choose to drive using either Tank Mode, or POV Mode

        double rry = Math.abs(gamepad2.right_stick_y)< JOYSTICK_SENTEST ? 0 : gamepad2.right_stick_y;

        liftPower = Range.clip(rry, -1.0, 1.0);
        liftMotorR.setPower(liftPower);
        liftMotorL.setPower(liftPower);


        leftBackPower    = Range.clip(-lx - rx - ry, -1.0, 1.0);
        leftFrontPower    = Range.clip(-lx + rx - ry, -1.0, 1.0);
        rightFrontPower   = Range.clip(-lx + rx + ry, -1.0, 1.0);
        rightBackPower   = Range.clip(-lx - rx + ry, -1.0, 1.0) ;

        double maxSpeed =0.55;
        leftFront.setPower(leftFrontPower*maxSpeed);
        leftBack.setPower(leftBackPower*maxSpeed);
        rightFront.setPower(rightFrontPower*maxSpeed);
        rightBack.setPower(rightBackPower*maxSpeed);
//        if (gamepad1.x) {
//            servo_claw_pos = SERVO_CLAW_INIT;
//        }
//        if (gamepad1.b) {
//            servo_claw_pos = SERVO_CLAW_GRAB;
//        }
//
//        servo_claw.setPosition(servo_claw_pos);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("lf encoder", leftFront.getCurrentPosition());
        telemetry.addData("rf encoder", rightFront.getCurrentPosition());
        telemetry.addData("lb encoder", leftBack.getCurrentPosition());
        telemetry.addData("rb encoder", rightBack.getCurrentPosition());
//        telemetry.addData("servo", "gamepad1.x")
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
