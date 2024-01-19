/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

@TeleOp(name="iterative TEST")
//@Disabled
public class iterativeTEST extends OpMode
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
    //    expansion hub port 0
    private DcMotor liftMotorT = null; //turn
    //    expansion hub port 1
    private DcMotor liftMotorE = null; //extend
    //port 0 of expansion servo
    Servo servo_plane;
    double servo_plane_pos;
    //port 1
    Servo servo_door_r;
    double servo_door_rpos;
    //port 3
    Servo servo_door_l;
    double servo_door_lpos;
    //port 2
    CRServo servo_in_r;
    double servo_in_rpow;
    //port 5
    CRServo servo_in_l;
    double servo_in_lpow;
    //port 0
    Servo servo_spin_r;
    double servo_spin_rpos;
    //port 4
    Servo servo_spin_l;
    double servo_spin_lpos;

    //TODO: fix errors

    //in
    static final double SERVO_DOOR_RINIT = .75;
    //out
    static final double SERVO_DOOR_RGRAB = .27;

    //in
    static final double SERVO_DOOR_LINIT = .23;
    //out
    static final double SERVO_DOOR_LGRAB = .64;

    //in
    static final double SERVO_PLANE_INIT = .4;
    //out
    static final double SERVO_PLANE_OUT = 0;

    static final double SERVO_SPIN_RUP = .99;
    static final double SERVO_SPIN_LUP = .5;

    static final double SERVO_SPIN_RFLAT = .3;
    static final double SERVO_SPIN_LFLAT = .8;

    static final double SERVO_STILL = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront"); //xr odometry
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront"); //xl odometry
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //y odometry
        liftMotorT = hardwareMap.get(DcMotor.class, "liftMotorT");
        liftMotorE = hardwareMap.get(DcMotor.class, "liftMotorE");

        servo_door_r = hardwareMap.servo.get("servoDoorR");
        servo_door_rpos = SERVO_DOOR_RGRAB;
        servo_door_r.setPosition(servo_door_rpos);

        servo_door_l = hardwareMap.servo.get("servoDoorL");
        servo_door_lpos = SERVO_DOOR_LGRAB;
        servo_door_l.setPosition(servo_door_lpos);

        servo_in_r = hardwareMap.crservo.get("servoInR");
        servo_in_rpow = SERVO_STILL;
        servo_in_r.setPower(servo_in_rpow);

        servo_in_l = hardwareMap.crservo.get("servoInL");
        servo_in_lpow = SERVO_STILL;
        servo_in_l.setPower(servo_in_lpow);

        servo_spin_r = hardwareMap.servo.get("servoSpinR");
        servo_spin_r.setDirection(Servo.Direction.REVERSE);
        servo_spin_rpos = SERVO_SPIN_RFLAT;
        servo_spin_r.setPosition(servo_spin_rpos);

        servo_spin_l = hardwareMap.servo.get("servoSpinL");
        servo_spin_lpos = SERVO_SPIN_LFLAT;
        servo_spin_l.setPosition(servo_spin_lpos);

        servo_plane = hardwareMap.servo.get("servo_plane");
        servo_plane_pos = SERVO_PLANE_INIT;
        servo_plane.setPosition(servo_plane_pos);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        liftMotorT.setDirection(DcMotor.Direction.FORWARD);
        liftMotorE.setDirection(DcMotor.Direction.FORWARD);
//        drone1.setDirection(DcMotor.Direction.FORWARD);
//        drone2.setDirection(DcMotor.Direction.FORWARD);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        drone1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        drone2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        liftMotor.setTargetPosition(0);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double liftPowerT;
        double liftPowerE;

        final double JOYSTICK_SEN = .007;
        final double JOYSTICK_SENTEST = .005; //made really low might not work
        // if mathabs < joystick -> (?) 0 else (:) set to leftstick

        double lx = Math.abs(gamepad1.left_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.left_stick_x;
        //lx is turning
        double rx = Math.abs(gamepad1.right_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_x;
        //rx is strafing
        double ry = Math.abs(gamepad1.right_stick_y)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_y;
        //llx
        double rrx = Math.abs(gamepad2.left_stick_x)< JOYSTICK_SENTEST ? 0 : gamepad2.left_stick_x;
        //^^turn
        double rry = Math.abs(gamepad2.right_stick_y)< JOYSTICK_SENTEST ? 0 : gamepad2.right_stick_y;
        //^^extend

        //front and back
        //two motors spinning very fast opp directions
        //two servos for claw
        if (gamepad2.x) {
            telemetry.addData("gamepadx", "init");
            servo_door_rpos = SERVO_DOOR_RINIT;
            servo_door_lpos = SERVO_DOOR_LINIT;
        }
        if (gamepad2.b) {
            servo_door_rpos = SERVO_DOOR_RGRAB;
            servo_door_lpos = SERVO_DOOR_LGRAB;
            telemetry.addData("gamepadb", "grab");
        }
        if (gamepad2.a) {
            servo_plane_pos = SERVO_PLANE_OUT;
            telemetry.addData("gamepadb", "planein");
        }
        if (gamepad2.y) {
            servo_plane_pos = SERVO_PLANE_INIT;
            telemetry.addData("gamepady", "planeout");
        }

        if (gamepad1.a) {
            telemetry.addData("gamepada", "toggle intake");

            servo_in_rpow = -.7;
            servo_in_lpow = .7;

        }

        if (gamepad1.y) {
            telemetry.addData("gamepadb", servo_spin_rpos);
            servo_spin_rpos = SERVO_SPIN_RFLAT;
            servo_spin_lpos = SERVO_SPIN_LFLAT;
        }

        if (gamepad1.x) {
            telemetry.addData("gamepadx", servo_spin_rpos);
            servo_spin_rpos = SERVO_SPIN_RUP;
            servo_spin_lpos = SERVO_SPIN_LUP;
        }

        if (gamepad1.b) {
            telemetry.addData("gamepadx", "spinning other way?");
            servo_in_rpow = SERVO_STILL;
            servo_in_lpow = SERVO_STILL;
        }

        servo_door_r.setPosition(servo_door_rpos);
        servo_door_l.setPosition(servo_door_lpos);

        servo_in_r.setPower(servo_in_rpow);
        servo_in_l.setPower(servo_in_lpow);

        servo_spin_r.setPosition(servo_spin_rpos);
        servo_spin_l.setPosition(servo_spin_lpos);

        telemetry.addData("Servo intake power:", servo_door_rpos);
        servo_plane.setPosition(servo_plane_pos);

        leftBackPower    = Range.clip(-lx - rx - ry, -1.0, 1.0);
        leftFrontPower    = Range.clip(-lx + rx - ry, -1.0, 1.0);
        rightFrontPower   = Range.clip(-lx + rx + ry, -1.0, 1.0);
        rightBackPower   = Range.clip(-lx - rx + ry, -1.0, 1.0) ;
        liftPowerT = Range.clip(rrx, -1.0, 1.0);
        liftPowerE = Range.clip(rry, -1.0, 1.0);


        // Send calculated power to wheels
        double maxSpeed =0.55;
        leftFront.setPower(leftFrontPower*maxSpeed);
        leftBack.setPower(leftBackPower*maxSpeed);
        rightFront.setPower(rightFrontPower*maxSpeed);
        rightBack.setPower(rightBackPower*maxSpeed);
        liftMotorT.setPower(liftPowerT*.4);
        liftMotorE.setPower(liftPowerE*.4);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("lf", leftFront.getCurrentPosition());
        telemetry.addData("rf", rightFront.getCurrentPosition());
        telemetry.addData("lb", leftBack.getCurrentPosition());
        telemetry.addData("rb", rightBack.getCurrentPosition());
        telemetry.addData("LIFT turn", liftMotorT.getCurrentPosition());
        telemetry.addData("LIFT extend", liftMotorE.getCurrentPosition());



        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}