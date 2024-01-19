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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.openCV.SignalColor;
import org.firstinspires.ftc.teamcode.openCV.camera;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.Math;
import java.sql.SQLOutput;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="auto TEST")

public class redAutoParkTEST extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor liftMotorE = null;
    private DcMotor liftMotorT = null;

    private Encoder xrEncoder = null;
    private Encoder xlEncoder = null;
    private Encoder frontEncoder = null;
    private OpenCvWebcam webcam;
    private camera pipeline;
    private SignalColor color;
    private int zone;

    Servo servo_claw; //open and close
    double servo_claw_pos = SERVO_CLAW_GRAB;
    //in
    static final double SERVO_CLAW_INIT = .52;
    //out
    static final double SERVO_CLAW_GRAB = .64;

    private Thread telemetryH = new Thread() {
        @Override
        public void run() {
            while (opModeInInit() || opModeIsActive()) {
                telemetry.addData("rf", rightFront.getCurrentPosition());
                telemetry.addData("x1", xrEncoder.getCurrentPosition());
                telemetry.addData("x2", xlEncoder.getCurrentPosition());
                telemetry.addData("y", frontEncoder.getCurrentPosition());
                telemetry.addData("color", pipeline.getBiggestArea());
                telemetry.addData("zone",  pipeline.getZone());
                telemetry.update();
            }
        }
    };


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        pipeline = new camera(telemetry);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                //if the camera cannot open
                telemetry.addData("camera status: ", "error");
            }

        });

        double startTime = System.currentTimeMillis();
        while (pipeline.getBiggestArea() == SignalColor.IDK && System.currentTimeMillis() - startTime < 3000) {
            telemetry.addData("camera status: ", "not ready");
            telemetry.update();
        }

        // Initialize the drive system variables.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");//xr odometry
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");//xl odometry
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");//y odometry
        liftMotorE = hardwareMap.get(DcMotor.class, "liftMotorE");
        liftMotorT = hardwareMap.get(DcMotor.class, "liftMotorT");
        servo_claw = hardwareMap.get(Servo.class, "servo_claw");

        servo_claw.setPosition(servo_claw_pos);



        xrEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        xlEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));

        int xrPos = xrEncoder.getCurrentPosition();
        int xlPos = xlEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);



        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Send telemetry message to indicate successful Encoder reset
        telemetryH.start();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        color = pipeline.getBiggestArea();
        zone = pipeline.getZone();

        liftMotorT.setTargetPosition(-150);
        liftMotorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotorE.setTargetPosition(10); //TODO: FIX
        liftMotorE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()){

            if (true) {
                forwardDrive(.4,2500);
                if(zone == 0){
                    strafeRight(.3, 500);
                    telemetry.addData("posr", xrEncoder.getCurrentPosition());
                    telemetry.addData("posl", xlEncoder.getCurrentPosition());
                    telemetry.addData("posy", frontEncoder.getCurrentPosition());
                } else if(zone == 1){
                    forwardDrive(.4, 300);
                } else if(zone == 2){
                    strafeLeft(.4, 300);
                }
                liftMotorE.setTargetPosition(10); //ranbdom val, dont use this!!!!
                liftMotorT.setTargetPosition(-200); //random value

                forwardDrive(-.4, 150);
                //add in parking
                color = SignalColor.IDK;
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            } else if (color == SignalColor.IDK) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }

            strafeRight(.6, 1500);
            servo_claw.setPosition(servo_claw_pos);

            telemetry.addData("Path", "Complete");

            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
        }}

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    public void forwardDrive(double power, int position) {

        System.out.println("in forward drive");

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);


        //wait until reaches position
        while (Math.abs(xrEncoder.getCurrentPosition()) < position && opModeIsActive()) {
            telemetry.addData("position: ", rightFront.getCurrentPosition());
            telemetry.update();

        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeLeft(double power, int position) {

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        System.out.println("in strafe l");

        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(-power);

        //wait until finishes turning
        while (Math.abs(xrEncoder.getCurrentPosition()) < position && opModeIsActive()) {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeRight(double power, int position) {

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        rightFront.setPower(power);

        //wait until finishes turning
        while (Math.abs(xrEncoder.getCurrentPosition()) < position && opModeIsActive()) {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}