package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
            Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
            Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX()+1.0);
            telemetry.addData("x1", leftEncoder.getCurrentPosition());
            telemetry.addData("x2", rightEncoder.getCurrentPosition());
            telemetry.addData("x3", frontEncoder.getCurrentPosition());
            telemetry.addData("y", poseEstimate.getY()+"");
            telemetry.addData("heading", poseEstimate.getHeading()+"");
            telemetry.update();
        }
    }
}
