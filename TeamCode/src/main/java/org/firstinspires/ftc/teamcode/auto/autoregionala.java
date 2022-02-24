package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;
import org.firstinspires.ftc.teamcode.hardware.servo_odo;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous
//@Disabled
public class autoregionala extends LinearOpMode
{
    OpenCvCamera webcam;
    public int result = 0;

    public static double intake_speed = 0.64;

    public DcMotorEx intake1 = null;
    public DcMotorEx brat = null;

    public static double startX = 0;
    public static double startY = 0;

    public static double brat_power = 0.6;
    public static double brat_power_incet = 0.28;
    public static int brat_sus = 715;
    public static int brat_jos = 0;
    public static int brat_jos_intake = -30;

    @Override
    public void runOpMode()
    {

        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        servo_odo odo = new servo_odo(hardwareMap);
        odo.jos();
        someRandomShit();

        brat.setTargetPosition(-30);
        brat.setPower(brat_power);

        // Drive

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(startX+1.5, startY+24, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.setTargetPosition(brat_sus);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(0.2, () -> {
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setPower(brat_power_incet);
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeTo(new Vector2d(startX+15, startY+2),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(27, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+54, startY+2), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+58.5, startY+2), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    cleste1.close();
                    cleste2.semi();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power_incet+0.1);
                })
                .addTemporalMarker(1.15, () -> {
                    brat.setTargetPosition(brat_jos_intake);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(2.2, () -> {
                    cleste1.open();
                    cleste2.close();
                })
                .addTemporalMarker(2.4, () -> {
                    intake1.setVelocity(1000);
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .strafeTo(new Vector2d(startX+15, startY),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+1, startY+22), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    intake1.setVelocity(-1000);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.75, () -> {
                    intake1.setVelocity(0);
                })
                .addTemporalMarker(1.2, () -> {
                    brat.setTargetPosition(brat_sus);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(1.4, () -> {
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(1.85, () -> {
                    brat.setPower(brat_power_incet);
                })
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeTo(new Vector2d(startX+15, startY+1),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+58, startY+2), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+63, startY+2), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    cleste1.close();
                    cleste2.semi();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(1.15, () -> {
                    brat.setTargetPosition(brat_jos_intake);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(2.2, () -> {
                    cleste1.open();
                    cleste2.close();
                })
                .addTemporalMarker(2.4, () -> {
                    intake1.setVelocity(1000);
                })
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .strafeTo(new Vector2d(startX+15, startY),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX-1.5, startY+23), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    intake1.setVelocity(-1000);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.75, () -> {
                    intake1.setVelocity(0);
                })
                .addTemporalMarker(1.2, () -> {
                    brat.setTargetPosition(brat_sus);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(1.4, () -> {
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(1.85, () -> {
                    brat.setPower(brat_power_incet);
                })
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .strafeTo(new Vector2d(startX+15, startY),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+58, startY+2), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+65.5, startY+2), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    cleste1.close();
                    cleste2.semi();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(1.15, () -> {
                    brat.setTargetPosition(brat_jos_intake);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(2.2, () -> {
                    cleste1.open();
                    cleste2.close();
                })
                .addTemporalMarker(2.4, () -> {
                    intake1.setVelocity(1000);
                })
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .strafeTo(new Vector2d(startX+15, startY),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+3, startY+25), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    intake1.setVelocity(-1300);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.75, () -> {
                    intake1.setVelocity(0);
                })
                .addTemporalMarker(1.2, () -> {
                    brat.setTargetPosition(brat_sus);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(1.4, () -> {
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(1.85, () -> {
                    brat.setPower(brat_power_incet);
                })
                .build();

        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                .strafeTo(new Vector2d(startX+15, startY),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(startX+70, startY), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    cleste1.close();
                    cleste2.semi();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power_incet);
                })
                .addTemporalMarker(1.15, () -> {
                    brat.setTargetPosition(brat_jos_intake);
                    brat.setPower(brat_power_incet);
                })
                .build();

        cleste1.close();
        cleste2.close();

        waitForStart();

        while (opModeIsActive())
        {
            drive.followTrajectory(trajectory1);
            cleste1.hub();
            cleste2.hub();
            drive.followTrajectory(trajectory2);
            sleep(500);
            drive.followTrajectory(trajectory3);
            cleste1.hub();
            cleste2.hub();
            drive.followTrajectory(trajectory4);
            sleep(500);
            drive.followTrajectory(trajectory5);
            cleste1.hub();
            cleste2.hub();
            drive.followTrajectory(trajectory6);
            sleep(500);
            drive.followTrajectory(trajectory7);
            cleste1.hub();
            cleste2.hub();
            drive.followTrajectory(trajectory8);
            stop();
        }
    }

    public void someRandomShit(){
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setPower(0.0);

        brat = hardwareMap.get(DcMotorEx.class, "brat");
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brat.setDirection(DcMotor.Direction.FORWARD);
        brat.setTargetPosition(1);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(0.0);
        brat.setTargetPositionTolerance(2);
    }

}