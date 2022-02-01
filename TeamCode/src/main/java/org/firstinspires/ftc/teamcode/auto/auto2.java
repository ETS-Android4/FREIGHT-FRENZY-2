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
import org.firstinspires.ftc.teamcode.hardware.servo_brat;
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
public class auto2 extends LinearOpMode
{

    public static double intake_velo = 1250;
    public static double outtake_velo = 2000;
    //public static double outtake_dist = 1950;
    public static int outtake_sus = 1600;
    public static int outtake_mijl = 1100;
    public static int outtake_jos = 750;

    public static double down_pos = 5;
    public static double p = 2.5;
    public static double i = 1;
    public static double d = 0;
    public static double f = 13;
    public static double pp = 10;
    public DcMotorEx outtake = null;
    public DcMotorEx intake1 = null;

    public static double startX = 0;
    public static double startY = 0;


    @Override
    public void runOpMode()
    {

        servo_brat brat = new servo_brat(hardwareMap);
        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        servo_odo odo = new servo_odo(hardwareMap);
        someRandomShit();



        // Drive

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(startX+18.25, startY+27, Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.jos();
                    outtake.setTargetPosition((int)outtake_sus);
                    outtake.setVelocity(outtake_velo);
                })
                .addTemporalMarker(0.75, () -> {
                    brat.sus();
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineToConstantHeading(new Vector2d(startX+44.75, startY+28), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(8.5, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.35, () -> {
                    outtake.setTargetPosition((int)20);
                    outtake.setVelocity(outtake_velo);
                })
                .addTemporalMarker(0.7, () -> {
                    cleste2.open();
                    intake1.setVelocity(intake_velo);
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .splineToConstantHeading(new Vector2d(startX+40, startY+1), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(startX+52.5, startY+1), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(13, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.1, () -> {
                    outtake.setTargetPosition((int)10);
                    outtake.setVelocity(outtake_velo);
                    cleste2.open();
                })
                .addTemporalMarker(0.9, () -> {
                    intake1.setVelocity(intake_velo);
                })

                .build();



        /*

        TrajectorySequence myBot = new TrajectorySequence()
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, Math.toRadians(180), Math.toRadians(180), 10.5)
                .setDimensions(12.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startX, startY, 0))



                                //next cycle
                                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                                    //opreste intake
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                    //invers intake
                                    //inchide cleste
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                                    //glisiera 200
                                    //opreste intake
                                })

                                //.setAccelConstraint(TrajectoryAccelerationConstraint.get(10.0))
                                .waitSeconds(0.25)
                                .lineToLinearHeading(new Pose2d(startX+10, startY+1, Math.toRadians(0)))

                                .UNSTABLE_addDisplacementMarkerOffset(-15, () -> {
                                    //glisiera 1000
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(16, () -> {
                                    //brat sus
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(39, () -> {
                                    //deschide cleste
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(52, () -> {
                                    //inchide cleste
                                    //brat jos
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(80, () -> {
                                    //glisiera 200
                                })
                                .splineToConstantHeading(new Vector2d(startX-13, startY+23), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(startX+12, startY+1), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(startX+44, startY+1), Math.toRadians(0))
                                .UNSTABLE_addDisplacementMarkerOffset(18, () -> {
                                    //glisiera 0
                                    //cleste 1 open
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(28, () -> {
                                    //start intake
                                })

                                .build()
                );

         */


        waitForStart();

        while (opModeIsActive())
        {
            /*
            cleste1.close();
            cleste2.close();
            outtake.setTargetPosition(outtake_sus);
            outtake.setVelocity(outtake_velo);
             */
            odo.jos();
            sleep(250);
            drive.followTrajectory(trajectory1);
            cleste1.open();
            cleste2.open();
            sleep(300);
            outtake.setTargetPosition(1890);
            outtake.setVelocity(outtake_velo);
            sleep(600);
            brat.jos();
            cleste1.close();
            cleste2.close();
            sleep(700);
            outtake.setTargetPosition(150);
            drive.followTrajectory(trajectory2);
            cleste2.close();
            cleste1.close();
            sleep(500);
            stop();
        }

    }

    public void someRandomShit(){

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setVelocityPIDFCoefficients(p, i, d, f);
        outtake.setPositionPIDFCoefficients(pp);
        outtake.setTargetPosition(5);
        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.setPower(0.0);
        outtake.setTargetPositionTolerance(2);


        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setPower(0.0);
    }

}