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
public class autofreight extends LinearOpMode
{
    OpenCvCamera webcam;
    public int result = 0;

    public static double intake_velo = 1100;
    public static double outtake_velo = 2000;
    //public static double outtake_dist = 1950;
    public static int outtake_sus = 1300;
    public static int outtake_mijl = 790;
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

    public class IgnitePipeline extends OpenCvPipeline {


        // Working Mat variables
        Mat blur = new Mat();
        Mat hsv = new Mat();
        Mat channel = new Mat();
        Mat thold = new Mat();
        Mat region1_Cb, region2_Cb, region3_Cb;
        int avg1, avg2, avg3;


        // Drawing variables
        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);


        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(12,111);
        final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(128,111);
        final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(245,111);
        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 18;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        @Override
        public Mat processFrame(Mat input) {

            // Img processing
            Imgproc.medianBlur(input, blur, 5);
            Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, channel, 1);
            Imgproc.threshold(channel, thold, 120, 255, Imgproc.THRESH_BINARY);

            region1_Cb = thold.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = thold.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = thold.submat(new Rect(region3_pointA, region3_pointB));

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                result = 0;

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                result = 1;

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                result = 2;

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }
    }

    @Override
    public void runOpMode()
    {

        servo_brat brat = new servo_brat(hardwareMap);
        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        servo_odo odo = new servo_odo(hardwareMap);
        someRandomShit();

        // Camera

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        // Loading pipeline
        IgnitePipeline visionPipeline = new IgnitePipeline();
        webcam.setPipeline(visionPipeline);
        // Start streaming the pipeline
        webcam.startStreaming(320,240,OpenCvCameraRotation.UPSIDE_DOWN);


        // Drive

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(startX-13.5, startY+29, Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.jos();
                    outtake.setTargetPosition((int)outtake_sus);
                    outtake.setVelocity(outtake_velo);
                })
                .addTemporalMarker(0.6, () -> {
                    brat.sus();
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeTo(new Vector2d(startX+5, startY+1))
                .addTemporalMarker(0.55, () -> {
                    brat.jos();
                    cleste2.close();
                })
                .addTemporalMarker(1.1, () -> {
                    outtake.setTargetPosition(20);
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .strafeTo(new Vector2d(startX+51.5, startY+1))
                .addTemporalMarker(0.1, () -> {
                    cleste1.close();
                    cleste2.open();
                    intake1.setVelocity(intake_velo);
                })
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end(), true)
                .strafeTo(new Vector2d(startX+5, startY+1))
                .addTemporalMarker(0.8, () -> {
                    brat.sus();
                    intake1.setVelocity(0);
                })
                .build();

        Trajectory trajectory44 = drive.trajectoryBuilder(trajectory4.end(), true)
                .strafeTo(new Vector2d(startX-14, startY+29))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory44.end())
                .strafeTo(new Vector2d(startX+5, startY+1))
                .addTemporalMarker(0.55, () -> {
                    brat.jos();
                    cleste2.close();
                })
                .addTemporalMarker(1.1, () -> {
                    outtake.setTargetPosition(20);
                })
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .strafeTo(new Vector2d(startX+55.75, startY+1))
                .addTemporalMarker(0.1, () -> {
                    cleste1.close();
                    cleste2.open();
                    intake1.setVelocity(intake_velo);
                })
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end(), true)
                .strafeTo(new Vector2d(startX+5, startY+1))
                .addTemporalMarker(0.8, () -> {
                    brat.sus();
                    intake1.setVelocity(0);
                })
                .build();

        Trajectory trajectory77 = drive.trajectoryBuilder(trajectory7.end(), true)
                .strafeTo(new Vector2d(startX-14, startY+29))
                .build();

        Trajectory trajectorypen = drive.trajectoryBuilder(trajectory77.end())
                .strafeTo(new Vector2d(startX+5, startY+1))
                .addTemporalMarker(0.55, () -> {
                    brat.jos();
                    cleste2.close();
                })
                .addTemporalMarker(1.1, () -> {
                    outtake.setTargetPosition(20);
                })
                .build();

        Trajectory trajectoryfin = drive.trajectoryBuilder(trajectorypen.end())
                .strafeTo(new Vector2d(startX+55, startY+1))
                .build();



        odo.jos();
        telemetry.addData("", result+1);
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            /*
            if(result == 0)
            {
                odo.jos();
                sleep(250);
                drive.followTrajectory(trajectoryyy1);
                cleste1.open();
                cleste2.open();
                sleep(100);
                outtake.setTargetPosition(1880);
                outtake.setVelocity(outtake_velo);
                sleep(500);
                brat.jos();
                cleste1.close();
                cleste2.close();
                sleep(600);
                outtake.setTargetPosition(150);
                drive.followTrajectory(trajectory2);
                cleste2.close();
                cleste1.close();
                sleep(300);
                outtake.setTargetPosition(outtake_sus);
                drive.followTrajectory(trajectory3);
                cleste1.open();
                cleste2.open();
                sleep(50);
                drive.followTrajectory(trajectory4);
                intake1.setVelocity(-440);
                intake1.setVelocity(-440);
                sleep(2000);
                intake1.setVelocity(0);
                sleep(1500);
                drive.followTrajectory(trajectory5);
                drive.followTrajectory(trajectory6);
                drive.followTrajectory(trajectory7);
                outtake.setTargetPosition(-20);
                stop();
            }
            if(result == 1)
            {
                odo.jos();
                sleep(250);
                drive.followTrajectory(trajectoryy1);
                cleste1.open();
                cleste2.open();
                sleep(100);
                outtake.setTargetPosition(1880);
                outtake.setVelocity(outtake_velo);
                sleep(500);
                brat.jos();
                cleste1.close();
                cleste2.close();
                sleep(600);
                outtake.setTargetPosition(150);
                drive.followTrajectory(trajectory2);
                cleste2.close();
                cleste1.close();
                sleep(300);
                outtake.setTargetPosition(outtake_sus);
                drive.followTrajectory(trajectory3);
                cleste1.open();
                cleste2.open();
                sleep(50);
                drive.followTrajectory(trajectory4);
                intake1.setVelocity(-440);
                sleep(2000);
                intake1.setVelocity(0);
                sleep(1500);
                drive.followTrajectory(trajectory5);
                drive.followTrajectory(trajectory6);
                drive.followTrajectory(trajectory7);
                outtake.setTargetPosition(-20);
                stop();
            }

             */
            if(result == 2)
            {
                drive.followTrajectory(trajectory1);
                cleste2.open();
                cleste1.open();
                outtake.setTargetPosition(1880);
                outtake.setVelocity(outtake_velo);
                drive.followTrajectory(trajectory2);
                drive.followTrajectory(trajectory3);
                intake1.setVelocity(-intake_velo);
                sleep(550);
                cleste2.close();
                cleste1.close();
                sleep(300);
                outtake.setTargetPosition(outtake_sus);
                outtake.setVelocity(outtake_velo);
                drive.followTrajectory(trajectory4);
                cleste2.open();
                cleste1.open();
                outtake.setTargetPosition(1880);
                outtake.setVelocity(outtake_velo);
                drive.followTrajectory(trajectory5);
                drive.followTrajectory(trajectory6);
                intake1.setVelocity(-intake_velo);
                sleep(550);
                cleste2.close();
                cleste1.close();
                sleep(300);
                outtake.setTargetPosition(outtake_sus);
                outtake.setVelocity(outtake_velo);
                drive.followTrajectory(trajectory7);
                cleste2.open();
                cleste1.open();
                drive.followTrajectory(trajectorypen);
                drive.followTrajectory(trajectoryfin);
                stop();
            }
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