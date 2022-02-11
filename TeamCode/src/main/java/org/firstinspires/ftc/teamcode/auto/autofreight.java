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


        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50,111);
        final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(165,111);
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
        odo.jos();
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
                .lineToLinearHeading(new Pose2d(startX-13.5, startY+27, Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.jos();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.sus();
                })
                .build();

        Trajectory trajectory11 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(startX-13.5, startY+27, Math.toRadians(0)),
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
                    brat.jos();
                })
                .addTemporalMarker(0.5, () -> {
                    brat.second();
                })
                .build();

        Trajectory trajectory111 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(startX-13.5, startY+27, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(18, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.jos();
                })
                .addTemporalMarker(0.5, () -> {
                    brat.first();
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeTo(new Vector2d(startX+2.5, startY+3.5))
                .splineToConstantHeading(new Vector2d(startX+45, startY+1.5), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    brat.jos();
                    cleste2.semi();
                    cleste1.semi();
                })
                .addTemporalMarker(2.3, () -> {
                    cleste1.close();
                    cleste2.open();
                    intake1.setVelocity(intake_velo);
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .strafeTo(new Vector2d(startX+6, startY))
                .addTemporalMarker(0.8, () -> {
                    brat.sus();
                    intake1.setVelocity(0);
                })
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeTo(new Vector2d(startX-14, startY+28.75))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .strafeTo(new Vector2d(startX+2.5, startY+3.5))
                .splineToConstantHeading(new Vector2d(startX+49.5, startY+1.5), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    brat.jos();
                    cleste2.semi();
                    cleste1.semi();
                })
                .addTemporalMarker(2.3, () -> {
                    cleste1.close();
                    cleste2.open();
                    intake1.setVelocity(intake_velo);
                })
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .strafeTo(new Vector2d(startX+3.75, startY))
                .addTemporalMarker(0.8, () -> {
                    brat.sus();
                    intake1.setVelocity(0);
                })
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .strafeTo(new Vector2d(startX-14, startY+28.75))
                .build();

        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                .strafeTo(new Vector2d(startX+1, startY+3.5))
                .splineToConstantHeading(new Vector2d(startX+53.5, startY+1.5), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    brat.jos();
                    cleste2.semi();
                    cleste1.semi();
                })
                .addTemporalMarker(2.3, () -> {
                    cleste1.close();
                    cleste2.open();
                    intake1.setVelocity(intake_velo);
                })
                .build();

        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .strafeTo(new Vector2d(startX+5, startY))
                .addTemporalMarker(0.8, () -> {
                    brat.sus();
                    intake1.setVelocity(0);
                })
                .build();

        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end())
                .strafeTo(new Vector2d(startX-14, startY+28.75))
                .build();

        Trajectory trajectory100 = drive.trajectoryBuilder(trajectory10.end())
                .strafeTo(new Vector2d(startX, startY-3))
                .splineToConstantHeading(new Vector2d(startX+42.5, startY-2), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.4, () -> {
                    brat.jos();
                    cleste2.semi();
                    cleste1.semi();
                })
                .addTemporalMarker(2.3, () -> {
                    cleste1.close();
                    cleste2.open();
                    intake1.setVelocity(intake_velo);
                })
                .build();


        cleste1.close();
        cleste2.close();

        waitForStart();

        while (opModeIsActive())
        {
            if(result == 0)
            {
                drive.followTrajectory(trajectory111);
                cleste2.open();
                cleste1.open();
                drive.followTrajectory(trajectory2);
                intake1.setVelocity(-intake_velo);
                sleep(550);
                cleste2.close();
                cleste1.close();
                sleep(300);
                drive.followTrajectory(trajectory3);
                drive.followTrajectory(trajectory4);
                cleste2.open();
                cleste1.open();
                drive.followTrajectory(trajectory5);
                intake1.setVelocity(-intake_velo);
                sleep(550);
                cleste2.close();
                cleste1.close();
                sleep(300);
                drive.followTrajectory(trajectory6);
                drive.followTrajectory(trajectory7);
                cleste2.open();
                cleste1.open();
                drive.followTrajectory(trajectory8);
                intake1.setVelocity(-intake_velo);
                sleep(550);
                cleste2.close();
                cleste1.close();
                sleep(300);
                drive.followTrajectory(trajectory9);
                drive.followTrajectory(trajectory10);
                cleste2.open();
                cleste1.open();
                drive.followTrajectory(trajectory100);
                stop();
            }
            if(result == 1)
            {
                stop();
            }
            if(result == 2)
            {
                stop();
            }
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
    }

}