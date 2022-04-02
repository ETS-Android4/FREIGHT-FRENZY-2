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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
public class auto_albastru_freight extends LinearOpMode
{
    OpenCvCamera webcam;
    public int result = 0;

    public DcMotorEx intake1 = null;
    public DcMotorEx intake2 = null;
    public DcMotorEx brat = null;
    public DcMotorEx carusel = null;


    public Servo servoY = null;
    public Servo servoZ = null;
    public CRServo servoL = null;

    public double ruletaY = 0.80;
    public double ruletaZ = 0.60;

    public static double startX = 0;
    public static double startY = -1;

    public static double brat_power = 1.0;
    public static int brat_sus = 1990;
    public static int brat_hub_mid = 1470;
    public static int brat_hub_jos = 1100;
    public static int brat_jos = 0;

    Trajectory trajectory3;
    Trajectory trajectory4;
    Trajectory trajectory5;
    Trajectory trajectory6;
    Trajectory trajectory7;
    Trajectory trajectory8;
    Trajectory trajectory9;
    Trajectory trajectory10;


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


        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(101,141);
        final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(166,137);
        final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(237,132);
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
            telemetry.addData("zona", result+1);
            telemetry.update();
            return input;
        }
    }


    @Override
    public void runOpMode()
    {

        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        servo_odo odo = new servo_odo(hardwareMap);
        odo.jos();
        someRandomShit();

        brat.setTargetPosition(0);
        brat.setPower(brat_power);

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
                .lineToLinearHeading(new Pose2d(startX-1, startY+23, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(27, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.setTargetPosition(brat_sus);
                    brat.setPower(brat_power);
                })
                .build();

        Trajectory trajectory11 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(startX-1, startY+23, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(27, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.setTargetPosition(brat_hub_mid);
                    brat.setPower(brat_power);
                })
                .build();

        Trajectory trajectory111 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(startX-1, startY+23, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(27, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.close();
                    cleste2.close();
                    brat.setTargetPosition(brat_hub_jos);
                    brat.setPower(brat_power);
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeTo(new Vector2d(startX-18, startY-0.5))
                .splineToConstantHeading(new Vector2d(startX-49.5, startY-0.5), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(33.5, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(1.5, () -> {
                    cleste1.semi();
                    cleste2.open();
                })
                .addTemporalMarker(1.65, () -> {
                    intake2.setVelocity(3300);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory3))
                .build();


        trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .strafeTo(new Vector2d(startX-30, startY-1.25))
                .splineToConstantHeading(new Vector2d(startX, startY+20), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    intake2.setVelocity(-2000);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.75, () -> {
                    intake2.setVelocity(0);
                })
                .addTemporalMarker(0.95, () -> {
                    brat.setTargetPosition(brat_sus);
                    brat.setPower(brat_power);
                })
                .build();

        trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeTo(new Vector2d(startX-15, startY-3))
                .splineToConstantHeading(new Vector2d(startX-53.3, startY-3), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(33.5, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.hub();
                    cleste2.hub();
                    brat.setTargetPosition(brat_sus+150);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(1.5, () -> {
                    cleste1.semi();
                    cleste2.open();
                })
                .addTemporalMarker(1.65, () -> {
                    intake2.setVelocity(3300);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory5))
                .build();

        trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .strafeTo(new Vector2d(startX-32, startY-6))
                .splineToConstantHeading(new Vector2d(startX+0.75, startY+15.9), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    intake2.setVelocity(-2000);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.75, () -> {
                    intake2.setVelocity(0);
                })
                .addTemporalMarker(0.95, () -> {
                    brat.setTargetPosition(brat_sus);
                    brat.setPower(brat_power);
                })
                .build();

        trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .strafeTo(new Vector2d(startX-13.5, startY-7))
                //.splineToConstantHeading(new Vector2d(startX-44, startY), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(startX-56, startY-7), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(33.5, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.hub();
                    cleste2.hub();
                    brat.setTargetPosition(brat_sus+150);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(1.5, () -> {
                    cleste1.semi();
                    cleste2.open();
                })
                .addTemporalMarker(1.65, () -> {
                    intake2.setVelocity(3300);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory7))
                .build();

        trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .strafeTo(new Vector2d(startX-26, startY-8))
                .splineToConstantHeading(new Vector2d(startX+1.75, startY+12.6), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    intake2.setVelocity(-2000);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.75, () -> {
                    intake2.setVelocity(0);
                })
                .addTemporalMarker(1, () -> {
                    brat.setTargetPosition(brat_sus+30);
                    brat.setPower(brat_power);
                })
                .build();


        trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                .strafeTo(new Vector2d(startX-10, startY-9))
                //.splineToConstantHeading(new Vector2d(startX-44, startY), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(startX-59, startY-9), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(33.5, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.hub();
                    cleste2.hub();
                    brat.setTargetPosition(brat_sus+150);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.6, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(1.5, () -> {
                    cleste1.semi();
                    cleste2.open();
                })
                .addTemporalMarker(1.65, () -> {
                    intake2.setVelocity(3300);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory9))
                .build();

        trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .strafeTo(new Vector2d(startX-55, startY-13))
                .splineToConstantHeading(new Vector2d(startX-24, startY-12.4), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(startX+3, startY+9.8), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    intake2.setVelocity(-2000);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.75, () -> {
                    intake2.setVelocity(0);
                })
                .addTemporalMarker(1.05, () -> {
                    brat.setTargetPosition(brat_sus+30);
                    brat.setPower(brat_power);
                })
                .build();

        trajectory10 = drive.trajectoryBuilder(trajectory9.end())
                .strafeTo(new Vector2d(startX-9, startY-14))
                .splineToConstantHeading(new Vector2d(startX-50, startY-13), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(33.5, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0, () -> {
                    cleste1.hub();
                    cleste2.hub();
                    brat.setTargetPosition(brat_sus+150);
                    brat.setPower(brat_power);
                })
                .addTemporalMarker(0.5, () -> {
                    cleste1.close();
                    cleste2.close();
                })
                .addTemporalMarker(0.8, () -> {
                    brat.setTargetPosition(brat_jos);
                    brat.setPower(brat_power);
                })
                .build();

        cleste1.close();
        cleste2.close();

        waitForStart();

        while (opModeIsActive())
        {
            if(result == 0){
                drive.followTrajectory(trajectory111);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory2);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory4);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory6);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory8);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory10);
                stop();
            }
            else if(result == 1){
                drive.followTrajectory(trajectory11);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory2);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory4);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory6);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory8);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory10);
                stop();
            }
            else if(result == 2){
                drive.followTrajectory(trajectory1);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory2);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory4);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory6);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory8);
                cleste1.hub();
                cleste2.hub();
                sleep(150);
                drive.followTrajectory(trajectory10);
                stop();
            }


            /*
            sleep(100);
            intake1.setVelocity(-1800);
            sleep(100);
            drive.followTrajectory(trajectory3);
            cleste1.hub();
            cleste2.hub();
            sleep(150);
            drive.followTrajectory(trajectory4);
            sleep(100);
            intake1.setVelocity(-1800);
            sleep(100);
            drive.followTrajectory(trajectory5);
            cleste1.hub();
            cleste2.hub();
            sleep(150);
            drive.followTrajectory(trajectory6);
            sleep(100);
            intake1.setVelocity(-1800);
            sleep(100);
            drive.followTrajectory(trajectory7);
            cleste1.hub();
            cleste2.hub();
            sleep(150);
            drive.followTrajectory(trajectory8);
            sleep(100);
            intake1.setVelocity(-1800);
            sleep(100);
            drive.followTrajectory(trajectory9);
            cleste1.hub();
            cleste2.hub();
            sleep(150);
            drive.followTrajectory(trajectory10);

             */
            stop();
        }
    }


    public void someRandomShit(){

        servoL = hardwareMap.get(CRServo.class, "servoL");
        servoY = hardwareMap.get(Servo.class, "servoY");
        servoZ = hardwareMap.get(Servo.class, "servoZ");
        servoL.setPower(0);
        servoY.setPosition(ruletaY);
        servoZ.setPosition(ruletaZ);


        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");

        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setPower(0.0);


        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        intake2.setPower(0.0);


        carusel = hardwareMap.get(DcMotorEx.class, "carusel");

        carusel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carusel.setDirection(DcMotor.Direction.FORWARD);
        carusel.setPower(0.0);


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