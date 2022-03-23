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
public class auto_rosu_freight_test extends LinearOpMode
{


    public DcMotorEx intake1 = null;
    public DcMotorEx intake2 = null;
    public DcMotorEx brat = null;
    public DcMotorEx carusel = null;


    public Servo servoY = null;
    public Servo servoZ = null;
    public CRServo servoL = null;

    public double ruletaY = 0.45;
    public double ruletaZ = 0.55;

    public static double startX = 0;
    public static double startY = 0;

    public static double brat_power = 1.0;
    public static int brat_sus = 1987;
    public static int brat_hub_mid = 1530;
    public static int brat_hub_jos = 1100;
    public static int brat_jos = 0;


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

        // Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        cleste1.close();
        cleste2.close();

        waitForStart();

        while (opModeIsActive())
        {
            Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(startX+1, startY+22.5), Math.toRadians(0),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .splineToConstantHeading(new Vector2d(startX+57.5, startY+1), Math.toRadians(0))
                    .addTemporalMarker(2.4, () -> {
                        cleste1.close();
                        cleste2.semi();
                    })
                    .addTemporalMarker(6.5, () -> {
                        brat.setTargetPosition(brat_jos);
                        brat.setPower(brat_power);
                    })
                    .addTemporalMarker(6.7, () -> {
                        cleste1.open();
                        cleste2.close();
                    })
                    .addTemporalMarker(7, () -> {
                        intake1.setVelocity(1800);
                    })
                    .build();
            drive.followTrajectory(trajectory1);
            cleste1.open();
            cleste2.close();
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