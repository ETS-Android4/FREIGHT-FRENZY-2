package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_odo;


import java.util.Arrays;

@TeleOp
@Disabled
public class augmenteddrive extends LinearOpMode {

    private double root2 = Math.sqrt(2.0);
    private boolean slow_mode = false;
    private boolean reverse_intake = false;

    public boolean ok = false;
    public boolean ok_intake = false;
    public boolean ok2 = false;

    public static double outtake_velo = 2000;
    //public static double outtake_dist = 1950;
    public static double outtake_sus = 1350;
    public static double outtake_mijl = 760;
    public static double outtake_jos = 550;
    public static int down_pos = 50;

    public static double p = 2.5;
    public static double i = 1;
    public static double d = 0;
    public static double f = 13;
    public static double pp = 10;
    public DcMotorEx outtake = null;
    public DcMotorEx intake1 = null;
    public DcMotorEx intake2 = null;

    private ElapsedTime runtime = new ElapsedTime();

    public double intake_speed = 0.58;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector;
    // The heading we want the bot to end on for targetA
    double targetAHeading;

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(0);
    double targetAngleZero = Math.toRadians(180);

    SampleMecanumDrive drive;


    @Override
    public void runOpMode() {


        someRandomShit();
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");

        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setPower(0.0);


        /* Intake motor 1 */
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        intake2.setPower(0.0);


        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        servo_odo odo = new servo_odo(hardwareMap);

        odo.jos();
        cleste1.close();
        cleste2.close();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setTargetPosition(50);
        outtake.setVelocity(1000);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:


                    // GAMEPAD 1 //

                    drive.setDrivePower(
                            new Pose2d(
                                     gamepad1.left_stick_x,
                                    -gamepad1.left_stick_y,
                                    -gamepad1.right_stick_x
                            )
                    );


                    if(gamepad1.a)
                        resetPositionLine();


                    // GAMEPAD 2 //

                    if(gamepad2.dpad_up){
                        outtake.setTargetPosition((int)outtake_sus);
                        outtake.setVelocity(outtake_velo);
                    }

                    if(gamepad2.dpad_left){
                        intake1.setVelocity(0);
                        outtake.setTargetPosition(1730);
                        outtake.setVelocity(outtake_velo);
                        runtime.reset();
                        ok = true;
                    }

                    if(runtime.seconds() > 0.5 && ok)
                    {
                        ok = false;
                        ok2 = true;
                        cleste1.semi();
                        cleste2.semi();
                        runtime.reset();
                    }

                    if(runtime.seconds() > 0.5 && ok2)
                    {
                        ok2 = false;
                        outtake.setTargetPosition(down_pos);
                    }


                    if(gamepad2.x)
                    {
                        cleste1.open();
                        cleste2.open();
                    }
                    if(gamepad2.a && gamepad2.b)
                    {
                        outtake.setTargetPosition(-100);
                        outtake.setVelocity(1000);
                        sleep(250);
                        someRandomShit();
                        sleep(250);
                    }

                    if(gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1){
                        cleste1.close();
                        cleste2.open();
                        outtake.setTargetPosition(down_pos);
                        outtake.setVelocity(outtake_velo);
                        intake1.setVelocity(-1500*Math.min(gamepad2.right_trigger, intake_speed));
                        ok_intake = true;
                    }
                    else if(gamepad2.right_trigger > 0.1) {
                        cleste1.close();
                        cleste2.open();
                        outtake.setTargetPosition(down_pos);
                        outtake.setVelocity(outtake_velo);
                        intake1.setVelocity(1500*Math.min(gamepad2.right_trigger, intake_speed));
                        ok_intake = true;
                    }
                    else if(ok_intake){
                        cleste1.close();
                        cleste2.close();
                        intake1.setVelocity(0);
                        ok_intake = false;
                    }

                    if(gamepad1.b){
                        intake1.setVelocity(-440);
                        intake2.setVelocity(440);
                        sleep(1430);
                        intake1.setVelocity(-1400);
                        intake2.setVelocity(1400);
                        sleep(250);
                        intake1.setVelocity(0);
                        intake2.setVelocity(0);
                    }

                    if(gamepad1.dpad_left)
                    {
                        currentMode = Mode.AUTOMATIC_CONTROL;
                        resetPositionLine();

                        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(), true)
                                .strafeTo(new Vector2d(-32, 1))
                                .splineToConstantHeading(new Vector2d(-56, 26), Math.toRadians(0),
                                        new MinVelocityConstraint(
                                                Arrays.asList(
                                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                                )
                                        ),
                                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .addTemporalMarker(0, () -> {
                                    outtake.setTargetPosition((int)outtake_sus+225);
                                    outtake.setVelocity(outtake_velo);
                                })
                                .addTemporalMarker(1, () -> {
                                })
                                .build();

                        drive.followTrajectory(trajectory1);
                    }

                    if(gamepad1.dpad_right)
                    {
                        currentMode = Mode.AUTOMATIC_CONTROL;
                        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(-56, 25.5, 0))
                                .strafeTo(new Vector2d(-46, 3.5))
                                .splineToConstantHeading(new Vector2d(-6, 4), Math.toRadians(0),
                                        new MinVelocityConstraint(
                                                Arrays.asList(
                                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                                )
                                        ),
                                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .addTemporalMarker(0.8, () -> {
                                    cleste1.semi();
                                    cleste2.semi();
                                })
                                .addTemporalMarker(1.4, () -> {
                                    outtake.setTargetPosition(down_pos);
                                })
                                .build();

                        cleste1.open();
                        cleste2.open();
                        sleep(200);
                        drive.followTrajectory(trajectory2);
                        someRandomShit();
                        sleep(200);
                        outtake.setTargetPosition(50);
                        outtake.setVelocity(outtake_velo);
                    }

                    if(gamepad1.dpad_up)
                    {
                        currentMode = Mode.AUTOMATIC_CONTROL;
                        resetPositionLine();

                        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                                .strafeTo(new Vector2d(-32, 0))
                                .splineToConstantHeading(new Vector2d(-56, 27), Math.toRadians(0),
                                        new MinVelocityConstraint(
                                                Arrays.asList(
                                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                                )
                                        ),
                                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .addTemporalMarker(0, () -> {
                                    outtake.setTargetPosition((int)outtake_sus+50);
                                    outtake.setVelocity(outtake_velo);
                                })
                                .addTemporalMarker(1, () -> {
                                })
                                .build();

                        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                                .strafeTo(new Vector2d(-46, 5))
                                .splineToConstantHeading(new Vector2d(-2, 6.25), Math.toRadians(0),
                                        new MinVelocityConstraint(
                                                Arrays.asList(
                                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                        new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                                )
                                        ),
                                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .addTemporalMarker(0.8, () -> {
                                    cleste1.semi();
                                    cleste2.semi();
                                })
                                .addTemporalMarker(1.4, () -> {
                                    outtake.setTargetPosition(down_pos);
                                })
                                .build();

                        drive.followTrajectory(trajectory1);
                        cleste1.semi();
                        cleste2.semi();
                        drive.followTrajectory(trajectory2);
                    }

                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }


                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
            telemetry.addLine()
                    .addData("X", poseEstimate.getX())
                    .addData("Y", poseEstimate.getY())
                    .addData("angle", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();

        }
    }


    void resetPositionLine()
    {
        drive.setPoseEstimate(new Pose2d(0,0, 0));
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

    }

}