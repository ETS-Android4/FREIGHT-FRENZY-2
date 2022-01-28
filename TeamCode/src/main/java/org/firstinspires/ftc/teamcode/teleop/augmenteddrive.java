package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_brat;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import static java.lang.Boolean.FALSE;

@TeleOp
//@Disabled
public class augmenteddrive extends LinearOpMode {

    private double root2 = Math.sqrt(2.0);
    private boolean slow_mode = false;
    private boolean reverse_intake = false;

    public boolean ok = false;
    public boolean ok_intake = false;
    public boolean okk_intake = true;

    public static double outtake_velo = 2000;
    public static double outtake_sus = 1200;
    public static double outtake_mijl = 770;
    public static double outtake_jos = 550;

    public static double down_pos = 5;
    public static double p = 2.5;
    public static double i = 1;
    public static double d = 0;
    public static double f = 13;
    public static double pp = 10;
    public DcMotorEx outtake = null;
    public DcMotorEx intake1 = null;
    public DcMotorEx intake2 = null;

    public double intake_speed = 0.58;

    public static com.acmerobotics.roadrunner.control.PIDCoefficients MOTOR_VELO_PID = new com.acmerobotics.roadrunner.control.PIDCoefficients(0.00038, 0.0000012, 0);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime veloTimer = new ElapsedTime();

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

    Vector2d towerVector = new Vector2d(125, -23);

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


        servo_brat brat = new servo_brat(hardwareMap);
        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        cleste1.close();
        cleste2.close();
        brat.jos();


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if(gamepad1.dpad_up)
                    {
                        resetPositionLine();

                        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                                .strafeTo(new Vector2d(-20, 0))
                                .splineTo(new Vector2d(-35, -20), Math.toRadians(90))
                                .addTemporalMarker(0.8, () -> {
                                    outtake.setTargetPosition((int)outtake_sus);
                                    outtake.setVelocity(outtake_velo);
                                })
                                .addTemporalMarker(1.3, () -> {
                                    brat.sus();
                                })
                                .build();

                        drive.followTrajectory(trajectory1);
                    }

                    if(gamepad1.dpad_down)
                    {
                        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                                .splineTo(new Vector2d(-20, 0), Math.toRadians(0))
                                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                                .addTemporalMarker(0.4, () -> {
                                    cleste1.close();
                                    cleste2.close();
                                    brat.jos();
                                })
                                .addTemporalMarker(1.1, () -> {
                                    outtake.setTargetPosition(150);
                                })
                                .build();

                        cleste1.open();
                        cleste2.open();
                        sleep(400);
                        drive.followTrajectory(trajectory2);
                    }


                    if(gamepad1.a)
                        resetPositionLine();


                    // GAMEPAD 2 //

                    if(gamepad2.right_trigger > 0.1) {
                        cleste1.close();
                        cleste2.open();
                        outtake.setTargetPosition((int)down_pos);
                        outtake.setVelocity(outtake_velo);

                        intake1.setVelocity(1500*Math.min(gamepad2.right_trigger, intake_speed));

                        ok_intake = true;
                    }

                    else if(ok_intake){
                        cleste1.close();
                        cleste2.close();
                        intake1.setVelocity(0);
                        sleep(350);
                        outtake.setTargetPosition(200);
                        outtake.setVelocity(outtake_velo);
                        ok_intake = false;
                    }

                    if(gamepad2.b){
                        intake1.setVelocity(-440);
                        intake2.setVelocity(440);
                        sleep(1380);
                        intake1.setVelocity(-1400);
                        intake2.setVelocity(1400);
                        sleep(180);
                        intake1.setVelocity(0);
                        intake2.setVelocity(0);
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