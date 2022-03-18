package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;
import org.firstinspires.ftc.teamcode.hardware.servo_odo;

import java.util.Arrays;

@Config
@TeleOp
//@Disabled
public class testcarusel extends LinearOpMode {

    private double root2 = Math.sqrt(2.0);
    private boolean slow_mode = false;
    private boolean reverse_intake = false;

    double power = 0;
    double time = 0;
    int timeCount = 0;

    public DcMotorEx intake2 = null;

    private ElapsedTime runtime = new ElapsedTime();


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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {


        double[] powers = {0.5, 0.6, 0.6, 0.6, 1};

        double timeShift = 45;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();




        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);

        cleste1.close();
        cleste2.close();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            //Gamepad 1 Variables
            // waitForStart();
            runtime.reset();

            double rightTrigger = (gamepad1.right_trigger);//

            telemetry.addData("Is Right Trigger Pressed: ", rightTrigger);
            telemetry.addData("POWER APPLIED: ", power); //Should be increasing on the screen
            telemetry.addData("time", time);
            telemetry.update();

            if(rightTrigger > 0.5){
                if(timeCount+1 < powers.length) {
                    time += 0.01;
                    if (time > timeShift) {
                        timeCount++;
                        time = 0;
                    }
                }
                if(power < powers[timeCount]){
                    power+=0.01;
                }
                intake2.setPower(power);
            } else {
                intake2.setPower(0);
                power = 0;
                timeCount = 0;
                time = 0;
            }

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

            dashboardTelemetry.update();

        }
    }


    void resetPositionLine()
    {
        drive.setPoseEstimate(new Pose2d(0,0, 0));
    }


}