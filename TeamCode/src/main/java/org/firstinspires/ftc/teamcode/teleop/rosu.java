/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;
import org.firstinspires.ftc.teamcode.hardware.servo_odo;

@Config
@TeleOp
//@Disabled
public class rosu extends LinearOpMode {

    init_robot conserva = new init_robot();

    private double root2 = Math.sqrt(2.0);
    private int slow_mode = 1;
    private int intake_state = 0;

    public int poz_outtake_urcare = 0;

    public boolean brat_reset1 = false;
    public boolean brat_reset2 = false;

    private ElapsedTime runtime_outtake = new ElapsedTime();
    private ElapsedTime runtime_intake = new ElapsedTime();

    DcMotorEx brat = null;
    public static double brat_power = 1;
    public static int brat_sus = 2000;
    public static int brat_shared = 1270;
    public static int brat_shared_jos = 1000;
    public static int brat_jos = 0;
    public int outtake_error = 0;

    public static double intake_speed = 0.8;

    public Servo servoY = null;
    public Servo servoZ = null;
    public CRServo servoL = null;

    public double ruletaY = 0.80;
    public double ruletaZ = 0.38;

    double power = 0;
    double time = 0;
    int timeCount = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {

        double[] powers = {0.57, 0.62, 0.66, 0.71, 1};

        double timeShift = 45;


        conserva.init(hardwareMap);
        someRandomShit();
        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;

        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        servo_odo odo = new servo_odo(hardwareMap);

        odo.sus();
        cleste1.close();
        cleste2.close();

        brat.setTargetPosition(0);
        brat.setPower(brat_power);
        conserva.carusel.setVelocity(400);
        sleep(200);
        conserva.carusel.setVelocity(0);


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {


            // Drive

            double direction = Math.atan2(-gp1.left_stick_y, gp1.left_stick_x) - Math.PI/2;
            double rotation = -gp1.right_stick_x;
            double speed = Math.sqrt(gp1.left_stick_x*gp1.left_stick_x + gp1.left_stick_y*gp1.left_stick_y);


            if(gp1.dpad_up){
                slow_mode = 2;
            }
            else if(gp1.left_bumper){
                slow_mode = 0;
            }
            else if(gp1.right_bumper){
                slow_mode = 1;
            }
            if(slow_mode == 0){
                setDrivePowers(direction, 0.43 * Math.pow(speed, 3.0), 0.35 * Math.pow(rotation, 3.0));
            }else if(slow_mode == 1){
                setDrivePowers(direction, 0.8 * Math.pow(speed, 3.0),0.7 * Math.pow(rotation, 3.0));
            }
            else if(slow_mode == 2){
                setDrivePowers(direction, 1 * Math.pow(speed, 3.0),1 * Math.pow(rotation, 3.0));
            }



            // Brat

            if(gp2.right_bumper && !brat_reset1){
                brat.setTargetPosition(brat.getCurrentPosition() + 100);
                brat_reset1 = true;
            }
            else{
                brat_reset1 = false;
            }
            if(gp2.left_bumper && !brat_reset2){
                brat.setTargetPosition(brat.getCurrentPosition() - 100);
                brat_reset2 = true;
            }
            else{
                brat_reset2 = false;
            }


            if(gp2.dpad_up && poz_outtake_urcare == 0){
                cleste1.close();
                cleste2.close();
                runtime_outtake.reset();
                poz_outtake_urcare = 1;
            }
            if(runtime_outtake.seconds() > 0.25 && poz_outtake_urcare == 1)
            {
                brat.setTargetPosition(brat_sus+outtake_error);
                brat.setPower(brat_power);
                poz_outtake_urcare = 0;
            }


            if(gp2.dpad_left && poz_outtake_urcare == 0){
                cleste1.close();
                cleste2.close();
                brat.setTargetPosition(brat_shared_jos);
                brat.setPower(brat_power);
            }


            if(gp2.dpad_right && poz_outtake_urcare == 0){
                cleste1.close();
                cleste2.close();
                brat.setTargetPosition(brat_shared);
                brat.setPower(brat_power);
            }


            if(gp2.dpad_down){
                cleste1.close();
                cleste2.close();
                brat.setTargetPosition(brat_jos);
                brat.setPower(brat_power);
            }

            if(gp2.a)
            {
                cleste1.hub();
                cleste2.hub();
            }
            if(gp2.x)
            {
                cleste2.open();
            }
            if(gp2.b)
            {
                cleste1.open();
            }



            // Intake

            if(gp2.left_trigger > 0.15 && gp2.right_trigger > 0.15){
                cleste1.open();
                cleste2.open();
                conserva.intake1.setVelocity(-2000*Math.min(gp2.right_trigger, intake_speed));
                conserva.intake2.setVelocity(-2000*Math.min(gp2.right_trigger, intake_speed));
                intake_state = 1;
                runtime_intake.reset();
            }
            else if(gp2.right_trigger > 0.15) {
                cleste1.open();
                cleste2.semi();
                conserva.intake1.setVelocity(2500*Math.min(gp2.right_trigger, intake_speed));
                intake_state = 1;
                runtime_intake.reset();
            }
            else if(gp2.left_trigger > 0.15) {
                cleste1.semi();
                cleste2.open();
                conserva.intake2.setVelocity(2500*Math.min(gp2.left_trigger, intake_speed));
                intake_state = 1;
                runtime_intake.reset();
            }
            else if(intake_state == 1){
                conserva.intake1.setVelocity(-2000);
                conserva.intake2.setVelocity(-2000);
                intake_state = 2;
            }
            if(intake_state == 2 && runtime_intake.seconds() > 0.3){
                cleste1.close();
                cleste2.close();
                intake_state = 3;
            }
            if(intake_state == 3 && runtime_intake.seconds() > 0.6){
                conserva.intake1.setPower(0);
                conserva.intake2.setPower(0);
                intake_state = 0;
            }



            // Ruleta

            double ruletaL = -gp2.right_stick_x;
            if(Math.abs(gp2.left_stick_x) > 0.2){
                ruletaY += gp2.left_stick_x/650;
                servoY.setPosition(ruletaY);
            }
            if(Math.abs(gp2.left_stick_y) > 0.2){
                ruletaZ += gp2.left_stick_y/1400;
                servoZ.setPosition(ruletaZ);
            }
            if(ruletaL > 0.2){
                servoL.setPower(-1);
            }
            else if(ruletaL < -0.2){
                servoL.setPower(1);
            }
            else{
                servoL.setPower(0);
            }
            if(gp2.right_stick_button)
            {
                ruletaY = 0.41;
                ruletaZ = 0.60;
                servoY.setPosition(ruletaY);
                servoZ.setPosition(ruletaZ);
            }
            if(gp2.left_stick_button)
            {
                ruletaY = 0.73;
                ruletaZ = 0.43;
                servoY.setPosition(ruletaY);
                servoZ.setPosition(ruletaZ);
            }



            // Carusel

            double rightTrigger = (gp1.right_trigger);
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
                conserva.carusel.setPower(power);
            } else {
                conserva.carusel.setPower(0);
                power = 0;
                timeCount = 0;
                time = 0;
            }



            // Telemetry

            telemetry.addData("intake1 ", conserva.intake1.getVelocity());
            telemetry.addData("intake2 ", conserva.intake2.getVelocity());
            telemetry.addData("carusel ", conserva.carusel.getVelocity());
            telemetry.update();

        }

    }


    /* Mecanum drive */
    public void setDrivePowers(double direction, double speed, double rotateSpeed){
        double directionRads = direction;

        double sin = Math.sin(Math.PI/4 - directionRads);
        double cos = Math.cos(Math.PI/4 - directionRads);

        conserva.lf.setPower(-root2 * speed * sin + rotateSpeed);
        conserva.rf.setPower(-root2 * speed * cos - rotateSpeed);
        conserva.lr.setPower(-root2 * speed * cos + rotateSpeed);
        conserva.rr.setPower(-root2 * speed * sin - rotateSpeed);
    }

    public void someRandomShit(){

        servoL = hardwareMap.get(CRServo.class, "servoL");
        servoY = hardwareMap.get(Servo.class, "servoY");
        servoZ = hardwareMap.get(Servo.class, "servoZ");
        servoL.setPower(0);
        servoY.setPosition(ruletaY);
        servoZ.setPosition(ruletaZ);


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