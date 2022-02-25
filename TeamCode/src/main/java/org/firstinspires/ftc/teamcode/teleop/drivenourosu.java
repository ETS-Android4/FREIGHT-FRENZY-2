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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;
import org.firstinspires.ftc.teamcode.hardware.servo_gheara;
import org.firstinspires.ftc.teamcode.hardware.servo_odo;

@Config
@TeleOp
//@Disabled
public class drivenourosu extends LinearOpMode {

    init_robot conserva = new init_robot();

    private double root2 = Math.sqrt(2.0);
    private boolean slow_mode = false;

    public int poz_outtake_urcare = 0;
    public int poz_outtake_coborare = 0;
    public boolean ok_intake = false;
    public boolean ok_intake2 = false;
    public boolean ok_outtake_error = false;
    public boolean ok_outtake_error2 = false;

    private ElapsedTime runtime_outtake = new ElapsedTime();
    private ElapsedTime runtime_outtake2 = new ElapsedTime();
    private ElapsedTime runtime_intake = new ElapsedTime();

    DcMotorEx brat = null;
    public static double brat_power = 0.6;
    public static double brat_power_incet = 0.28;
    public static int brat_sus = 730;
    public static int brat_jos = 0;
    public static int brat_jos_intake = -15;
    public int outtake_error = 0;

    public static double intake_speed = 0.8;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {


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

        brat.setTargetPosition(-15);
        brat.setPower(brat_power);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {

            /* gamepad 1 */

            /* Drive */

            double direction = Math.atan2(-gp1.left_stick_y, gp1.left_stick_x) - Math.PI/2;
            double rotation = -gp1.right_stick_x;
            double speed = Math.sqrt(gp1.left_stick_x*gp1.left_stick_x + gp1.left_stick_y*gp1.left_stick_y);

            if(gp1.left_bumper){
                slow_mode = true;
            }
            if(gp1.right_bumper){
                slow_mode = false;
            }
            if(slow_mode){
                setDrivePowers(direction, 0.43 * Math.pow(speed, 3.0), 0.35 * Math.pow(rotation, 3.0));
            }else{
                setDrivePowers(direction, 0.75 * Math.pow(speed, 3.0),0.55 * Math.pow(rotation, 3.0));
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
                poz_outtake_urcare = 2;
                outtake_error += 10;
            }
            if(runtime_outtake.seconds() > 0.75 && poz_outtake_urcare == 2)
            {
                brat.setPower(brat_power_incet);
                poz_outtake_urcare = 0;
            }


            if(gp2.dpad_left && poz_outtake_urcare == 0){
                cleste1.setServoPositions(0.08);
                cleste2.setServoPositions(0.77);
                runtime_outtake.reset();
                poz_outtake_urcare = 1;
            }
            if(runtime_outtake.seconds() > 0.35 && poz_outtake_urcare == 1)
            {
                brat.setTargetPosition(brat_sus+outtake_error);
                brat.setPower(brat_power);
                poz_outtake_urcare = 2;
                outtake_error += 5;
            }
            if(runtime_outtake.seconds() > 0.75 && poz_outtake_urcare == 2)
            {
                brat.setPower(brat_power_incet);
                poz_outtake_urcare = 0;
            }


            if(gp2.dpad_right && !ok_outtake_error)
            {
                outtake_error += 25;
                ok_outtake_error = true;
            }
            if(!gp2.dpad_right)
            {
                ok_outtake_error = false;
            }


            /*
            if(gp2.dpad_left && !ok_outtake_error2)
            {
                outtake_error -= 25;
                ok_outtake_error2 = true;
            }
            if(!gp2.dpad_left)
            {
                ok_outtake_error2 = false;
            }
             */


            if(gp2.dpad_down && poz_outtake_coborare == 0){
                cleste1.close();
                cleste2.semi();
                brat.setTargetPosition(brat_jos+150);
                brat.setPower(brat_power_incet+0.1);
                runtime_outtake2.reset();
                poz_outtake_coborare = 1;
            }
            if(runtime_outtake2.seconds() > 0.36 && poz_outtake_coborare == 1)
            {
                brat.setTargetPosition(brat_jos_intake);
                brat.setPower(brat_power_incet);
                poz_outtake_coborare = 0;
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


            if(gp2.left_trigger > 0.15 && gp2.right_trigger > 0.15){
                cleste1.open();
                //cleste2.open();
                conserva.intake1.setVelocity(-1500*Math.min(gp2.right_trigger, intake_speed));
                //conserva.intake2.setVelocity(-1000*Math.min(gp2.right_trigger, intake_speed));
                ok_intake = true;
                ok_intake2 = true;
                runtime_intake.reset();
            }
            else if(gp2.right_trigger > 0.15) {
                cleste1.open();
                cleste2.close();
                conserva.intake1.setVelocity(1200*Math.min(gp2.right_trigger, intake_speed));
                ok_intake = true;
                ok_intake2 = true;
                runtime_intake.reset();
            }
            else if(ok_intake){
                conserva.intake1.setVelocity(-1300);
                //conserva.intake2.setVelocity(-600);
            }
            if(ok_intake2 && runtime_intake.seconds()>0.3){
                cleste1.close();
                cleste2.close();
                ok_intake2 = false;
            }
            if(ok_intake && runtime_intake.seconds()>0.7){
                conserva.intake1.setVelocity(0);
                //conserva.intake2.setVelocity(0);
                ok_intake = false;
            }

            if(gp1.b){
                conserva.intake1.setVelocity(450);
                conserva.intake2.setVelocity(450);
                sleep(1420);
                conserva.intake1.setVelocity(1000);
                conserva.intake2.setVelocity(1000);
                sleep(260);
                conserva.intake1.setVelocity(0);
                conserva.intake2.setVelocity(0);
            }


            /* Telemetry */
            //telemetry.addData("slow_mode", slow_mode);
            //telemetry.addData("outtake", brat.getCurrentPosition());
            telemetry.addData("outtake compensation", outtake_error);
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