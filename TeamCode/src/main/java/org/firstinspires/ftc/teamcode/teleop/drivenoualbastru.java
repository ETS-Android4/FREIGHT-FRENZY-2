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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_brat;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;
import org.firstinspires.ftc.teamcode.hardware.servo_odo;

@Config
@TeleOp
//@Disabled
public class drivenoualbastru extends LinearOpMode {

    init_robot conserva = new init_robot();

    private double root2 = Math.sqrt(2.0);
    private boolean slow_mode = false;
    private boolean reverse_intake = false;

    public boolean ok_intake = false;
    public boolean ok = false;

    private ElapsedTime runtime = new ElapsedTime();

    public static double intake_speed = 0.4;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {


        conserva.init(hardwareMap);
        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;

        servo_brat brat = new servo_brat(hardwareMap);
        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);
        servo_odo odo = new servo_odo(hardwareMap);

        odo.sus();
        brat.jos();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {

            /* gamepad 1 */

            /* Drive */

            double direction = Math.atan2(gp1.left_stick_y, -gp1.left_stick_x) - Math.PI/2;  //-y +x
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
                setDrivePowers(direction, 0.7 * Math.pow(speed, 3.0),0.45 * Math.pow(rotation, 3.0));
            }

            /*
            if(gp2.dpad_right){
                outtake.setTargetPosition((int)outtake_mijl);
                outtake.setVelocity(outtake_velo);
            }
            if(gp2.dpad_down){
                outtake.setTargetPosition((int)outtake_jos);
                outtake.setVelocity(outtake_velo);
            }

            if(gp2.dpad_left){
                conserva.intake1.setVelocity(0);
                runtime.reset();
                ok = true;
            }

            if(runtime.seconds() > 0.5 && ok)
            {
                ok = false;
                ok2 = true;
                brat.jos();
                cleste1.close();
                cleste2.close();
                runtime.reset();
            }

            if(runtime.seconds() > 0.5 && ok2)
            {
                ok2 = false;
                outtake.setTargetPosition(down_pos);
            }
            */

            if(gp2.y){
                brat.sus();
            }
            if(gp2.b){
                brat.second();
            }
            if(gp2.a){
                brat.first();
            }
            if(gp2.x)
            {
                cleste1.open();
                cleste2.open();
            }
            if(gp2.dpad_down){
                cleste1.open();
                cleste2.open();
                brat.kindajos();
                runtime.reset();
                ok = true;
            }
            if(runtime.seconds() > 1.05 && ok)
            {
                brat.jos();
                ok = false;
            }
            /*
            if(gp2.a && gp2.b)
            {
                outtake.setTargetPosition(-100);
                outtake.setVelocity(1000);
                sleep(250);
                resetOuttakeEncoder();
                sleep(250);
            }
            */

            if(gp2.left_trigger > 0.1 && gp2.right_trigger > 0.1){
                cleste1.open();
                cleste2.open();
                conserva.intake1.setVelocity(-1500*Math.min(gp2.right_trigger, intake_speed));
                conserva.intake2.setVelocity(-1500*Math.min(gp2.right_trigger, intake_speed));
                ok_intake = true;
                runtime.reset();
            }
            else if(gp2.right_trigger > 0.1) {
                cleste1.open();
                cleste2.semi();
                conserva.intake1.setVelocity(1500*Math.min(gp2.right_trigger, intake_speed));
                ok_intake = true;
                runtime.reset();
            }
            else if(gp2.left_trigger > 0.1) {
                cleste1.semi();
                cleste2.open();
                conserva.intake2.setVelocity(1500*Math.min(gp2.left_trigger, intake_speed));
                ok_intake = true;
                runtime.reset();
            }
            else if(ok_intake){
                conserva.intake1.setVelocity(-870);
                conserva.intake2.setVelocity(-870);
            }
            else if(ok_intake && runtime.seconds() > 0.6){
                conserva.intake1.setVelocity(0);
                conserva.intake2.setVelocity(0);
                cleste1.close();
                cleste2.close();
                ok_intake = false;
            }

            if(gp1.b){
                conserva.intake1.setVelocity(-440);
                conserva.intake2.setVelocity(440);
                sleep(1430);
                conserva.intake1.setVelocity(-1400);
                conserva.intake2.setVelocity(1400);
                sleep(250);
                conserva.intake1.setVelocity(0);
                conserva.intake2.setVelocity(0);
            }


            /* Telemetry */
            telemetry.addData("slow_mode", slow_mode);
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

}