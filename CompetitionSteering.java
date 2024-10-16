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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CompetitionSteering")

public class CompetitionSteering extends OpMode{

    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;
    public DcMotor  leftArm     = null;
    public DcMotor  rightArm    = null;
    public DcMotor  frontArm    = null;
    public Servo    armServo    = null;
    public CRServo  clawServo   = null;
    
//Hudson Lucas touch this variable and youre demoted.
double Pow = 0.5;

    @Override
    public void init() {
        
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear    = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear   = hardwareMap.get(DcMotor.class, "rightRear");
        leftArm     = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm    = hardwareMap.get(DcMotor.class, "rightArm");
        frontArm    = hardwareMap.get(DcMotor.class,"frontArm");
        armServo    = hardwareMap.get(Servo.class,"armServo");
        clawServo   = hardwareMap.get(CRServo.class,"clawServo");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        frontArm.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData(">", "Robot is READY TO RRRRRUMBLE.  Press Play if you so please. Hudson smells like cheese.");    //

    }

    @Override
    public void loop() {
        double left;
        double right;
        double left2;
        double right2;
        double clawPos;
 

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        left2 = -gamepad2.left_stick_y * 0.1 * 8 +0.06/*value8rotation*/;
        right2 = -gamepad2.right_stick_y * 0.1 * 8/*value8rotation*/;
        leftFront.setPower(left);
        rightFront.setPower(right);
        leftRear.setPower(left);
        rightRear.setPower(right);
        leftArm.setPower(left2);
        rightArm.setPower(left2);
        frontArm.setPower(right2);
        armServo.setPosition(0.0); 
        clawServo.setPower(0.0);
        if (gamepad1.left_bumper){
            leftFront.setPower(-1.0) ;
            rightFront.setPower(1.0) ;
            leftRear.setPower(1.0) ;
            rightRear.setPower(-1.0) ;} //strafe right
        else if (gamepad1.right_bumper){
            leftFront.setPower(1.0) ;
            rightFront.setPower(-1.0) ;
            leftRear.setPower(-1.0) ;
            rightRear.setPower(1.0) ;} //strafe left
        else if (gamepad1.x){
            leftFront.setPower(-1.0) ;
            rightFront.setPower(1.0) ;
            leftRear.setPower(-1.0) ;
            rightRear.setPower(1.0) ;} //rotate counterclockwise
        else if (gamepad1.b){
            leftFront.setPower(1.0) ;
            rightFront.setPower(-1.0) ;
            leftRear.setPower(1.0) ;
            rightRear.setPower(-1.0) ;} //rotate clockwise
        else if (gamepad1.y){
            leftFront.setPower(0.5) ;
            rightFront.setPower(0.5) ;
            leftRear.setPower(0.5) ;
            rightRear.setPower(0.5) ;} //slow mode forward
        else if (gamepad1.a){
            leftFront.setPower(-0.5) ;
            rightFront.setPower(-0.5) ;
            leftRear.setPower(-0.5) ;
            rightRear.setPower(-0.5) ;} //slow mode backward
/*————————————————————————Gamepad 2—————————————————————————————————————————*/
     /*   if (gamepad2.x) {
            armServo.setPosition(1);}
        else if (gamepad2.b) {
            armServo.setPosition(0);}
        else {
            armServo.setPosition(0.5);}  //servo 2 
            */
            
        if(gamepad2.right_bumper){clawServo.setPower(0.8);}
        else if (gamepad2.left_bumper){clawServo.setPower(-0.8);}
        else{clawServo.setPower(0);}
        
        
        
       if (gamepad2.x) {
            Pow += 0.1;  // Increase Pow
            Pow = Range.clip(Pow, 0.0, 0.3);  // Keep Pow within valid servo range
            armServo.setPosition(Pow);
        }
        if (gamepad2.b) {
            Pow -= 0.1;  // Decrease Pow
            Pow = Range.clip(Pow, 0.0, 0.3);  // Keep Pow within valid servo range
            armServo.setPosition(Pow);
        }

        if (gamepad2.dpad_down) {
            frontArm.setPower(0.4);}
        else if (gamepad2.dpad_up) {
            frontArm.setPower(-0.4);}
            else{   frontArm.setPower(-0.09);}
            
   /* if (gamepad2.dpad_right) {
  rightArm.setPower(0.1);
        frontArm.setPower(0.1);}
            else{         
                
                                   leftArm.setPower(left2);
        rightArm.setPower(right2);
            }*/
            
            
        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("armPos", "%.2f", left2);
        telemetry.addData("Servo Position", "Pow: %.2f", Pow);  // Display Pow on telemetry

    }

    @Override
    public void stop() {
    }
}
