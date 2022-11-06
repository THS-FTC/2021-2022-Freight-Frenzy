/*
- 1 is first
Controls:
    Gamepad 1 (all random functions):
        left BUMPER BUTTON - reset all carousel and intake servos
        right BUMPER BUTTON - turn on the carousel motor
        left TRIGGER - intake forward
        right TRIGGER - intake backward
        x button - arm 1st position
        a button - arm 2nd position
        b button - arm 3rd position
    Gamepad 2 (drive):
        left analog stick - all controls that make logical
        right analog stick - radial turns in logical order and drift
        right BUMPER BUTTON - turn all motors off
Point sequence
    1st part:
        score freight to shipping hub level 3
    End Game:
        deliver all ducks
        fully park in warehouse
        if extra time:
            ***try to have shared shipping hub touching floor on our side
- Servo_3 = claw tilt
- Servo_4 = claw open/close
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.util.Date;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.vuforia.Vuforia;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp

public class teamTwoRed extends LinearOpMode {
    // Junction height values represented as motor encoder values for 4-stage Viper Slide Kit
    int groundJunction = 600;
    int lowJunction = 2900;
    int midJunction = 5400;
    int highJunction = 5400;
    double slideSpeed = 2800.0;//2787 PPR is max encoder PPR of Gobilda 435 rpm motor
    int armTarget = 0;//as encoder values
    int slidePosition = 0;
    boolean calibrated = false;

    // Hardware classes + names
    private Blinker Control_Hub;//NEEDED - DON'T DELETE!!
    private DcMotorEx Motor_1;//front left
    private DcMotorEx Motor_2;//front right
    private DcMotorEx Motor_3;//back left
    private DcMotorEx Motor_4;//back right
    private DcMotorEx armMotor;
    private Gyroscope imu;
    private Servo clawServo;

    // Motor variables for mecanum drive
    double armSpeed = 4000.0;
    double motor_reduction = 0.4;//for drivetrain
    double motor_1_pwr = 0.0;
    double motor_2_pwr = 0.0;
    double motor_3_pwr = 0.0;
    double motor_4_pwr = 0.0;
    double motor_denom;

    // Inputs
    double left_stick2_x; // triggers and bumpers
    double left_stick2_y;
    double right_stick2_x;
    /*
    double left_stick1_y;
    boolean left_bump1;
    boolean right_bump1;
    boolean left_bump2;
    boolean right_bump2;
    double left_trig1;
    double right_trig1;
    */
    double left_trig2;
    double right_trig2;
    boolean a1; //a,b,x,y buttons
    boolean b1;
    boolean x1;
    boolean y1;
    boolean a2;
    boolean b2;
    boolean x2;
    //boolean y2;
    //boolean dpad_left;
    //boolean dpad_down;
    //boolean dpad_up;
    //boolean dpad_right;

    @Override
    public void runOpMode() {
        //hardware initializing
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Motor_1 = hardwareMap.get(DcMotorEx.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotorEx.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotorEx.class, "Motor_3");
        Motor_4 = hardwareMap.get(DcMotorEx.class, "Motor_4");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");//to move the claw, grab the cone.
        //other things
        Motor_1.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_3.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_2.setDirection(DcMotorEx.Direction.FORWARD);
        Motor_4.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        /*Motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        telemetry.addData("Status", "Initialized");
        sleep(1500);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder of slideMotor when slide is fully retracted
        telemetry.addData("armEncoder", armMotor.getCurrentPosition());
        telemetry.update();
        /*armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(armSpeed);*/
        waitForStart();

        //main loop
        while (opModeIsActive()) {
            normal_motor();
            claw();
            team_2_arm();
            //telemetry.addData("motor1", left_stick2_y);
            telemetry.update();
        }

    }



    void normal_motor(){//normal motor control maths + telemetry
        right_stick2_x = -this.gamepad2.right_stick_x;
        left_stick2_x = this.gamepad2.left_stick_x;
        left_stick2_y = -this.gamepad2.left_stick_y;

//        if(right_bump1){
//            motor_reduction = 0.4;
//        }
//        else if(left_bump1){
//            motor_reduction = 0.2;
//        }

        //drivetrain
        telemetry.addData("lefty:", left_stick2_y);
        motor_denom = Math.max(Math.abs(left_stick2_y) + Math.abs(left_stick2_x) + Math.abs(right_stick2_x), 1.0);
        motor_1_pwr = (left_stick2_y + left_stick2_x + right_stick2_x)/motor_denom;//LF
        motor_2_pwr = (left_stick2_y - left_stick2_x - right_stick2_x)/motor_denom;//RF
        motor_3_pwr = (left_stick2_y - left_stick2_x + right_stick2_x)/motor_denom;//LB
        motor_4_pwr = (left_stick2_y + left_stick2_x - right_stick2_x)/motor_denom;//LR
        Motor_1.setPower(motor_1_pwr  * motor_reduction);
        Motor_2.setPower(motor_2_pwr  * motor_reduction);
        Motor_3.setPower(motor_3_pwr  * motor_reduction);
        Motor_4.setPower(motor_4_pwr  * motor_reduction);
        telemetry.addData("motor_1", motor_1_pwr);
        telemetry.addData("motor_2", motor_2_pwr);
        telemetry.addData("motor_3", motor_3_pwr);
        telemetry.addData("motor_4", motor_4_pwr);
        telemetry.addData("encoder-left", Motor_1.getCurrentPosition());
        telemetry.addData("encoder-mid", Motor_2.getCurrentPosition());
        telemetry.addData("encoder-right", Motor_3.getCurrentPosition());
        //telemetry.update();
    }

    void slideCalibrate(){
        armMotor.setPower(-0.75);
        // sleep(100);
        telemetry.addData("velocity", armMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.update();
        if (armMotor.getVelocity(AngleUnit.DEGREES) >= -0.05 && calibrated == false){
            calibrated = true;
            armMotor.setPower(0.0);
        }
        if (calibrated == true) {
            telemetry.addData("calibrated", true);
            telemetry.addData("armEncoder:", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    void claw(){
        a2 = this.gamepad1.a;
        x2 = this.gamepad1.x;
        if (a2 == true){
            clawServo.setPosition(30.0/270.0);//position for OPEN CLAW

        }
        else if (x2 == true){
            clawServo.setPosition(100.0/270.0);//position for CLOSED CLAW
        }
    }

    void team_2_arm(){//team 2 arm design with claw and 2 servos
        left_trig2 = this.gamepad1.left_trigger;
        right_trig2 = this.gamepad1.right_trigger;
        telemetry.addData("righttrig", right_trig2);
        if(left_trig2 > 0.0 && armTarget > 0){
            armTarget -= 20;
            armMotor.setTargetPosition(armTarget);
        }
        if(right_trig2 > 0.0 && armTarget <= 3000){
            armTarget += 20;
            armMotor.setTargetPosition(armTarget);
            telemetry.addData("targetpos", armMotor.getTargetPosition());
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(armSpeed);
        telemetry.addData("armPos: ", armTarget);
        //telemetry.update();
    }
}