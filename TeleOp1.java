package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "TeleOp 1.0")
public class TeleOp1 extends LinearOpMode {
    private DcMotor tlm, trm, blm, brm, slideMotor;

    int slideStatus = 0; //0 is neutral, -1 is down, +1 is up
    private double basisAngle = 0;
    private int servoState = 2;
    private int armState = 5;
    private Servo armServo;
    private double bS = 0.93;
    private double S = 0.95;//changes based on whatever the driver likes go form 0.95 to 1 range exclusive
    double ticks = 0;
    private double power = 0.0;
    int st = 0;
    private DigitalChannel maxLimitTouch;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    private int x = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //hex motor 20:1 gearbox -> linear slide; touch sensor for max height

        armServo = hardwareMap.get(Servo.class, "armServo");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        tlm = hardwareMap.get(DcMotor.class, "frontLeft");
        trm = hardwareMap.get(DcMotor.class, "frontRight");
        blm = hardwareMap.get(DcMotor.class, "backLeft");
        brm = hardwareMap.get(DcMotor.class, "backRight");

        blm.setDirection(DcMotorSimple.Direction.REVERSE);

        tlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        armServo.scaleRange(0.0, 0.8);
        armServo.setDirection(Servo.Direction.REVERSE);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.02 || gamepad1.left_trigger > 0.02) { //left == ccw rotation, right == cw rotation
                int absR = 0;

                if (gamepad1.right_trigger > 0) {
                    power = Math.abs(gamepad1.right_trigger)*3/4;
                } else if (gamepad1.left_trigger > 0) {
                    power = -Math.abs(gamepad1.left_trigger)*3/4;
                }

                trm.setPower(-power);   // abs -> (+) => + power to trm (should be -, refer to cmt above)
                brm.setPower(-power);   // abs -> (+) => + power to brm
                tlm.setPower(power);   // abs -> (+) => - power to tlm (keep in mind left is being fed reverse, so +reverse is -)
                blm.setPower(power);  // abs -> (+) => + power to blm (keep in mind left is being fed reverse, so -reverse is +)
                st = 0;
            } else {//only when there's no response on the triggers

                //so calc for sign isn't undefined, and also falls out of +/- 0.02 margin for x adjustment of joystick
                if (Math.abs(gamepad1.right_stick_x) != 0) {
                    power = (S * power + (1 - S) * gamepad1.right_stick_x);
                    st = -1;
                    trm.setPower(-power);   // abs -> (+) => + power to trm (should be -, refer to cmt above)
                    brm.setPower(power);  // abs -> (+) => - power to brm
                    tlm.setPower(power);   // abs -> (+) => - power to tlm (keep in mind left is being fed reverse, so -reverse is +)
                    blm.setPower(-power);   // abs -> (+) => - power to blm (keep in mind left is being fed reverse, so -reverse is +)
                }

                //so calc for sign isn't undefined, also falls out of +/- 0.02 margin for y adjustment of joystick
                else if (Math.abs(gamepad1.left_stick_y) != 0) {
                    power = (S * power + (1 - S) * gamepad1.left_stick_y);
                    st = 1;
                    trm.setPower(-power);    // abs -> (+) => + power to trm
                    brm.setPower(-power);     // abs -> (+) => + power to brm
                    tlm.setPower(-power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
                    blm.setPower(-power);     // abs -> (+) => + power to blm (keep in mind left if being fed reverse, so +reverse is -)

                } else { //accounts for dead margin (x == 0)
                    if (st == -1) { //implies that the last movement was horizontal - slowly tapers off
                        power = (bS * power + (1 - bS) * gamepad1.right_stick_x);
                        st = -1;
                        trm.setPower(-power);
                        brm.setPower(power);
                        tlm.setPower(power);
                        blm.setPower(-power);
                    }
                    if (st == 1) { //implies that the last movement was vertical - slowly tapers off
                        power = (bS * power + (1 - bS) * gamepad1.left_stick_y);
                        st = 1;
                        trm.setPower(-power);
                        brm.setPower(-power);
                        tlm.setPower(-power);
                        blm.setPower(-power);
                    }
                    if (st == 0) {
                        trm.setPower(0);
                        brm.setPower(0);
                        tlm.setPower(0);
                        blm.setPower(0);
                    }
                }
            }
            if (gamepad2.x) {//Carry position
                slideMotor.setTargetPosition((int) (COUNTS_PER_INCH * 2.5 / 2));
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);

                if (slideMotor.getCurrentPosition() > (int) (COUNTS_PER_INCH * 2.5) / 2) {
                    slideMotor.setPower(0);
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (gamepad2.a) {//Low pole accounting for cone height + inches of ground
                slideMotor.setTargetPosition((int) (COUNTS_PER_INCH) * (17 / 2 ));
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);
                if (slideMotor.getCurrentPosition() > (int) (COUNTS_PER_INCH * (17 / 2 ))) {
                    slideMotor.setPower(0);
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (gamepad2.b) {//Medium pole 23.5 inch plus other factors
                slideMotor.setTargetPosition((int) (COUNTS_PER_INCH * (27/2)));
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);

                if (slideMotor.getCurrentPosition() > (int) (COUNTS_PER_INCH * (27/2))) {
                    slideMotor.setPower(0);
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            else if(gamepad2.y){
                slideMotor.setTargetPosition((int) (COUNTS_PER_INCH * (17)));
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);

                if (slideMotor.getCurrentPosition() > (int) (COUNTS_PER_INCH * (17))) {
                    slideMotor.setPower(0);
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (gamepad2.dpad_down){
                //either midway up or at its desination
                slideMotor.setTargetPosition(2);
                slideMotor.setPower(-0.5);
                if (slideMotor.getCurrentPosition() <= 2) {
                    slideMotor.setPower(0);
                    slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }
            }
            else if(gamepad2.dpad_up) {
                slideMotor.setTargetPosition((int) (COUNTS_PER_INCH * 0.5 + slideMotor.getCurrentPosition()));
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.1);
            }


            if (gamepad2.left_bumper) {
                telemetry.addLine("servoClose");
                telemetry.update();
                armServo.setPosition(1.0);
            } else {
                telemetry.addLine("servoOpen");
                telemetry.update();
                armServo.setPosition(0.1);
            }


            telemetry.addData("MotorEncoder", slideMotor.getCurrentPosition());
            telemetry.addData("x: ", x);
            telemetry.update();
            telemetry.addLine("encoder count of top right motor " + trm.getCurrentPosition());
            telemetry.addLine("cumulative wheel powers (tr, tl, br, bl): " + trm.getPower() + ", " + tlm.getPower() + ", " + brm.getPower() + ", " + blm.getPower());
            telemetry.addLine("servo value: " + armServo);
            telemetry.update();

            sleep(20);

        }

    }

}
