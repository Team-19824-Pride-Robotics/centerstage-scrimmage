package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


@TeleOp(group = "drive")
@Config
public class drive extends OpMode {




public static double sPos = .1;
    private DcMotorEx RF;
    private DcMotorEx RB;
    private DcMotorEx LF;
    private DcMotorEx LB;
    private DcMotorEx winch;

    Servo servo1;

    public static int extend = 1600;
    public static int retract = 300;

    public static int start = -800;

    public static double winchup = .5;
    public static double winchdown = 1;






    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo1 = hardwareMap.get(Servo.class, "servo");

        winch = hardwareMap.get(DcMotorEx.class, "winch");

        //drive
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB= hardwareMap.get(DcMotorEx.class, "RB");
        LF= hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");

        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);

        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





    }

    @Override
    public void loop() {



        /*///////
        VARIABLE
        *////////


        double drive = -gamepad1.left_stick_y;
        double rotatel = gamepad1.right_stick_x;
        double rotater = -gamepad1.right_stick_x;

        //gamepad
        double d_power = .5;
        //strafe
        double s_power = 1 * gamepad1.right_trigger * gamepad1.left_trigger;



        /*////////////////
        DRIVER 1 CONTROLS
        */////////////// /q

        //drive

        RF.setPower(drive + rotatel);
        RB.setPower(drive + rotatel);
        LF.setPower(drive + rotater);
        LB.setPower(drive + rotater);


        //trigger strafe control
        if (gamepad1.right_trigger<.1) {
            RF.setPower(-s_power);
            RB.setPower(s_power);
            LF.setPower(s_power);
            LB.setPower(-s_power);
        }
        if (gamepad1.left_trigger<.1) {
            RF.setPower(-s_power);
            RB.setPower(s_power);
            LF.setPower(s_power);
            LB.setPower(-s_power);
        }
// d_pad drive control
        if (gamepad1.dpad_up) {
            RF.setPower(d_power);
            RB.setPower(d_power);
            LF.setPower(d_power);
            LB.setPower(d_power);
        }
        if (gamepad1.dpad_down) {
            RF.setPower(-d_power);
            RB.setPower(-d_power);
            LF.setPower(-d_power);
            LB.setPower(-d_power);
        }
        if (gamepad1.dpad_left) {
            RF.setPower(d_power);
            RB.setPower(-d_power);
            LF.setPower(-d_power);
            LB.setPower(d_power);
        }
        if (gamepad1.dpad_right) {
            RF.setPower(-d_power);
            RB.setPower(d_power);
            LF.setPower(d_power);
            LB.setPower(-d_power);
        }

        if (gamepad2.y) {
            winch.setTargetPosition(extend);
            winch.setPower(winchup);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.a) {
            winch.setTargetPosition(retract);

            winch.setPower(winchdown);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        if (gamepad2.b) {
            winch.setTargetPosition(start);
            winch.setPower(winchdown);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }

        if (gamepad2.x){
            extend += 100;
        }
        if (gamepad2.dpad_up){
            extend -= 100;
        }


if (gamepad2.dpad_down) {
     servo1.setPosition(sPos);
}

telemetry.addData("elevpos", winch.getCurrentPosition());
        telemetry.addData("extend pos", extend);



    }


}