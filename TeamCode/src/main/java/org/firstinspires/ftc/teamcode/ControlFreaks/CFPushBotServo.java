//package org.firstinspires.ftc.teamcode.ControlFreaks;
//
///**
// * Created by adevries on 11/6/2015.
// */
//
//import android.util.Log;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyBlock;
//import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyBlockList;
//import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyCamera;
//
//import java.util.ArrayList;
//import java.util.Calendar;
//import java.util.List;
//
////import com.qualcomm.ftcrobotcontroller.opmodes.ColorSensorDriver;
//
//public class CFPushBotServo {
//
//
//    private String config_servo_name = "digger";
//
//    private Servo v_servo = null;
//
//    //Global Vars to the class
//    private static final double ServoErrorResultPosition = -0.0000000001;
//
//    LinearOpMode opMode;
//
//    /**
//     * Indicate whether a message is a available to the class user.
//     */
//    private boolean v_warning_generated = false;
//    private boolean v_error_generated = false;
//    /**
//     * Store a message to the user if one has been generated.
//     */
//    private String v_warning_message = "No Errors";
//    private String v_error_message = "No Errors";
//
//    public CFPushBotServo()
//    {
//    }
//
//
//
//    //--------------------------------------------------------------------------
//    //
//    // init
//    //
//    /**
//     * Perform any actions that are necessary when the OpMode is enabled.
//     *
//     * The system calls this member once when the OpMode is enabled.
//     */
//    public void init (LinearOpMode ahOpmode, String configName)
//
//    {
//        //
//        // Use the hardwareMap to associate class members to hardware ports.
//        //
//        // Note that the names of the devices (i.e. arguments to the get method)
//        // must match the names specified in the configuration file created by
//        // the FTC Robot Controller (Settings-->Configure Robot).
//
//        opMode = ahOpmode;
//        config_servo_name = configName;
//        v_warning_generated = false;
//        v_warning_message = "Can't map; ";
//        v_error_generated = false;
//        v_error_message = "Can't map; ";
//        //
//        // Connect the blockgrabber servo.
//        //
//        try
//        {
//            v_servo = opMode.hardwareMap.servo.get(config_servo_name);
//            //v_servo.scaleRange(v_servo_blockslide_MinPosition,v_servo_blockslide_MaxPosition);
//            v_servo.setDirection(v_servo_direction);
//            v_servo.setPosition (v_servo_InitPosition);
//        }
//        catch (Exception p_exeception)
//        {
//            debugLogException(config_servo_name, "missing", p_exeception);
//            v_servo = null;
//        }
//
//    } // init
//
//    private double  v_servo_InitPosition = 0.0;
//    void setInitPosition(double initPosition) {
//        v_servo_InitPosition = initPosition;
//    }
//
//
//    private Servo.Direction v_servo_direction = Servo.Direction.FORWARD;
//    void setDirection(Servo.Direction servoDirection) {
//        v_servo_direction = servoDirection;
//        v_servo.setDirection(v_servo_direction);
//    }
//
//    void debugLogException(String type, String msg, Exception ex){
//        if (ex != null){
//            m_warning_message(type);
//        }
//        String debugMessage = type + ":" + msg;
//        if (ex != null) {
//            String errMsg = ex.getMessage();
//            if (errMsg != null) {
//                debugMessage = debugMessage + errMsg;
//            }
//
//            String stackTrace = Log.getStackTraceString(ex);
//            if (stackTrace != null) {
//                debugMessage = debugMessage + "\n" + stackTrace;
//            }
//        }
//        if (v_debug) {
//            Log.e("CFPushBotHardware", debugMessage);
//        }
//    }
//
//    void debugPrint(String msg){
//        String debugMessage = msg;
//        Log.d("CFPushBotHardware", debugMessage);
//    }
//
//    void warningPrint(String msg){
//        Log.w("CFPushBotHardware", msg);
//    }
//
//
//
//    private ElapsedTime v_timewait2_elapsedtime;
//    private boolean v_is_timewaiting2_complete;
//    private int v_timewait2_milliseconds;
//
//    private ElapsedTime v_timewait_elapsedtime;
//    private boolean v_is_timewaiting_complete;
//    private float v_timewait_seconds;
//
//    public boolean timewait(float seconds){
//        v_timewait_elapsedtime = new ElapsedTime();
//        v_timewait_seconds = seconds;
//        v_is_timewaiting_complete = false;
//        return true;
//    }
//
//    public boolean timewait_Complete(){
//        if ( v_timewait_elapsedtime.seconds() > v_timewait_seconds ){
//            v_is_timewaiting_complete = true;
//        }
//        return v_is_timewaiting_complete;
//    }
//
//    private boolean timewait2Milliseconds(int milliseconds){
//        v_timewait2_elapsedtime = new ElapsedTime();
//        v_timewait2_milliseconds = milliseconds;
//        v_is_timewaiting2_complete = false;
//        return true;
//    }
//
//    private boolean timewait2Milliseconds_Complete(){
//        if ( v_timewait2_elapsedtime.milliseconds() >= v_timewait2_milliseconds ){
//            v_is_timewaiting2_complete = true;
//        }
//        return v_is_timewaiting2_complete;
//    }
//
//
//    private Calendar v_loop_previous_timestamp;
//    private long v_slow_loop_milliseconds = 0;
//    public void hardware_loop() throws InterruptedException{
//        //jewel ease so not to hard hit ground
//            if(v_servo_jewel_is_retracting){
//                if( v_servo_jewel_position > v_servo_jewel_MinPosition){
//                    v_servo_jewel_position = v_servo_jewel_position - .05;
//                    v_servo_jewel.setPosition(v_servo_jewel_position);
//                }else{
//                    v_servo_jewel.setPosition(v_servo_jewel_MinPosition);
//                    v_servo_jewel_is_retracting = false;
//                }
//            }
//
//
//            if(v_debug || v_zeromessage_set || v_errormessage_set) {
//                update_telemetry();
//                opMode.updateTelemetry(opMode.telemetry);
//            }
//        }
//
//        opMode.idle();
//        //waitForTick(5);
//    }
//
//    public long hardware_loop_slowtime_milliseconds(){
//        return v_slow_loop_milliseconds;
//    }
//
//    public void hardware_stop(){
//
//    }
//
//
//
//
//
//    public void extend () throws InterruptedException
//    {
//        if (v_motor_extender != null)
//        {
//            v_extender_state = 0;
//            v_motor_extender_Position = v_motor_extender_Position + v_motor_extender_encoder_max - v_motor_extender_ExtendSlowdownTicks;
//
//            v_motor_extender.setTargetPosition(v_motor_extender_Position);
//            set_second_message("extendinging extender");
//            v_motor_extender.setPower(v_motor_extender_power);
//        }
//    }
//
//    private int v_extender_state = 0;
//    public boolean extend_complete () {
//        if (v_motor_extender != null) {
//            switch (v_extender_state) {
//                case 0:
//                    if (v_motor_extender.isBusy() == false) {
//                        v_motor_extender_Position = v_motor_extender_Position + v_motor_extender_ExtendSlowdownTicks;
//                        v_motor_extender.setPower(0.0F);
//                        v_motor_extender.setTargetPosition(v_motor_extender_Position);
//                        set_second_message("slider almost extended");
//                        v_motor_extender.setPower(0.5F);
//
//                        v_extender_state++;
//                    }
//                    break;
//                case 1:
//                    if (v_motor_extender.isBusy() == false) {
//                        v_motor_slider.setPower(0.0F);
//                        v_slider_isExtended = true;
//                        set_second_message("slider loaded");
//                        return true;
//                    }
//                    break;
//            }
//            return false;
//
//        }else{
//            return true;
//        }
//    }
//
//
//    //Retract the slider
//
//    //--------------------------------------------------------------------------
//    //
//    // Retract_slider
//    //
//    /**
//     * Retract the slider
//     */
//    private boolean v_slider_isExtended = false;
//    public void slider_step (int stepAmount, boolean min2Override)
//    {
//        try{
//            if (v_motor_slider != null )
//            {
//                int cPosition = v_motor_slider.getCurrentPosition();
//                v_motor_slider_Position =  cPosition + stepAmount;
//                if(min2Override ==false && v_motor_slider_Position < v_motor_slider_encoder_min2 && cPosition > v_motor_slider_encoder_min2  ){
//                    v_motor_slider_Position = v_motor_slider_encoder_min2;
//
//                }else if(v_motor_slider_Position < v_motor_slider_encoder_min && min2Override ==true){
//                    v_motor_slider_Position = v_motor_slider_encoder_min;
//
//                }else if(v_motor_slider_Position > v_motor_slider_encoder_max) {
//                    v_motor_slider_Position = v_motor_slider_encoder_max;
//                }
//
//                v_motor_slider.setTargetPosition(v_motor_slider_Position);
//                v_motor_slider.setPower(v_motor_slider_power);
//            }
//            set_second_message("Slider Step " + v_motor_slider_Position + " minOverride:" + min2Override);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("slider_step", "error", p_exeception);
//        }
//    }
//    public void slider_stop (boolean min2Override)
//    {
//        try{
//            if (v_motor_slider != null )
//            {
//                v_motor_slider.setPower(0);
//                v_motor_slider_Position = v_motor_slider.getCurrentPosition();
//                if(min2Override ==false && v_motor_slider_Position < v_motor_slider_encoder_min2 && v_motor_slider_Position > v_motor_slider_encoder_min2  ){
//                    v_motor_slider_Position = v_motor_slider_encoder_min2;
//
//                }else if(v_motor_slider_Position < v_motor_slider_encoder_min && min2Override ==true){
//                    v_motor_slider_Position = v_motor_slider_encoder_min;
//
//                }else if(v_motor_slider_Position > v_motor_slider_encoder_max) {
//                    v_motor_slider_Position = v_motor_slider_encoder_max;
//                }
//            }
//            set_second_message("Slider Stop " + v_motor_slider_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("slider_stop", "error", p_exeception);
//        }
//    }
//
//    public void slider_hold ()
//    {
//        try{
//            if (v_motor_slider != null )
//            {
//                v_motor_slider.setTargetPosition(v_motor_slider_Position);
//                v_motor_slider.setPower(.2);
//            }
//            set_second_message("Slider Stop " + v_motor_slider_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("slider_stop", "error", p_exeception);
//        }
//    }
//
//    public void slider_retract ()
//    {
//
//        if (v_motor_slider != null )
//        {
//
//            v_motor_slider.setTargetPosition(v_motor_slider_encoder_min);
//            set_second_message("Retracting slider");
//            v_motor_slider.setPower(v_motor_slider_power);
//        }
//
//    }
//    public boolean slider_retract_complete ()
//    {
//        if (v_motor_slider != null )
//        {
//
//            if (v_motor_slider.isBusy()== false) {
//                v_motor_slider.setPower(0.0F);
//                v_slider_isExtended = false;
//                set_second_message("Retracted slider");
//                return true;
//            }
//        }
//        return false;
//    }
//
//
//    public void lifter_extend () throws InterruptedException
//    {
//        if (v_motor_lifter != null)
//        {
////            if (v_lifter_isExtended == true){
////                set_second_message("lifter already extended");
////                return;
////            }
//            v_lifter_state = 0;
//            v_motor_lifter_Position = v_motor_lifter_Position + v_motor_lifter_encoder_max - v_motor_lifter_ExtendSlowdownTicks;
//
//            v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//            set_second_message("extendinging lifter");
//            v_motor_lifter.setPower(v_motor_lifter_power);
//
//        }
//    }
//
//    private int v_lifter_state = 0;
//    public boolean lifter_extend_complete () {
//        if (v_motor_lifter != null) {
//            switch (v_lifter_state) {
//                case 0:
//                    if (v_motor_lifter.isBusy() == false) {
//                        v_motor_lifter_Position = v_motor_lifter_Position + v_motor_lifter_ExtendSlowdownTicks;
//                        v_motor_lifter.setPower(0.0F);
//                        v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//                        set_second_message("lifter almost extended");
//                        v_motor_lifter.setPower(0.5F);
//
//                        v_lifter_state++;
//                    }
//                    break;
//                case 1:
//                    if (v_motor_lifter.isBusy() == false) {
//                        v_motor_lifter.setPower(0.0F);
//                        v_lifter_isExtended = true;
//                        set_second_message("lifter loaded");
//                        return true;
//                    }
//                    break;
//            }
//            return false;
//
//        }else{
//            return true;
//        }
//    }
//
//
//
//    /**
//     * Retract the slider
//     */
//    private boolean v_lifter_isExtended = false;
//    public void lifter_step (int stepAmount)
//    {
//        try{
//            if (v_motor_lifter != null )
//            {
//                v_motor_lifter_Position = v_motor_lifter.getCurrentPosition() + stepAmount;
//                if(v_motor_lifter_Position <= v_motor_lifter_encoder_min  ) {
//                    v_motor_lifter_Position = v_motor_lifter_encoder_min;
//                }
//                if(v_motor_lifter_Position >= v_motor_lifter_encoder_max  ) {
//                    v_motor_lifter_Position = v_motor_lifter_encoder_max;
//                }
//                if(v_motor_lifter_Position >= v_motor_lifter_encoder_min && v_motor_lifter_Position <= v_motor_lifter_encoder_max ) {
//                    v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//                    v_motor_lifter.setPower(v_motor_lifter_power);
//                }
//            }
//            set_second_message("lifter Step " + v_motor_lifter_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("lifter_step", "error", p_exeception);
//        }
//    }
//
//    public void lifter_stop ()
//    {
//        try{
//            if (v_motor_lifter != null )
//            {
//                v_motor_lifter.setPower(0);
//                v_motor_lifter_Position = v_motor_lifter.getCurrentPosition();
//                v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//            }
//            set_second_message("lifter Stop " + v_motor_lifter_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("lifter_stop", "error", p_exeception);
//        }
//    }
//
//    public void lifter_testbot_reverse(){
//        try{
//            if (v_motor_lifter != null )
//            {
//                v_motor_lifter.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            }
//            set_second_message("lifter testbot Reverse ");
//        }catch (Exception p_exeception)
//        {
//            debugLogException("lifter_testbot_reverse", "error", p_exeception);
//        }
//    }
//
//    public void lifter_stepmin(int steps){
//        v_motor_lifter_encoder_min = v_motor_lifter_encoder_min + steps;
//    }
//
//    public void lifter_retract ()
//    {
//
//        if (v_motor_lifter != null )
//        {
////            if (v_lifter_isExtended == false){
////                set_second_message("lifter not loaded");
////                return;
////            }
//            v_motor_lifter.setTargetPosition(v_motor_lifter_encoder_min);
//            set_second_message("Retracting lifter");
//            v_motor_lifter.setPower(v_motor_lifter_power);
//        }
//
//    }
//    public boolean lifter_retract_complete ()
//    {
//        if (v_motor_lifter != null )
//        {
//
//            if (v_motor_lifter.isBusy()== false) {
//                v_motor_lifter.setPower(0.0F);
//                v_lifter_isExtended = false;
//                set_second_message("Retracted lifter");
//                return true;
//            }
//        }
//        return false;
//    }
//
//
//
//    public void blockgrabber_toggle ()
//    {
//        try {
//            if (v_servo_blockgrabber_is_extended == true){
//                blockgrabber_close();
//            }else{
//                blockgrabber_open();
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockgrabber_toggle", "error", p_exeception);
//        }
//    }
//
//
//
//
//    //Below are The Telmetry Code to Write Debug to the Phones
//
//    String secondMessage = "N/A";
//
//
//
//}