package org.firstinspires.ftc.teamcode.ControlFreaks;

//import com.qualcomm.ftccommon.DbgLog;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Calendar;

/**
 * Created by adevries on 11/16/2015.
 */
public class AdafruitLEDBackpack7Seg {
    static final String logId   = "Ada7Seg:";       // Tag identifier in FtcRobotController.LogCat
    private String v_deviceName = "Not Set";
    private byte I2CAddress = 0x70;
    private Wire v_ledseg;
    private static final byte HT16K33_BLINK_CMD = (byte) 0x80;
    private static final byte HT16K33_BLINK_DISPLAYON = 0x01;
    private static final byte HT16K33_BLINK_OFF = 0;
    private static final byte HT16K33_BLINK_2HZ  = 1;
    private static final byte HT16K33_BLINK_1HZ = 2;
    private static final byte HT16K33_BLINK_HALFHZ = 3;
    private static final byte HT16K33_CMD_BRIGHTNESS = (byte) 0xE0;
    private final static byte numbertable[] = {
            0x3F, /* 0 */
            0x06, /* 1 */
            0x5B, /* 2 */
            0x4F, /* 3 */
            0x66, /* 4 */
            0x6D, /* 5 */
            0x7D, /* 6 */
            0x07, /* 7 */
            0x7F, /* 8 */
            0x6F, /* 9 */
            0x77, /* a */
            0x7C, /* b */
            0x39, /* C */
            0x5E, /* d */
            0x79, /* E */
            0x71, /* F */
    };
    public enum blinkRate{
        noblink,
        slow,
        medium,
        fast
    }
    private boolean v_timer_complete = false;
    private boolean v_ledseg_enabled = false;
    public AdafruitLEDBackpack7Seg (HardwareMap hardwareMap, String deviceName, byte RealI2CAddress) throws Exception
    {
        try{
            I2CAddress = RealI2CAddress;
            v_deviceName = deviceName;
            v_ledseg = new Wire(hardwareMap,deviceName,I2CAddress * 2);
            init();
        }catch (Exception p_exeception)
        {
            debugLogException("Error Creating Wire", p_exeception);
            v_ledseg = null;
            throw p_exeception;
        }
    }
    public AdafruitLEDBackpack7Seg (HardwareMap hardwareMap, String deviceName) throws Exception

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.
        try{
            v_deviceName = deviceName;
            v_ledseg = new Wire(hardwareMap,deviceName,I2CAddress * 2);
            init();

        }catch (Exception p_exeception)
        {
            debugLogException("Error Creating Wire", p_exeception);
            v_ledseg = null;
        }

    }

    private void init() throws Exception{
        try{
            if (v_ledseg != null) {
                v_ledseg.write(0x21, 0);
                v_ledseg.write(0x81, 0);
                v_ledseg.beginWrite(0x00);
                v_ledseg.write(numbertable[0]);
                v_ledseg.write(0x00);
                v_ledseg.write(numbertable[0]);
                v_ledseg.write(0x00);
                v_ledseg.write(0x2);
                v_ledseg.write(0x00);
                v_ledseg.write(numbertable[0]);
                v_ledseg.write(0x00);
                v_ledseg.write(numbertable[0]);
                v_ledseg.write(0x00);
                v_ledseg.endWrite();
                v_ledseg_enabled = true;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            throw p_exeception;
        }
    }

    public boolean is_timer_complete(){
        return v_timer_complete;
    }

    /**
     * put this in your loop so its called over and over in the loop
     */
    public void loop(){
        if (v_timer_mode == true && v_ledseg != null){
            long enddateSecs = v_timer_enddate.getTimeInMillis()/1000;
            long nowSecs =  Calendar.getInstance().getTimeInMillis() / 1000;
            int currentSeconds = (int) (enddateSecs - nowSecs);

            if (v_timer_seconds != currentSeconds){
                //debugLogException("loop(): cs:" + currentSeconds + ",ns:" + nowSecs + ",eds:" + enddateSecs,null);
                v_timer_seconds = currentSeconds;
                writeDigits(secondsToMinutesSeconds(v_timer_seconds),true);
                if (v_timer_seconds == 0 ){
                    set_blink_rate(blinkRate.noblink);
                    v_timer_mode = false;
                    v_timer_complete = true;
                }else if(v_timer_seconds <=10){
                    set_blink_rate(blinkRate.fast);
                }
            }
        }

    }

    //private boolean v_timer_mode = false;

    //private int v_seconds = 0;
    public boolean set_blink_rate(blinkRate rate){
        if(v_ledseg!=null) {

            byte byte_blinkRate;
            switch (rate) {
                case medium:
                    byte_blinkRate = HT16K33_BLINK_1HZ;
                    break;
                case slow:
                    byte_blinkRate = HT16K33_BLINK_HALFHZ;
                    break;
                case fast:
                    byte_blinkRate = HT16K33_BLINK_2HZ;
                    break;
                default:
                    byte_blinkRate = HT16K33_BLINK_OFF;
                    break;
            }

            v_ledseg.beginWrite(0x81 | (byte_blinkRate << 1));
            v_ledseg.write(0x00);
            v_ledseg.endWrite();
            return true;
        }else{
            return false;
        }
    }

    public boolean set_brightness(byte level) {
        if (level > 15) level = 15;
        if (v_ledseg != null) {
            v_ledseg.beginWrite(HT16K33_CMD_BRIGHTNESS | level);
            v_ledseg.write(0x00);
            v_ledseg.endWrite();
            return true;
        }else{
            return false;
        }


    }

    private String secondsToMinutesSeconds(int seconds){
        int mins = (int) Math.floor((seconds / 60));
        int secs = seconds % 60;
        String retval = Integer.toString(secs);
        if (retval.length() < 2){
            retval = "0" + retval;
        }
        return Integer.toString(mins) + retval ;
    }

    private boolean v_timer_mode = false;
    //private Calendar v_timer_startdate;
    private Calendar v_timer_enddate;
    private int v_timer_seconds = 0;
    public boolean startTimer(int seconds){
        if (v_timer_mode == false) {
            v_timer_complete = false;
            v_timer_seconds = seconds;
            //v_timer_startdate = Calendar.getInstance();
            v_timer_enddate = Calendar.getInstance();
            v_timer_enddate.add(Calendar.SECOND, seconds);
            debugPrint("startTimer():" + v_timer_enddate.toString());
            writeDigits(secondsToMinutesSeconds(seconds), true);
            v_timer_mode = true;
            return true;
        }else{
            debugLogException("timer already running call stopTimer() or wait for it to finish", null);
            return false;
        }
    }

    private byte getDigitValue(String digit){
       if (digit.equals("1")){
           return numbertable[1];
       }else if (digit.equals("2")){
           return numbertable[2];
       }else if (digit.equals("3")){
           return numbertable[3];
       }else if (digit.equals("4")){
           return numbertable[4];
       }else if (digit.equals("5")){
           return numbertable[5];
       }else if (digit.equals("6")){
           return numbertable[6];
       }else if (digit.equals("7")){
           return numbertable[7];
       }else if (digit.equals("8")){
           return numbertable[8];
       }else if (digit.equals("9")){
           return numbertable[9];
       }else if (digit.toLowerCase().equals("a")){
           return numbertable[10];
       }else if (digit.toLowerCase().equals("b")){
           return numbertable[11];
       }else if (digit.toLowerCase().equals("c")){
           return numbertable[12];
       }else if (digit.toLowerCase().equals("d")){
           return numbertable[13];
       }else if (digit.toLowerCase().equals("e")){
           return numbertable[14];
       }else{
           return numbertable[0];
       }
    }

    public boolean writeSeconds(int seconds){
        return writeDigits(secondsToMinutesSeconds(seconds), true);
    }

    public boolean writeDigits(String number, boolean displayColon){
       try{
            if(v_ledseg != null) {

                String digits = number;
                if (digits.length() < 4 ) {
                    digits = "0000".substring(digits.length()) + digits;
                }
                v_ledseg.beginWrite(0x00);
                for (int i = 0; i < 4; i++){
                    if (i == 2) {
                        if (displayColon) {
                            v_ledseg.write(0x2);
                        }else {
                            v_ledseg.write(0x00);
                        }
                        v_ledseg.write(0x00);
                    }
                    String digit = String.valueOf(digits.charAt(i));
                    byte bvalue = getDigitValue(digit);
                    //debugLogException("7seg write d:" + digit + ", v:" + Integer.toHexString(bvalue),null);
                    //v_ledseg.write(bvalue & 0xFF);
                    //v_ledseg.write(bvalue >> 8);
                    v_ledseg.write(bvalue);
                    v_ledseg.write(0x00);

                }

                v_ledseg.endWrite();
                return true;
            }else{
                return false;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            return false;
        }

    }
    int v_test_start = 0;
    public boolean writetest(){
        try{
            if(v_ledseg != null) {
                v_ledseg.beginWrite(0x00);
                v_ledseg.write(numbertable[v_test_start]);
                v_ledseg.write(0x00);
                v_ledseg.write(numbertable[v_test_start + 1]);
                v_ledseg.write(0x00);
                v_ledseg.write(0x2);
                v_ledseg.write(0x00);
                v_ledseg.write(numbertable[v_test_start + 2]);
                v_ledseg.write(0x00);
                v_ledseg.write(numbertable[v_test_start + 3]);
                v_ledseg.write(0x00);
                v_ledseg.endWrite();
                v_test_start++;
                if (v_test_start > 11){
                    v_test_start = 0;
                }
                return true;
            }else{
                return false;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            return false;
        }

    }

    public boolean isEnabled(){
        return v_ledseg_enabled;
    }
    public boolean enabled(boolean enable){
        try{
            if (v_ledseg_enabled != enable) {
                v_ledseg_enabled = enable;
                if (enable == true) {
                    //writeDigits("0000",true);
                    v_ledseg.write(0x81, 0);
                } else {
                    //writeDigits("    ", false);
                    v_ledseg.write(0x80, 0);
                }
            }
            return true;
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            return false;
        }
    }

    public void stop() {
        if(v_ledseg != null) {
            v_ledseg.close();
        }
    }
    void debugLogException( String msg, Exception ex){

        String debugMessage = logId + msg;
        if (ex != null) {
            String errMsg = ex.getMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            }else{
                debugMessage = debugMessage + " error. is null";
            }
            String stackTrace = Log.getStackTraceString(ex);
            if (stackTrace != null) {
                debugMessage = debugMessage + "\n" + stackTrace;
            }
        }else{
            debugMessage = debugMessage + " error is null";
        }


        android.util.Log.e("AdafruitLEDBackpack7Seg",debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }
    void debugPrint(String msg){
        String debugMessage = msg;
        android.util.Log.d("CFPushBotHardware", debugMessage);
    }

    void warningPrint(String msg){
        android.util.Log.w("CFPushBotHardware", msg);
    }
}
