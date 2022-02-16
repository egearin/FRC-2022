// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.FireAnimation;

import frc.team6429.robot.Constants;
import frc.team6429.robot.RobotData.AnimationTypes;

/** Add your docs here. */
public class LED {

    private static LED mInstance = new LED();

    public static LED getInstance(){
        return mInstance;
    }

    //CANdle
    public CANdle mCANdle;
    public CANdleConfiguration config;

    //Animation
    public RainbowAnimation rainbow;
    public ColorFlowAnimation colorFlow;
    public LarsonAnimation larson;
    public RgbFadeAnimation rgbFade;
    public SingleFadeAnimation singleFade;
    public StrobeAnimation strobe;
    public TwinkleAnimation twinkle;
    public TwinkleOffAnimation twinkleOff;
    public FireAnimation fire;
    public Direction direction;
    public Animation animation;
    public AnimationTypes animationTypes;



    //LED Configuration
    public void configLED(){
        mCANdle = new CANdle(Constants.candleID, "CANdle");
        mCANdle.configAllSettings(config);
        mCANdle.getAllConfigs(config);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; //set the strip type to RGB
    }

    /**
     * select and change animation type
     * @param change
     */
    public void changeAnimation(AnimationTypes change) {
        switch (change) {
          case ColorFlow:
            //setColorFlow(r, g, b, w, speed, numLed, direction);
            break;
          case Fire:
            //setFireAnim(r, g, b, brightness, speed, numLed, sparking, cooling);
            break;
          case Larson:
            //setLarson(r, g, b, w, speed, numLed, mode, size);
            break;
          case Rainbow:
            //setRainbow(speed, brightness, numLed);
            break;
          case RgbFade:
            //setRgbFade(speed, brightness, numLed);
            break;
          case SingleFade:
            //setSingleFade(r, g, b, w, speed, numLed);
            break;
          case Strobe:
            //setStrobe(r, g, b, w, speed, numLed);
            break;
          case Twinkle:
            //setTwinkle(r, g, b, w, speed, numLed, divider);
            break;
          case TwinkleOff:
            //setTwinkleOff(r, g, b, w, speed, numLed, divider);
            break;
          case SetAll:
            animation = null;
            break;
        }
    }

    /**
     * Set Rainbow Animation
     * @param brightness The brightness of the LEDs [0, 1]
     * @param speed How fast the rainbow travels through the leds [0, 1]
     * @param numLed How many LEDs are controlled by the CANdle
     */
    public void setRainbow(double speed, double brightness, int numLed){
        mCANdle.animate(new RainbowAnimation(brightness, speed, numLed));
    }

    /**
     * Set RGB Fade Animation
     * @param brightness How bright the LEDs are [0, 1]
     * @param speed How fast the LEDs fade between Red, Green, and Blue [0, 1]
     * @param numLed How many LEDs are controlled by the CANdle
     */
    public void setRgbFade(double speed, double brightness, int numLed){
        mCANdle.animate(new RgbFadeAnimation(speed,brightness,numLed));
    }

    /**
     * Set Colorflow Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs is the CANdle controlling
     * @param direction What direction should the color move in
     */
    public void setColorFlow(int r, int g, int b, int w, double speed, int numLed, Direction direction){
        mCANdle.animate(new ColorFlowAnimation(r, g, b, w, speed, numLed, direction));
    }

    /**
     * Set Larson Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed The number of LEDs the CANdle will control
     * @param mode How the pocket of LEDs will behave once it reaches the end of the strip
     * @param size How large the pocket of LEDs are [0, 7]
     */
    public void setLarson(int r, int g, int b, int w, double speed, int numLed, BounceMode mode, int size){
        mCANdle.animate(new LarsonAnimation(r, g, b, w, speed, numLed, mode, size));
    }

    /**
     * Set Single Fade Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     */
    public void setSingleFade(int r, int g, int b, int w, double speed, int numLed){
        mCANdle.animate(new SingleFadeAnimation(r, g, b, w, speed, numLed));
    }

    /**
     * Set Strobe Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     */
    public void setStrobe(int r ,int g, int b, int w, double speed, int numLed){
        mCANdle.animate(new StrobeAnimation(r, g, b, w, speed, numLed));
    }

    /**
     * Set Twinkle Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     * @param divider What percentage of LEDs can be on at any point
     */
    public void setTwinkle(int r, int g, int b, int w, double speed, int numLed, TwinklePercent divider){
        mCANdle.animate(new TwinkleAnimation(r, g, b, w, speed, numLed, divider));
    }
    
    /**
     * Set Twinkle Off Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     * @param divider What percentage of LEDs can be on at any point
     */
    public void setTwinkleOff(int r, int g, int b, int w, double speed, int numLed, TwinkleOffPercent divider){
        mCANdle.animate(new TwinkleOffAnimation(r, g, b, w, speed, numLed, divider));
    }

    /**
     * Set Fire Animation
     * @param brightness How bright should the animation be [0, 1]
     * @param speed How fast will the flame be processed at [0, 1]
     * @param numLed How many LEDs is the CANdle controlling
     * @param sparking The rate at which the Fire "Sparks" [0, 1]
     * @param cooling The rate at which the Fire "Cools" along the travel [0, 1]
     */
    public void setFireAnim(int r, int g, int b, double brightness, double speed, int numLed, double sparking, double cooling){
        mCANdle.animate(new FireAnimation(brightness, speed, numLed, sparking, cooling));
    }

    //Custom led color
    public void setCustom(){
    }
}
