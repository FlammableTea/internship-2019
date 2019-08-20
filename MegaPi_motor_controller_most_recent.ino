/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    Me_Megapi_encoder_pid_pos.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/07/14
 * @brief   Description: this file is sample code for Megapi encoder motor device.
 *
 * Function List:
 *    1. uint8_t MeEncoderOnBoard::getPortB(void);
 *    2. uint8_t MeEncoderOnBoard::getIntNum(void);
 *    3. void MeEncoderOnBoard::pulsePosPlus(void);
 *    4. void MeEncoderOnBoard::pulsePosMinus(void);
 *    5. void MeEncoderOnBoard::setMotorPwm(int pwm);
 *    6. double MeEncoderOnBoard::getCurrentSpeed(void);
 *    7. void MeEncoderOnBoard::setSpeedPid(float p,float i,float d);
 *    8. void MeEncoderOnBoard::setPosPid(float p,float i,float d);
 *    7. void MeEncoderOnBoard::setPosPid(float p,float i,float d);
 *    8. void MeEncoderOnBoard::setPulse(int16_t pulseValue);
 *    9. void MeEncoderOnBoard::setRatio(int16_t RatioValue);
 *    10. void MeEncoderOnBoard::moveTo(long position,float speed,int16_t extId,cb callback);
 *    11. void MeEncoderOnBoard::loop(void);
 *    12. long MeEncoderOnBoard::getCurPos(void);
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2016/07/14    1.0.0          build the new
 * </pre>
 */

#include <MeMegaPi.h>

myMeEncoderOnBoard Encoder_1(SLOT1);
int count =0;
int prev_state=0;
int new_state=0;
int skipped=0;
int target_pos =0;
int cur_pos = 0;
int last_pos =0;


void isr_process_encoder1(void)
{
//  new_state = digitalRead(Encoder_1.getPortB());
//  if (new_state==prev_state) {skipped=1;}
//  else {skipped=0;}
//  prev_state = new_state; // save old state
   
  if (digitalRead(Encoder_1.getPortA()) ) { // A enc = rise 
    if(digitalRead(Encoder_1.getPortB()) == 0)    {
      Encoder_1.pulsePosMinus();
    }
    else {
      Encoder_1.pulsePosPlus();
    }
  }
  //Serial.println(Encoder_1.getPulsePos()); // dchang - pulsepos = encoder value, currentPos = pulse*ratio// need to define ratio
}

void setup()
{  
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, CHANGE);// implementing for rise & fall = increase resolution by 2
  //attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  //Serial.println(Encoder_1.getIntNum());
  //Serial.println(Encoder_1.getPortA());
  //Serial.println(Encoder_1.getPortB());
  //Serial.println(digitalPinToInterrupt(Encoder_1.getPortA()));
  //Serial.println(digitalPinToInterrupt(Encoder_1.getPortB()));
  
  //Set PWM 8KHz
 TCCR1A = _BV(WGM10);
 TCCR1B = _BV(CS11) | _BV(WGM12);

  //TCCR2A = _BV(WGM21) | _BV(WGM20);
  //TCCR2B = _BV(CS21);

 Encoder_1.setPulse(7);
 // Encoder_2.setPulse(7);
 Encoder_1.setRatio(26.9);
 // Encoder_2.setRatio(26.9);
 Encoder_1.setPosPid(1.8,0,1.2);
 // Encoder_2.setPosPid(1.8,0,1.2);
 Encoder_1.setSpeedPid(0.18,0,0);
 // Encoder_2.setSpeedPid(0.18,0,0);
}

void loop()
{
  if(Serial.available())
  {
    String input = Serial.readString();
    String msgType = input.substring(0, 3);
    if(msgType.equals("pos")){
      String msgNum = input.substring(4);
      Encoder_1.moveTo(msgNum.toInt(), 150); 
    }
    if(msgType.equals("spd")){
      String msgNum = input.substring(4);
      Encoder_1.setSpeed(msgNum.toFloat());
    }
  }
    //char a = Serial.read();
//    switch(a)
//    {
//      case '0':
//      target_pos = 0;
//      break;
//      case '1':
//      target_pos = 500;
//      break;
//      case '2':      
//      target_pos = 1000;
//      break;
//      case '3':
//      target_pos = 2000;
//      break;
//      case '4':
//      target_pos = -500;
//      break;
//      case '5':
//      target_pos = -1000;
//      break;
//      case '6':
//       target_pos = -2000;
//       break;
//      default:
//      break;
//    }
//     Encoder_1.moveTo(target_pos,100);
//  }
  Encoder_1.loop();
  
  //cur_pos = Encoder_1.getCurPos();
  
  //if (cur_pos < target_pos && cur_pos < last_pos) { Serial.println (Encoder_1.getCurPos());}
  //else if (cur_pos > target_pos && cur_pos > last_pos) { Serial.println (Encoder_1.getCurPos());}
  //last_pos = Encoder_1.getCurPos(); // update last position
  


  
  //Encoder_2.loop();
  Serial.print("Spped 1:");
  Serial.print(Encoder_1.getCurrentSpeed());
  //Serial.print(" ,Spped 2:");
  Serial.print("CurPos 1: ");
  Serial.println(Encoder_1.getCurPos());
  //Serial.print(" ,CurPos 2:");
  //Serial.println(Encoder_2.getCurPos());
}
