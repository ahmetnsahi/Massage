 /* Msg.c
 *
 *  Created on: Sep 14, 2023
 *      Author: ahmet
 */

#include "Programs.h"
#include "main.h"
#include "stdio.h"


struct Motor M[16];

Control msP;
extern uint8_t RxBuf[];
uint8_t stopf=0;
uint16_t motor_sayısı = 0;
uint16_t pr=0;  //progmraları seçer
uint16_t pr1=0; //programları atar
uint16_t pr2=0;
uint16_t pr3=0;
extern start;
extern int a1;
extern int a2;
extern int a3;
uint32_t prg_time=0;
uint32_t prg_milis=0;
                                                                                   /* Tüm Programlar */
  // NOT : time, temp(on_of) ,duty(duty*5x)  , sonra motor sırası;

//Agrı Terapi
struct prg GA={2,1,140,7}; //1
struct prg BS={2,1,150,7};
struct prg BK={2,1,160,7};
struct prg AT={2,1,170,1600};
struct prg BT={2,1,180,1600};
struct prg DK={3,1,190,110};
struct prg BF={5,1,200,104};



//Tıbbı Seans

struct prg DG={3,1,160,495};//8
struct prg KD={2,1,175,1663};
struct prg LF={3,1,170,1659};
struct prg AÜ={3,1,180,1659};
struct prg MH={3,1,190,1659};
struct prg KT={1,1,200,2047};  //ikinci tur hızlı demiş

//Zayıflatma

struct prg TV={3,1,160,511};//14
struct prg KG={3,1,160,120};
struct prg BA={3,1,160,1616};
struct prg KY={3,1,160,120};
struct prg ÖA={4,1,160,510};
struct prg GS={4,1,160,120};
struct prg KB={4,1,160,448};//20




extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;


void pinit(void){

	  M[0].Channel=TIM_CHANNEL_1;  M[0].Duty=0;   M[0].Tımer=htim1;
	  M[1].Channel=TIM_CHANNEL_2;  M[1].Duty=0;   M[1].Tımer=htim1;
	  M[2].Channel=TIM_CHANNEL_3;  M[2].Duty=0;   M[2].Tımer=htim1;
	  M[3].Channel=TIM_CHANNEL_4;  M[3].Duty=0;   M[3].Tımer=htim1;//


	  M[4].Channel=TIM_CHANNEL_1;  M[4].Duty=0;   M[4].Tımer=htim3;
	  M[5].Channel=TIM_CHANNEL_2;  M[5].Duty=0;   M[5].Tımer=htim3;//
	  M[6].Channel=TIM_CHANNEL_3;  M[6].Duty=0;   M[6].Tımer=htim4;
	  M[7].Channel=TIM_CHANNEL_4;  M[7].Duty=0;   M[7].Tımer=htim4;


	  M[8].Channel=TIM_CHANNEL_1;  M[8].Duty=0;   M[8].Tımer=htim17;
	  M[9].Channel=TIM_CHANNEL_4;  M[9].Duty=0;   M[9].Tımer=htim2;
	  M[10].Channel=TIM_CHANNEL_3; M[10].Duty=0;  M[10].Tımer=htim2;
	  M[11].Channel=TIM_CHANNEL_4; M[11].Duty=0;  M[11].Tımer=htim3;

	  M[12].Channel=TIM_CHANNEL_3; M[12].Duty=0;  M[12].Tımer=htim3;
	  M[13].Channel=TIM_CHANNEL_1; M[13].Duty=0;  M[13].Tımer=htim2;
	  M[14].Channel=TIM_CHANNEL_2; M[14].Duty=0;  M[14].Tımer=htim2;
	  M[15].Channel=TIM_CHANNEL_1; M[15].Duty=0;  M[15].Tımer=htim16;

}

void Pwm_Start(Control Ms){

	for ( uint16_t var = 0;  var < 16; var++) {



		if (Ms.MotorSayi & (1 << var)) {  // n. bit 1

			if(var <= 12){
				M[var].Duty=Ms.MotorDuty;
			}else{
				M[var].Duty=Ms.TempDuty;
			}

		   __HAL_TIM_SET_COMPARE(&M[var].Tımer,M[var].Channel,M[var].Duty);

	    }
		else{                 // n. bit 0

		   M[var].Duty=0;
		  __HAL_TIM_SET_COMPARE(&M[var].Tımer,M[var].Channel,M[var].Duty);
	    }


       }


}



prg selectProgram(uint8_t pr) {
    switch (pr) {
        case 1: return GA;
        case 2: return BS;
        case 3: return BK;
        case 4: return AT;
        case 5: return BT;
        case 6: return DK;
        case 7: return BF;
        case 8: return DG;
        case 9: return KD;
        case 10: return LF;
        case 11: return AÜ;
        case 12: return MH;
        case 13: return KT;
        case 14: return TV;
        case 15: return KG;
        case 16: return BA;
        case 17: return KY;
        case 18: return ÖA;
        case 19: return GS;
        case 20: return KB;
        default: return (prg){0};; // Varsayılan bir değer döndürülebilir
    }
}






// Ana program seçim fonksiyonu
prgSelection prgSel() {
    prgSelection selection = {0}; // Tüm alanları sıfırla
    selection.selectedProgramsCount = 0;

    if (RxBuf[2] > 64 && RxBuf[2] <= 84) { // Geçerli program aralığı kontrolü
        selection.program1 = selectProgram(RxBuf[2] - 64);
        selection.selectedProgramsCount++;
    }
    if (RxBuf[3] > 64 && RxBuf[3] <= 84) { // Geçerli program aralığı kontrolü
        selection.program2 = selectProgram(RxBuf[3] - 64);
        selection.selectedProgramsCount++;
    }
    if (RxBuf[4] > 64 && RxBuf[4] <= 84) { // Geçerli program aralığı kontrolü
        selection.program3 = selectProgram(RxBuf[4] - 64);
        selection.selectedProgramsCount++;
    }

    return selection;
}


uint8_t  Stop_Full(void){
	 uint8_t bool;



	 if (RxBuf[0]=='S' && RxBuf[1]=='T' && RxBuf[2]=='P' ) {    //kodu kesiyor.stop char d

		 start=0;
		 Control mX;
		 mX.MotorSayi=0b1111111111111111; //motor sayıları için pwm
		 mX.MotorDuty=0;                //duty sıfır olarak set edildi.
		 mX.TempDuty=0;
		 Pwm_Start(mX);         //pwm out veriyor.
		 return bool=1;

	    }else return bool=0;


}



//Standar programlar için gereken kod.
void program(uint16_t time){

uint16_t tempsel=0;
prgSelection selectedPrograms = prgSel();
uint8_t prgsay=selectedPrograms.selectedProgramsCount;
// Isıtıcıların ayarı
// Gelen rxbuf[5]'in ilk 3 bitine göre karar verilecek
          if (RxBuf[5] & 0x01) {
            tempsel |= (1 << 13); // 1. bit set edilirse 13. biti 1 yap
            }
          if (RxBuf[5] & 0x02) {
	         tempsel |= (1 << 14); // 2. bit set edilirse 14. biti 1 yap
            }
          if (RxBuf[5] & 0x04) {
	         tempsel |= (1 << 15); // 3. bit set edilirse 15. biti 1 yap
            }
          if (RxBuf[5] & 0x08) {
            tempsel |= (1 << 8); // 4. bit set edilirse 8. biti 1 yap  ayak
            }
          if (RxBuf[5] & 0x10) {
             tempsel |= (1 << 9); // 5. bit set edilirse 9. biti 1 yap kemer
            }

   selectedPrograms.program1.motor |= tempsel;
   selectedPrograms.program2.motor |= tempsel;
   selectedPrograms.program3.motor |= tempsel;

   prg prgarry[]={selectedPrograms.program1,selectedPrograms.program2,selectedPrograms.program3};


   prg_time = time / prgsay; // Her bir program için süre
   printf("Birim program zamani =%d\n\r",prg_time);
   uint8_t timerhesap=0;
                   for (int var = 0; var < 3; var++) {

                       if (prgarry[var].speed == 0) {
                    	    timerhesap++;
                            continue;


                       }

                	   printf("program id =%d\n\r",var+1);
                	   while(1){
                		   standartprogram(prgarry[var],var+1-timerhesap); // Programı çalıştır

      	    	    	 if(total_time-timer >= (prg_time*(var+1-timerhesap)-2)){
      	    	    		 break;
      	    	    	 }
      	    	          if (Stop_Full()==1) { //prg çalışırken durdurma
      	    	    	    	   	            break;
      	    	    	  }

                	   }

  	    	           if (Stop_Full()==1) { //prg çalışırken durdurma
  	    	   	            break;
  	    	   	          }



			        }
      start=0;




}


void standartprogram (prg program,uint8_t pid){  //LOOP Mantıgı ile çalışan progrmalar. //0dan 10 a 10 dan 0 a gelen kod


uint8_t var=0;
uint16_t t =0;//Motor bitleri
msP.MotorSayi=0;  //motor sayaçları sıfırlandı
msP.MotorDuty=motorduty;
msP.TempDuty=tempduty;

msP.MotorSayi = program.motor & 0xE000;  //ısıtıcılar aktif edildi
Pwm_Start(msP);          // PWM  out veriyor.
    //Motorları dönen kod

	    for (var = 0;  var < 10; ++var) {

	          if (Stop_Full()==1) { //prg çalışırken durdurma
	            stop1:
	        	  break;
	          }else

	          {

	    	    if (program.motor & ( 1 <<var)) //hangi motorların açık olacagına bakıyor.
	    		{  /* n. bit 1 */

                 t =program.motor & 0xE000; //eger ısıtıcılar aktifse
	    	     t |= 1<<var ;            //açık olan motor bitini çekiyor.
	    	     msP.MotorSayi=t;            //motoru yüklüyor.
	    	     msP.MotorDuty=motorduty;
	    	     msP.TempDuty=tempduty;
	    	     Pwm_Start(msP);         //pwm out veriyor.
                 t=0;

	    	     //--------------Program geçişleri için delay ve delayda durdurma kodu------------------//
	    	     uint32_t tickstart = HAL_GetTick();
	    	     uint32_t wait = program.time*1000;

	    	     /* Add a freq to guarantee minimum wait */
	    	     if (wait < HAL_MAX_DELAY)
	    	     {
	    	       wait += (uint32_t)(uwTickFreq);
	    	     }

	    	     while ((HAL_GetTick() - tickstart) < wait)
	    	     {
	    	    	 if (Stop_Full()==1 | start==0) { //prg çalışırken durdurma
	    	    		 goto stop2;

	 	         }
	    	    	 if(total_time-timer >= (prg_time*pid)-1){
	    	    		 goto stop2;
	    	    	 }

	    	     }


	    	    // HAL_Delay(prgSel().time*1000); //progrma süresini çekiyor.
	    	    //----------------Delay------------------------------//


	    		}
	    		else {
	    		   /* n. bit 0 */
	                    //genel olarak işlem yok.
	    		     }

	           }


		 }

	       // ikinci tur

	        for (var=var-1 ;  var >0; --var) {

	          if (Stop_Full()==1) { //prg çalışırken durdurma
	            stop2:
	        	  break;

	          }
	          else
	          {

	    	    if (program.motor & ( 1 <<var)) //hangi motorların açık olacagına bakıyor.
	    		{  /* n. bit 1 */


	              t |=program.motor & 0xE000; //eger ısıtıcılar aktifse
		    	  t |= 1<<var ;            //açık olan motor bitini çekiyor.
		    	  msP.MotorSayi=t;            //motoru yüklüyor.
		    	  msP.MotorDuty=motorduty;
		    	  msP.TempDuty=tempduty;
		    	  Pwm_Start(msP);         //pwm out veriyor.
	              t=0;
	    	     //--------------Program geçişleri için delay ve delayda durdurma kodu------------------//
	    	     uint32_t tickstart = HAL_GetTick();
	    	     uint32_t wait = program.time*1000;

	    	     /* Add a freq to guarantee minimum wait */
	    	     if (wait < HAL_MAX_DELAY)
	    	     {
	    	       wait += (uint32_t)(uwTickFreq);
	    	     }

	    	      while ((HAL_GetTick() - tickstart) < wait)
	    	      {
	    	    	   if (Stop_Full()==1 | start==0) { //prg çalışırken durdurma
	    	    		 goto stop2;

	    	    		 if(total_time-timer >= prg_time*pid-1){
	    	    			 goto stop2;
	    	    		 }

	 	           }

	    	      }


	    	    // HAL_Delay(prgSel().time*1000); //progrma süresini çekiyor.
	    	    //----------------Delay------------------------------//


	    		}
	    		else {
	    		   /* n. bit 0 */
	                    //genel olarak işlem yok.
	    		     }

	           }


		 }

	   //DÖNEN SON KOD












}














