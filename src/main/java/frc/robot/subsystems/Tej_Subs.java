package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tejuino.TejuinoBoard;

public class Tej_Subs extends SubsystemBase {
  /** Creates a new Tesexjuino. */
  TejuinoBoard tesexjuino;
  static Tej_Subs instance;
  int timer = 0;
  int mode = 0;
  int whiteBlink = 20; //hice que los tics sean 20 pq deberia ser un segundo cada color. Así no se petan las luces pero no es spr lento alvvvvvv

  //INFORMACION GENERAL: P1 es Blanco/verde y en UBicacion es verde
  //INFORMACION GENERAL: P2 es Blanco/azul y en ubicacion es azul
  //INFORMACION GENERAL: P3 es blanco y yellow en ubicacion es amarillo 
  //INFORMACION GENERAL: el comando de endmatch es full rojo y hasta abajo
  
  public Tej_Subs() {

    tesexjuino = new TejuinoBoard();
  }
 public static Tej_Subs getInstance(){
  if (instance == null) {
    instance = new Tej_Subs();
  }
  return instance;
 }
  
 public void setMode(int m){
  mode = m;
 }
 public void periodic(){

    if( mode == 0){
      tesexjuino.turn_off_all_leds(0);
      tesexjuino.turn_off_all_leds(1);
      tesexjuino.turn_off_all_leds(2);
    }

    if( mode == 1){
      if( timer < whiteBlink ){
        wl();
      }else{
          // call all led green
          gl();

      }
      if(timer == 2*whiteBlink){
        timer = 0;
      }
      timer += 1;

    }//Mode 1 is green bruh
    //Esta es para P1
 
 
if( mode == 2){
  if( timer < whiteBlink ){
    wl();
  }else{
      // call all led green
      bl();

  }
  if(timer == 2*whiteBlink){
    timer = 0;
  }
  timer += 1;
 
}// mode 2 is blue
//esta es para el P2


if( mode == 3){
  if( timer < whiteBlink ){
    wl();
  }else{
      // call all led green
      yl();

  }
  if(timer == 2*whiteBlink){
    timer = 0;
  }
  timer += 1;
 
}//mode 3 is pink (subject to change)
//Esta es para el P3
}
  
  
  

  public void gl(){
  
  tesexjuino.all_leds_green(0);
  tesexjuino.all_leds_green(1);
  tesexjuino.all_leds_green(2);

  }

  public void wl(){
  
    tesexjuino.all_leds_white(0);
    tesexjuino.all_leds_white(1);
    tesexjuino.all_leds_white(2);
  
  }

  public void bl(){

    tesexjuino.all_leds_blue(0);
    tesexjuino.all_leds_blue(1);
    tesexjuino.all_leds_blue(2);

  }

  

  public void yl(){
    
  tesexjuino.all_leds_yellow(0);
  tesexjuino.all_leds_yellow(1);
  tesexjuino.all_leds_yellow(2);
  
  
  }

  public void endmatch(){

    
    tesexjuino.all_leds_red(0);
    tesexjuino.all_leds_red(1);
    tesexjuino.all_leds_red(2);

  }

}
