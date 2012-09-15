////////////////////////////////////////////////////////////////////////////
//                  _ _                       _                           //
//                 (_|_)                     | |                          //
//    __ _ ___  ___ _ _             __ _ _ __| |_        ___ _ __  _ __   //
//   / _` / __|/ __| | |           / _` | '__| __|      / __| '_ \| '_ \  //
//  | (_| \__ \ (__| | |          | (_| | |  | |_   _  | (__| |_) | |_) | //
//   \__,_|___/\___|_|_|           \__,_|_|   \__| (_)  \___| .__/| .__/  //
//                        ______                            | |   | |     //
//                       |______|                           |_|   |_|     //
////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include "ascii_art.h"





/////////////////////////////////////////////////////////
//   _   _              _____            _             //
//  | | (_)            / ____|          | |            //
//  | |_ _ _ __  _   _| |     ___  _ __ | |_ ___ _ __  //
//  | __| | '_ \| | | | |    / _ \| '_ \| __/ _ \ '__| //
//  | |_| | | | | |_| | |___| (_) | |_) | ||  __/ |    //
//   \__|_|_| |_|\__, |\_____\___/| .__/ \__\___|_|    //
//                __/ |           | |                  //
// Nathan Duprey |___/            |_|        Version 1 //
/////////////////////////////////////////////////////////
void print_tinyCopter_ASCII(){
  //Cool ASCII art banner
  Serial.println("  _   _              _____            _            ");
  Serial.println(" | | (_)            / ____|          | |           ");
  Serial.println(" | |_ _ _ __  _   _| |     ___  _ __ | |_ ___ _ __ ");
  Serial.print(" | __| | '_ ");
  Serial.write(0x5C);
  Serial.print("| | | | |    / _ ");
  Serial.write(0x5C);
  Serial.print("| '_ ");
  Serial.write(0x5C);
  Serial.print("| __/ _ ");
  Serial.write(0x5C);
  Serial.println(" '__|");
  Serial.println(" | |_| | | | | |_| | |___| (_) | |_) | ||  __/ |   ");
  Serial.print("  ");
  Serial.write(0x5C);
  Serial.print("__|_|_| |_|");
  Serial.write(0x5C);
  Serial.print("__, |");
  Serial.write(0x5C);
  Serial.print("_____");
  Serial.write(0x5C);
  Serial.print("___/| .__/ ");
  Serial.write(0x5C);
  Serial.print("__");
  Serial.write(0x5C);
  Serial.println("___|_|   ");
  Serial.println("               __/ |           | |                 ");
  Serial.println("Nathan Duprey |___/            |_|        Version 1");
  Serial.println("");
}
