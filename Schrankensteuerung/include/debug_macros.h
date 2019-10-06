#ifndef DEBUG_MACROS_H
#define DEBUG_MACROS_H

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINTLN(x) Serial.println (x)
 #define DEBUG_PRINTF3(x, y, z) Serial.printf (x, y, z)
 #define DEBUG_PRINTF2(x, y) Serial.printf (x, y)
 #define DEBUG_PRINT(x) Serial.print (x)
 #define DEBUG_PRINTDIAG(x) WiFi.printDiag (x)
#else
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINTF3(x, y, z)
 #define DEBUG_PRINTF2(x, y) 
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDIAG(x)
#endif

#endif