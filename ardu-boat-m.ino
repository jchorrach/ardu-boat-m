/*********************************************************
**
**  ARDU-BOAT-M
**  sketch monitorización del barco con arduino
**
**  -Revisa agua en la sentina
**  -Marcha/Paro bomba achique
**  -Tension en bateria
**  -Comunicacion GSM
**  -Deteccion fuga de gas
**  -Detección intrusos
**
*********************************************************
Mapa de pines:
------------------------------------
00: Rx Serial
01: Tx Serial
02:< 
03:<Alarma intrusos [ROBO] in
04:
05:
06:
07: Rx tarjeta GSM
08: Tx tarjeta GSM
09: Reset tarjeta GSM [RESET_GSM] out
10:
11: Sensor agua [WATER] in
12: Relay bomba achique [PUMP] out
13: Sensor gas butano [SGAS] in
A0: Bateria Servicio [VBATS] in
A1: Bateria Motor [VBATM] in
A2:
A3:
A4: SDA para LCD
A5: SCL para LCD
*/

#include <stdlib.h>
#include <EEPROM.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#define NAME "BOAT-SAFE"     // Nombre del producto
#define VERSION "Ver. Beta"  // Version

#define END_CMD '\r'       // Fin linea de comando

#define INFO_ENABLED  false // Activa/desactiva debug información (0)
#define DEBUG_ENABLED false // Activa/desactiva debug (1)
#define ALERT_ENABLED false // Activa/desactiva debug alertas (2)

//--------------------------------------------------------------
// Display LCD
// 
// LCD1602 controlado via adaptador I2C
//
#define LCD_TIME_PAG 3500    // ms para cambio de pagina display LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
byte num_dis = 0;            // Pagina que se enseña en el display
long page_d_milis;           // Tiempo para que se cambie pagina el LCD


//--------------------------------------------------------------
// Comunicaciones GSM
//
// SIM900 shield
//
#define TLFSMS_HEADER "TS"   // Tag del comando tlf envio sms
#define TLFSMS_LEN 13        // Longitud del comando tlf envio sms ej.34123123123
#define MODEM_TIME_CHK 14000 // ms de funcionamiento en modo automatico
#define RESET_GSM 9          // Pin Reset de la placa GSM
String tlf_sms;              // Telefono al que se envian los SMS
String tlf_auth;             // Telefono autorizado a interrogar
SoftwareSerial GSM(7, 8);    // Usa sensores digitales 7RX y 8TX
String gsmbuf;               // buffer array para recibir datos
long chk_m_milis = 0;        // Tiempo en que deberia pararse automaticamente la bomba
byte netGSM = 0;             // Indica si hay conexion de red GSM (0 No, 1 Si)

//....................... SEND_SMS_ALARM .......................
byte SEND_SMS_ALARM = 0; // Indicador de alarma enviada por SMS (bitmap)
//
//  00000000
//  |||||||+--- bit(1): si 1 SMS Alarma agua sentina enviado
//  ||||||+-----bit(2): si 1 SMS Alarma batt. motor baja enviado
//  |||||+------bit(3): si 1 SMS Alarma batt. servicio baja enviado
//  ||||+-------bit(4): si 1 SMS Alarma gas detectado enviado
//  |||+--------bit(5): si 1 SMS Alarma intruso enviado
//
//..............................................................

//--------------------------------------------------------------
// Setup
// CMD: S<valor>
// CMD: SA0 -> Reset alarmas y contador achique
//
#define SETUP_HEADER "S" // Tag del comando setup
#define SETUP_MSG_LEN 3  // Longitud del comando setup


//--------------------------------------------------------------
// Nivel del agua en sentina
//
#define WATER 11         // Nivel del agua (Digital In)
#define WATER_INTERVAL_VAL 5000  // Intervalo ms para comparar niveles

bool waterState = 0; // Estado del sensor boya

//--------------------------------------------------------------
// Comado bomba de achique
// CMD: A<valor>
// CMD: AS -> En marcha bomba de achique 
// CMD: AA -> En marcha bomba de achique con stop automatico
// CMD: AN -> Para bomba de achique
//
#define PUMP 12          // Control rele bomba (Digital Out)
#define PUMP_HEADER "A"  // Tag del comando Bomba achique
#define PUMP_MSG_LEN 2   // Longitud del comando Bomba achique
#define PUMP_TIME_AUTO 12000 // ms de funcionamiento en modo automatico
#define PUMP_MAX_START 2     // Maximo numero de activaciones por periodo
#define PUMP_TIME_MAX_START 400000 // Periodo en el que se evalua el numero de marchas de la bomba

byte flag_a = 0;     // Si 0 sin activacion 1 inicio bomba si 2 esperando duracion para apagar
long stop_b_milis;   // Tiempo en que deberia pararse automaticamente la bomba
byte num_starts = 0; // Conteo de marchas automaticas bomba

//---------------------------------------------------------------
// Lectura de tension de las baterias
//
//
#define VBATS 0 // Pin analogico para leer tension bat. servicio
#define VBATM 1 // Pin analogico para leer tension bat. motor
#define MIN_VOL 12.1 // Valor de alarma para voltaje bajo

//---------------------------------------------------------------
// Lectura de sensor gas butano
//
//
#define SGAS 13       // Pin para leer sensor de gas

//---------------------------------------------------------------
// Trigger alarma de robo
//
//
#define ROBO 3       // Pin(interrupciones) alarma de robo

//--------------------------------------------------------------

//-------------------------- Alarmas ---------------------------
byte alarmas = 0; // Indicador de alramas activas (bitmap)
//
//  00000000
//  |||||||+--- bit(1): si 1 Alarma agua sentina
//  ||||||+-----bit(2): si 1 Alarma batt. motor baja
//  |||||+------bit(3): si 1 Alarma batt. servicio baja
//  ||||+-------bit(4): si 1 Alarma gas detectado
//  |||+--------bit(5): si 1 Alarma intruso
//
//--------------------------------------------------------------

String debug_msg;    // Mensaje de debug
String comm_msg;     // Mensaje de comunicaciones

char c_string[20]; // String que contendra el comando recibido
int sindex = 0;    // Indice del proximo caracter en el string

long fin_time_starts; // Tiempo de inicio de evaluacion marchas bomba

//**********************************************************************************************************************

void setup()
{
  Serial.begin(9600);   // Inicializa monitor Serie
  GSM.begin(19200);     // Inicializa Modem GSM
  lcd.init();           // Inicializa el lcd 
  lcd.backlight();
  lcd.home ();                // Primera linea y columna del display
  lcd.print(NAME);
  lcd.setCursor ( 0, 1 );     // Segunda linea del display
  lcd.print (VERSION);
  
  GSMPower();                 // Activa shiled GSM
  
  // escritura eeprom
  //EEPROM.write(0,'3');
  //EEPROM.write(1,'4');
  //EEPROM.write(2,'6');
  //EEPROM.write(3,'3');
  //EEPROM.write(4,'9');
  //EEPROM.write(5,'6');
  //EEPROM.write(6,'3');
  //EEPROM.write(7,'5');
  //EEPROM.write(8,'7');
  //EEPROM.write(9,'5');
  //EEPROM.write(10,'1');
  
  //EEPROM.write(11,'6');
  //EEPROM.write(12,'3');
  //EEPROM.write(13,'9');
  //EEPROM.write(14,'6');
  //EEPROM.write(15,'3');
  //EEPROM.write(16,'5');
  //EEPROM.write(17,'7');
  //EEPROM.write(18,'5');
  //EEPROM.write(19,'1');
  
  // Lee TLF SMS guardado en eeprom
  // formato prefijo pais y tlf
  // p.e. 34123123123
  for (int i = 0; i < 11; i++)
  {
    sindex = EEPROM.read(i);
    if (sindex > 47 || sindex < 58 )
      tlf_sms.concat(char(sindex));
    else
      tlf_sms.concat("*");
  }
  // Lee TLF AUTORIZADO guardado en eeprom
  // formato 9 digitos tlf
  // p.e. 123123123
  for (int i = 11; i < 20; i++)
  {
    sindex = EEPROM.read(i);
    if (sindex > 47 || sindex < 58 )
      tlf_auth.concat(char(sindex));
    else
      tlf_auth.concat("*");
  }
  
  
  
  lcd.clear();
  lcd.home();
  lcd.print(F("Mode Remote"));
  
  pinMode(RESET_GSM, OUTPUT);
  
  pinMode(SGAS, INPUT); 
  pinMode(PUMP, OUTPUT);
  pinMode(WATER, INPUT);
  pinMode(ROBO, INPUT);
  
  digitalWrite(ROBO, HIGH);  // Habilita internal pullup
  digitalWrite(WATER, HIGH); // Habilita internal pullup
  digitalWrite(PUMP, HIGH);  // Habilita internal pullup
  
 
  flag_a = 0;                // Bomba automatica desactivada

  fin_time_starts = PUMP_TIME_MAX_START;  // Tiempo limite para evaluar marchas automaticas
  
  page_d_milis = millis() + LCD_TIME_PAG; // Establece tiempo para cambio de pagina en LCD
}

void loop()
{

  if (Serial.available())
  {
    reciveCommand();
  }
  // Chequeo agua en la sentina
  checkWater();
  // Chequeo estado funcionamiento bomba automatico
  autoPump();
  // Chequeo gas
  checkGas();
  //Chequeo intruso
  checkRobo();
  // Gerstion modem GSM
  _GSM();

  // Display
  LCDStatus();
  
  delay(500);
}

//**********************************************************************************************************************
//
// Comunicaciones GSM >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//
// Funcionalidades GSM
//
void _GSM()
{
  ProcessGSM(); // Lee comandos modem GSM
  ChkGSM();     // Chequeo modem GSM
  SMSAlertas(); // Gestion alertas SMS
}

//
//  Gestion de envio de SMS de alertas
//
void SMSAlertas()
{
   if (netGSM == 0)
    return; // El GSM no esta conectado
  if (alarmas > 0)
  {
     // Alerta Demasiados Achiques
     if ((alarmas & 1) == 1 && (SEND_SMS_ALARM & 1) == 0)
     {
        begin_sms();
        GSM.println(F("Demasiados achiques!"));
        delay(100);
        ProcessGSM();
        comm_msg = " Achiques: ";
        comm_msg.concat(num_starts);
        GSM.print(comm_msg);
        delay(100);
	SEND_SMS_ALARM = SEND_SMS_ALARM | 1; // bit(1) a 1
        end_SMS();
     } 
     // Alerta Bateria Motor baja
     else if ((alarmas & 2) == 2 && (SEND_SMS_ALARM & 2) == 0)
     {
        begin_sms();
        GSM.println(F("Voltaje bateria motor bajo!"));
        SEND_SMS_ALARM = SEND_SMS_ALARM | 2; // bit(2) a 1
        end_SMS();
     }
     // Alerta Bateria Servicio baja
     else if ((alarmas & 4) == 4 && (SEND_SMS_ALARM & 4) == 0)
     {
        begin_sms();
        GSM.println(F("Voltaje bateria servicio bajo!"));
	SEND_SMS_ALARM = SEND_SMS_ALARM | 4; // bit(3) a 1
        end_SMS();
     }
     // Alerta Gas detectado
     else if ((alarmas & 8) == 8 && (SEND_SMS_ALARM & 8) == 0)
     {
        begin_sms();
        GSM.println(F("Detectado gas!"));
	SEND_SMS_ALARM = SEND_SMS_ALARM | 8; // bit(4) a 1
        end_SMS();
     }
     // Alerta Intruso detectado
     else if ((alarmas & 16) == 16 && (SEND_SMS_ALARM & 16) == 0)
     {
        begin_sms();
        GSM.println(F("Detectado intruso a bordo!"));
	SEND_SMS_ALARM = SEND_SMS_ALARM | 16; // bit(5) a 1
        end_SMS();
     }
  }
}

void begin_sms()
{ 
   // Secuencia de inicio de envio de
   // mensaje SMS
   GSM.println(F("AT+CMGF=1"));
   delay(100);
   ProcessGSM();
   
   comm_msg = "AT+CMGS=\"";
   comm_msg.concat(tlf_sms);
   comm_msg.concat("\"");
   GSM.println(comm_msg);
   delay(100);
   ProcessGSM();
}

void end_SMS()
{
     // Secuencia de finalizacion de
     // mensaje SMS
     delay(100);
     ProcessGSM();  
     GSM.println((char)26); //ASCII para ctrl+z
     delay(100);
     ProcessGSM();
     delay(500); 
}

//
// Recepcion mensajes modem GSM
//
void ProcessGSM()
{
  // Chequear estado del modem GSM
  volatile char c;
  
  if (GSM.available())  // Verifica si existe mensaje del modem GSM
  {
    String st;
    gsmbuf = "";
    int i;
    while(GSM.available()) // Leer caracteres del mensaje del modem
    {
      c = GSM.read();
      gsmbuf = gsmbuf + c;  // Escribe caracteres en el buffer
      // Revisa si se trata una llamada entrante ->
      comm_msg = "+CLIP: \"";
      comm_msg.concat(tlf_auth);
      comm_msg.concat("\"");
      if (gsmbuf.indexOf(comm_msg)>0)
      {
        SMSEstado();
      } // <- Revisa si se trata una llamada entrante
    }
    // Revisa mensaje de operador conectado ->
    i = gsmbuf.indexOf("+COPS: 0");
    if (i>-1)
    {    
      st = gsmbuf.substring(i,gsmbuf.length());
      if (st.length()>16)
      {
        netGSM = 1;
        debug("Conn a red GSM",0);
      }
      else
      {
        netGSM = 0;
        debug("Sin red GSM",2);
      }    
    }    // <-- Revisa mensaje de operador conectado
    
    // Revisa SMS recibido ->
    i = gsmbuf.indexOf("+CMT: ");
    if (i>-1)
    {
      i = gsmbuf.indexOf("\n",i+1);
      if (i>-1)
      {
         st = gsmbuf.substring(i+1,gsmbuf.length()-2); // Todo el mensaje
      }     
      processCommand(st); // Revisa si es un comando conocido
    }  // <-- Revisa SMS recibido

  }

}

//
//  Chequea conexion del operador de red
//
void ChkGSM()
{
  
  if (millis() > chk_m_milis)
   chk_m_milis = millis() + MODEM_TIME_CHK; // Proximo chequeo red GSM
  else
    return;

  GSM.println(F("AT+COPS?"));
  delay(100);
  ProcessGSM();
  
}

//
//  Envia SMS de estado
//
void SMSEstado()
{
  Serial.print(gsmbuf);
  // Avisar de envio en el LCD
  lcd.clear();
  lcd.home ();
  lcd.print("Enviar SMS"); // Avisar de envio en el LCD
  lcd.setCursor(0,1);
  lcd.print("Estatus");
  page_d_milis = millis() + LCD_TIME_PAG; // Nuevo tiempo para cambiar pagina display
  // Preparar SMS de estatus
  debug("Enviar mensaje!!!",1);
  GSM.println(F("ATH")); // Colgar llamada entrante
  ProcessGSM();
  delay(500);
  GSM.println(F("AT+CMGF=1"));
  delay(100);
  ProcessGSM();
  
  comm_msg = "AT+CMGS=\"";
  comm_msg.concat(tlf_sms);
  comm_msg.concat("\"");
  GSM.println(comm_msg);
  delay(300);

  ProcessGSM();
  comm_msg = "B.Motor: ";
  comm_msg.concat(checkBatt("motor"));
  GSM.println(comm_msg);
  delay(100);
  ProcessGSM();
  comm_msg = "B.Serv.: ";
  comm_msg.concat(checkBatt("servicio"));
  GSM.println(comm_msg);
  ProcessGSM();
  delay(100);
  if (digitalRead(WATER)==LOW)
    GSM.println(F("Sentina VACIA"));
  else
    GSM.println(F("Sentina LLENA"));
  delay(100);
  ProcessGSM();
  comm_msg = "Achiques: ";
  comm_msg.concat(num_starts);
  GSM.print(comm_msg);
  delay(100);
  ProcessGSM();
  if ((alarmas & 1) == 1) // Alarma Demasidas marchas de bomba
    GSM.println(F("ALARMA ACHIQUE!"));
  delay(100);  
  GSM.println((char)26);//ASCII para ctrl+z
  delay(300);
  ProcessGSM();
}

//
// Equivalente a pulsar boton "power" en shield GSM
//
void GSMPower()
{
  pinMode(RESET_GSM, OUTPUT); 
  digitalWrite(RESET_GSM, LOW);
  delay(1000);
  digitalWrite(RESET_GSM,HIGH);
  delay(2000);
  digitalWrite(RESET_GSM,LOW);
  delay(4000);
  GSM.println(F("AT+CNMI=2,2,0,0,0")); // Visualizar SMS entrantes
  delay(100);
  ProcessGSM();
  chk_m_milis = millis() + MODEM_TIME_CHK; // Proximo chequeo red GSM
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Comunicaciones GSM
//
//  Gestion bomba achique >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//
//
// Arranca bomba achique
//
void startPump()
{
  digitalWrite(PUMP, LOW);
  debug_msg = "Bomba se activada";
  debug(debug_msg, 1);
}

//
// Para bomba achique
//
void stopPump()
{
  digitalWrite(PUMP, HIGH);
  debug_msg = "Bomba se apaga";
  debug(debug_msg, 1);
}

//
// Gestiona parada automatica bomba achique
//
void autoPump(){
  if (flag_a == 1)
  {
    // Revisa el numero maximo de puestas en marcha automaticas
    // si se supera el umbral enviar una alarma
    if (millis() > fin_time_starts) // Periodo evaluacion marchas bomba ->
    {
      debug_msg = "Empieza periodo evaluacion marchas bomba";
      debug(debug_msg, 1);

      fin_time_starts = millis() + PUMP_TIME_MAX_START; // Establece nuevo limite para revisar
      num_starts = 0;
    }
    else
    {
      alarmas = alarmas & 127; // Pone a 0 el bit(1)
      if (num_starts > PUMP_MAX_START) // Demasiadas marchas de bomba ->
      {
        alarmas = alarmas | 1; // Hay peligro de hundimiento. Pone a 1 el bit(1)
        debug_msg = "ALARMA DEMASIADAS ACTIVACIONES AUTOMATICAS DE BOMBA ";
        debug_msg.concat(num_starts);
        debug(debug_msg, 1);
        comm_msg = debug_msg;
        debug(comm_msg, 2);
        flag_a = 0; // Impide marcha bomba
        digitalWrite(PUMP, HIGH);
        return;
       } // <- Demasiadas marchas de bomba
    } // <- Periodo evaluacion marchas bomba

    debug_msg = "Bomba Activada con auto paro en ";
    debug_msg.concat(PUMP_TIME_AUTO/1000);
    debug_msg.concat(" segundos");
    debug(debug_msg, 1);

    num_starts ++;
    flag_a = 2;
    digitalWrite(PUMP, LOW);
    stop_b_milis = millis() + PUMP_TIME_AUTO;
  }
  else if (flag_a == 2 && millis() > stop_b_milis)
  {
    debug_msg = "Bomba Apagada automaticamente ";
    debug_msg.concat(PUMP_TIME_AUTO/1000);
    debug_msg.concat(" segundos");
    debug(debug_msg, 1);

    flag_a = 0;
    digitalWrite(PUMP, HIGH);
  }
 
}
//  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Gestion bomba achique 
//
// Gestion Carga bateria >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//
// Chequea voltaje baterias
//
String checkBatt(String batt)
{
  volatile int s;
  volatile int n;
  volatile float v;
  volatile float vi;
  char buff[10];
  
  if (batt == "motor")
  {
        n = 0;
        v = 0.0;
        while (n<5)
        {
          s = analogRead(VBATM);
          vi = (s * 0.0048875)*3.875;
          if (v<vi)
            v = vi;
          delay(50);
          n ++;
        }
        debug_msg = "Voltaje Motor:";
        alarmas = alarmas & 253; // Pone a 0 el bit(2)
        if (v < MIN_VOL && v > 1.0)
        {
          debug("Motor voltaje bajo !!", 2);
          alarmas = alarmas | 2; // Bateria motor baja. Pone a 1 el bit(2) 
        }

  } 
   else
  {
        n = 0;
        v = 0.0;
        while (n<5)
        {
          s = analogRead(VBATS);
          vi = (s * 0.0048875)*3.875;
          if (v<vi)
            v = vi;
          delay(50);
          n ++;
        }
        debug_msg = "Voltaje Servicio:";

        alarmas = alarmas & 251; // Pone a 0 el bit(3) 
        if (v < MIN_VOL && batt != "motor")
        {
           debug("Servicio voltaje bajo !!", 2);
           alarmas = alarmas | 4; // Bateria servicio baja. Pone a 1 el bit(3) 
        }
  }

 

  debug_msg.concat(dtostrf(v,3,1,buff));  
  debug(debug_msg, 1);
  
  return dtostrf(v,3,1,buff);
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Gestion Carga bateria
//
// Gestion agua en la sentina >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//

void checkWater()
{
   waterState = digitalRead(WATER);

  // Chequea el estado de la boya.
  // si el estado es HIGH entonces hay agua
  if (waterState == HIGH) {     
    debug_msg = "Hay agua en la sentina";
    debug(debug_msg, 1);
    
    if (flag_a == 0) // Mira si bomba no activa y existe agua ->
    {
      flag_a = 1; // Activacion con parada automatica de bomba achique
    }// <- Mira si bomba no activa y existe agua
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Gestion agua en la sentina
//
// Gestion detector gas butano >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//
// Chequea sensor de gas butano
//
void checkGas()
{
   alarmas = alarmas & 247; // Pone a 0 el bit(4) 
   volatile bool sensorgas = digitalRead(SGAS);

  // Chequea el sensor de gas.
  // si el estado es LOW entonces hay gas
  if (sensorgas == LOW) {     
    alarmas = alarmas | 8; // Activa alarma de gas. Pone a 1 el bit(4) 
    debug_msg = "Detecta gas butano";
    debug(debug_msg, 1);
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Gestion detector gas butano
//
// Gestion robo >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//
// Gestion de intruso
//
void checkRobo()
{
   alarmas = alarmas & 239; // Alarma intruso. Pone a 0 el bit(5)
   if (digitalRead(ROBO) == HIGH)
   {
      alarmas = alarmas | 16; // Activa alarma intruso. Pone a 1 el bit(5)  
      debug_msg = "Detecta intruso";
      debug(debug_msg, 1);
   }
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Gestion robo
//
// Status
//

void LCDStatus()
{
  if (millis() > page_d_milis)
  {
    page_d_milis = millis() + LCD_TIME_PAG; // Nuevo tiempo para cambiar pagina display
    lcd.init();           // Inicializa el lcd 
    lcd.home();
    if (num_dis==0)
    {
      // Pagina 1
      lcd.print(F("B.Motor:"));
      lcd.setCursor(9,0);
      lcd.print(checkBatt("motor"));
      lcd.setCursor(0,1);
      lcd.print(F("B.Serv.:"));
      lcd.setCursor(9,1);
      lcd.print(checkBatt("servicio"));
      num_dis = 1;
    } else if (num_dis==1)
    {
      // Pagina 2    
      if (digitalRead(WATER)==LOW)
        lcd.print(F("Sentina VACIA"));
      else
        lcd.print(F("Sentina LLENA"));
      lcd.setCursor(0,1);
      comm_msg = "Achiques: ";
      comm_msg.concat(num_starts);
      lcd.print(comm_msg);
      num_dis = 2;
    } else if (num_dis==2)
    {
      // Pagina 3    
      lcd.print(F("SMS A:"));
      lcd.print(tlf_sms);
      lcd.setCursor(0,1);
      if (netGSM==0)
        lcd.print(F("GSM No conec."));
      else
        lcd.print(F("GSM Conectado"));
      if (alarmas!=0) // Alarma Demasidas marchas de bomba
        num_dis = 3;
      else
        num_dis = 0;
      
    } else if (num_dis==3)
    {
      // Pagina 4 
      if ((alarmas & 1) == 1)
      {   
        lcd.clear();
        lcd.home();
        lcd.print(F("ALARMA ACHIQUE!"));
        delay(1500);
      }
      if ((alarmas & 8) == 8)
      {  
        lcd.clear();
        lcd.home();
        lcd.print(F("ALARMA GAS!"));
        delay(1500);
      }
      if ((alarmas & 16) == 16)
      { 
        lcd.clear();
        lcd.home();
        lcd.print(F("ALARMA INTRUSO!"));
        delay(1500);
      }
      num_dis=0;
      page_d_milis=0;
    }
    
  }
}

//
//  Recepción del comando
//
void reciveCommand(){
 // Lee caracteres hasta fin de comando
 while(Serial.available() > 0) {

    c_string[sindex] = Serial.read();

    if(c_string[sindex++] == END_CMD)
    {
      c_string[sindex-1] = '\0';
      processCommand(c_string);
      c_string[0]='\0';
      sindex = 0;
    }
 }
}

//
//  Procesar comando recibido
//
void processCommand(String cmd) {
    //Procesa comandos recibidos
    debug_msg = "Recibido cmd:";
    debug_msg.concat(cmd);
    debug_msg.concat(" long:");
    debug_msg.concat(cmd.length());
    debug(debug_msg, 1);
   
    // Comando bomba achique ->
    if (cmd.indexOf(PUMP_HEADER) == 0 && cmd.length()==PUMP_MSG_LEN)
    {
      char c = cmd[1];
      
      if (c == 'S') // Acciones ->
      {
        startPump(); // Arranca bomba
      }
      else if (c == 'A')
      {
        flag_a = 1; // Activacion con paro automatico
      }
      else
      {
        stopPump(); // Para bomba
      }// <- Acciones
    }// <- Comando bomba achique
    
        
    // Comando reset alarmas ->
    if (cmd.indexOf(SETUP_HEADER)==0 && cmd.length()== SETUP_MSG_LEN)
    {
      char c = cmd[1];
      
      if (c == 'A') // Acciones ->
      {
        num_starts = 0;     // Resetea contador bomba
	SEND_SMS_ALARM = 0; // SMS alarmas enviadas activar. Mapbit a 0
	alarmas = 0;        // Desactiva todas las alarmas. Mapbit a 0
      }
      
    }// <- Comando reset alarmas
      
    // Comando para guardar TLF SMS y TLF AUTH->
    if (cmd.indexOf(TLFSMS_HEADER)==0 && cmd.length()== TLFSMS_LEN)
    {
      // Guarda en la eeprom tlf_sms
      for (int i = 0; i < 11; i++)
      {
        char c = cmd[i+3];
        EEPROM.write(i,c);
      }
      // Guarda en la eeprom tlf_auth
      for (int i = 0; i < 9; i++)
      {
        char c = cmd[i+5];
        EEPROM.write(i+11,c);
      }
     
    }// <- Comando para guardar TLF SMS y TLF AUTH

}// <- processCommand

//
// Gestion de depuración
//
void debug(String txt, int tipo)
{
    if (INFO_ENABLED and tipo == 0)
    {
      Serial.println(F("INFO------------------------>"));
      Serial.println(txt);
      Serial.println(F("<------------------------INFO"));
    } 
    else if (DEBUG_ENABLED and tipo == 1)
    {
      Serial.println(F("DEBUG----------------------->"));
      Serial.println(txt);
      Serial.println(F("<-----------------------DEBUG"));
    } 
    else if (ALERT_ENABLED and tipo == 2)
    {
      Serial.println(F("ALERT----------------------->"));
      Serial.println(txt);
      Serial.println(F("<-----------------------ALERT"));  
    }
    
}
