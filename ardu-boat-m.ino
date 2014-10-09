/*********************************************************
**
**  ARDU-BOAT-M
**  sketch monitorización del barco con arduino
**
**  -Revisa agua en la sentina
**  -Marcha/Paro bomba achique
**  -Tension en bateria
**  -Comunicacion GSM
**
*********************************************************

Mapa de pines:
------------------------------------
00: Rx Serial
01: Tx Serial
02: 
03:
04:
05:
06:
07: Rx tarjeta GSM
08: Tx tarjeta GSM
09: Reset tarjeta GSM out
10:
11: Sensor agua [WATER] in
12: relay bomba achique [PUMP] out
13: Sensor gas butano [GAS] in
A0: Bateria Servicio [VBATS] in
A1: Bateria Motor [VBATM] in
A2:
A3:
A4: SDA para LCD
A5: SCL para LCD

*/


#include <stdlib.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#define NAME "ARDU-BOAT-M"
#define VERSION "Beta"

#define END_CMD '\r'       // Fin linea de comando
#define INFO_ENABLED true  // Activa/desactiva debug información (0)
#define DEBUG_ENABLED true // Activa/desactiva debug (1)
#define ALERT_ENABLED true // Activa/desactiva debug alertas (2)

//--------------------------------------------------------------
// Display LCD
// 
// LCD1602 controlado via adaptador I2C
//
#define LCD_TIME_PAG 3500   // ms para cambio de pagina display LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
byte num_dis = 0;            // Pagina que se enseña en el display
long page_d_milis;           // Tiempo para que se cambie pagina el LCD


//--------------------------------------------------------------
// Comunicaciones GSM
//
// SIM900 shield
//
// GA -> pulsa el boton power en la shield GSM
//
#define GSM_HEADER "G"  // Tag del comado Bomba achique
#define GSM_MSG_LEN 2   // Longitud del comando Bomba achique
#define MODEM_HEADER "$"  // Reenvio de comando al modem GSM
#define MODEM_TIME_CHK 14000 // ms de funcionamiento en modo automatico
#define TLF_CALL "********"   // Telefono permitido requerimiento SMS
#define TLF_SMS "+**********" // Telefono al que se envian los SMS
SoftwareSerial GSM(7, 8); // Usa sensores digitales 7RX y 8TX
String buffer;          // buffer array para recibir datos
long chk_m_milis = 0;   // Tiempo en que deberia pararse automaticamente la bomba
byte netGSM = 0;        // Indica si hay conexion de red GSM (0 No, 1 Si)
byte SEND_SMS_se = 0; // Si 1 entonces ya se ha enviado SMS de sentina
byte SEND_SMS_bm = 0; // Si 1 entonces ya se ha enviado SMS de voltaje bateria motor bajo
byte SEND_SMS_bs = 0; // Si 1 entonces ya se ha enviado SMS de voltaje bateria servicio bajo
byte SEND_SMS_ga = 0; // Si 1 entonces ya se ha enviado SMS de detectado gas

//--------------------------------------------------------------
// Setup
// S<valor>
// SW5 -> Aplica el valor al parametro de nivel agua
// SA0 -> Asigna valor al contador de intentos Achique
//
#define SETUP_HEADER "S" // Tag del comando setup
#define SETUP_MSG_LEN 3  // Longitud del comando setup


//--------------------------------------------------------------
// Status del sistema
//
// S
//
#define STATUS_HEADER "S" // Tag del comando status
#define STATUS_MSG_LEN 1  // Longitud del comando status

//--------------------------------------------------------------
// Nivel del agua en sentina
//
// W
//
#define WATER 11         // Nivel del agua (Digital In)
#define WATER_HEADER "W" // Tag del comando nivel de agua sentina
#define WATER_MSG_LEN 1  // Longitud del comando nivel de agua sentina
#define WATER_INTERVAL_VAL 5000  // Intervalo ms para comparar niveles

int waterState = 0; // Estado del sensor boya
byte alarma_se = 0; // Si 0 no hay peligro hundimiento, si 1 peligro hundimiento

//--------------------------------------------------------------
// Bomba de achique
//
// A<valor>
// AS -> En marcha bomba de achique 
// AA -> En marcha bomba de achique con stop automatico
// AN -> Para bomba de achique
//
#define PUMP 12              // Control rele bomba (Digital Out)
#define PUMP_HEADER "A"      // Tag del comado Bomba achique
#define PUMP_MSG_LEN 2       // Longitud del comando Bomba achique
#define PUMP_TIME_AUTO 12000 // ms de funcionamiento en modo automatico
#define PUMP_MAX_START 4     // Maximo numero de activaciones por periodo
#define PUMP_TIME_MAX_START 400000 // Periodo en el que se evalua el numero de marchas de la bomba ms

byte flag_a = 0;     // Si 0 sin activacion 1 inicio bomba si 2 esperando duracion para apagar
long stop_b_milis;   // Tiempo en que deberia pararse automaticamente la bomba
byte num_starts = 0; // Conteo de marchas automaticas bomba

//---------------------------------------------------------------
// Lectura de tension de las baterias
//
//
#define VBATS 0      // Pin analogico para leer tension bat. servicio
#define VBATM 1      // Pin analogico para leer tension bat. motor
#define MIN_VOL 12.1 // Valor de alarma para voltaje bajo
byte alarma_bm = 0;  // Si 0 bateria motor cargada, si 1 bateria motor baja
byte alarma_bs = 0;  // Si 0 bateria servicio cargada, si 1 bateria servicio baja

/---------------------------------------------------------------
// Lectura de sensor gas butano
//
//
#define GAS 13       // Pin para leer sensor de gas
int gasState = 0;    // Estado del sensor gas
byte alarma_ga = 0;  // Si 0 no detecta gas, si 1 se detecta gas

//--------------------------------------------------------------

String debug_msg;     // Mensaje de debug
String comm_msg;      // Mensaje de comunicaciones

char c_string[20];    // String que contendra el comando recibido
int sindex = 0;       // Indice del proximo caracter en el string

long fin_time_starts; // Tiempo de inicio de evaluacion marchas bomba

//**********************************************************************************************************************

void setup()
{
  Serial.begin(9600);     // Inicializa monitor Serie
  GSM.begin(19200);       // Inicializa Modem GSM
  lcd.init();             // Inicializa el lcd 
  lcd.backlight();        // Activa iluminación LCD
  lcd.home ();            // Primera linea y columna del display
  lcd.print(NAME);
  GSMPower();             // Activa shiled GSM
  lcd.setCursor ( 0, 1 ); // Segunda linea del display
  lcd.print (VERSION);  

  // Inicializar Pines
  pinMode(PUMP, OUTPUT);
  pinMode(WATER, INPUT);
  digitalWrite(WATER, HIGH);  // Habilita internal pullup
  digitalWrite(PUMP, HIGH);
 
  flag_a = 0;                 // Bomba automatica desactivada

  fin_time_starts = PUMP_TIME_MAX_START;  // Tiempo limite para evaluar marchas automaticas
  
  page_d_milis = millis() + LCD_TIME_PAG; // Establece tiempo para cambio de pagina en LCD
}

void loop()
{

  if (Serial.available())
  {
    reciveCommand();
  }

  // Chequeo detector gas
  checkGas();

  // Chequeo agua en la sentina
  checkWater();

  // Chequeo estado funcionamiento bomba automatico
  autoPump();

  // Funcionalidad GSM
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
  SMSAlert();   // Gestion alertas SMS
}

//
//  Gestion de envio de SMS de alertas
//
void SMSAlert()
{

  if (netGSM == 0)
  {
    return; // El GSM no esta conectado
  }

  if ((SEND_SMS_se == 0 and alarma_se == 1) or 
      (SEND_SMS_bm == 0 and alarma_bm == 1) or
      (SEND_SMS_bs == 0 and alarma_bs == 1) or
      (SEND_SMS_ga == 0 and alarma_ga == 1))
  {
     GSM.println("AT+CMGF=1");
     ProcessGSM();
     delay(300);
     comm_msg = "AT+CMGS=\"";
     comm_msg.concat(TLF_SMS);
     comm_msg.concat("\"");
     GSM.println(comm_msg);
     ProcessGSM();
     delay(300);

     // Alerta Demasiados Achiques
     if (alarma_se == 1)
     {
        GSM.println("Demasiados achiques!");
        ProcessGSM();
        delay(300);
        comm_msg = " Achiques: ";
        comm_msg.concat(num_starts);
        GSM.print(comm_msg);
        SEND_SMS_se = 1;
     } 
     // Alerta Bateria Motor baja
     else if (alarma_bm == 1)
     {
        GSM.println("Voltaje bateria motor bajo!");
        SEND_SMS_bm = 1;
     }
     // Alerta Bateria Servicio baja
     else if (alarma_bs == 1)
     {
        GSM.println("Voltaje bateria servicio bajo!");
        SEND_SMS_bs = 1;
     }

     ProcessGSM();
     delay(300);
     GSM.println((char)26);//ASCII para ctrl+z
     ProcessGSM();
     delay(500);  
  }
}


//
// Recepcion mensajes modem GSM
//
void ProcessGSM()
{
  // Chequear estado del modem GSM
  char c;
  
  if (GSM.available())  // Verifica si existe mensaje del modem GSM
  {
    String st;
    buffer = "";
    int i;
    while(GSM.available()) // Leer caracteres del mensaje del modem
    {
      c = GSM.read();
      buffer = buffer + c;  // Escribe caracteres en el buffer
      // Revisa si se trata una llamada entrante ->
      comm_msg = "+CLIP: \"";
      comm_msg.concat(TLF_CALL);
      comm_msg.concat("\"");
      if (buffer.indexOf(comm_msg)>0)
      {
        SMSEstado();
      } // <- Revisa si se trata una llamada entrante
    }
    // Revisa mensaje de operador conectado ->
    i = buffer.indexOf("+COPS: 0");
    if (i>-1)
    {    
      st = buffer.substring(i,buffer.length());
       if (st.length()>16)
        netGSM = 1;
      else
        netGSM = 0;    
    }    // <-- Revisa mensaje de operador conectado

    Serial.print("GSM>");
    Serial.print(buffer);  // Imprime mensaje en el puerto Serie
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
  GSM.println("AT+COPS?");
  ProcessGSM();
}


//
//  Envia SMS de estado
//
void SMSEstado()
{
  Serial.print(buffer);
  // Avisar de envio en el LCD
  lcd.clear();
  lcd.home ();
  lcd.print("Enviar SMS"); // Avisar de envio en el LCD
  lcd.setCursor(0,1);
  lcd.print("Estatus");
  page_d_milis = millis() + LCD_TIME_PAG; // Nuevo tiempo para cambiar pagina display
  // Preparar SMS de estatus
  debug("Enviar mensaje!!!",1);
  GSM.println("ATH"); // Colgar llamada entrante
  ProcessGSM();
  delay(500);
  GSM.println("AT+CMGF=1");
  ProcessGSM();
  delay(300);
  comm_msg = "AT+CMGS=\"";
  comm_msg.concat(TLF_SMS);
  comm_msg.concat("\"");
  GSM.println(comm_msg);
  ProcessGSM();
  delay(300);
  comm_msg = "B.Motor: ";
  comm_msg.concat(checkBatt("motor"));
  GSM.println(comm_msg);
  ProcessGSM();
  delay(300);
  comm_msg = "B.Serv.: ";
  comm_msg.concat(checkBatt("servicio"));
  GSM.println(comm_msg);
  ProcessGSM();
  delay(300);
  if (digitalRead(WATER)==LOW)
    GSM.println("Sentina VACIA");
  else
    GSM.println("Sentina LLENA");
  ProcessGSM();
  delay(300);
  comm_msg = "Achiques: ";
  comm_msg.concat(num_starts);
  GSM.print(comm_msg);
  ProcessGSM();
  delay(300);
  if (alarma_se == 1) // Alarma Demasidas marchas de bomba
    GSM.println("ALARMA ACHIQUE!");
  delay(300);  
  GSM.println((char)26);//ASCII para ctrl+z
  ProcessGSM();
  delay(500);
}

//
// Equivalente a pulsar boton "power" en shield GSM
//
void GSMPower()
{
  pinMode(9, OUTPUT); 
  digitalWrite(9,LOW);
  delay(1000);
  digitalWrite(9,HIGH);
  delay(2000);
  digitalWrite(9,LOW);
  delay(4000);
  GSM.println("AT+CNMI=2,2,0,0,0"); // Visualizar SMS entrantes
  ProcessGSM();
  delay(200);
  chk_m_milis = millis() + MODEM_TIME_CHK; // Proximo chequeo red GSM
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

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Comunicaciones GSM


//
//  Gestion bomba achique >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
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
      alarma_se = 0;
      if (num_starts > PUMP_MAX_START) // Demasiadas marchas de bomba ->
      {
        alarma_se = 1; // Hay peligor de hundimiento
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
  int s;
  float v;
  char buff[10];
  debug_msg = "";
  if (batt == "motor")
  {
        s = analogRead(VBATM);
        v = (s * 0.0048875)*3.875;
        debug_msg = "Voltaje Motor:";

        alarma_bm = 0; 
        if (v < MIN_VOL)
        {
           alarma_bm = 1; // Bateria motor baja
        }

  } 
   else
  {
        s = analogRead(VBATS);
        v = (s * 0.0048875)*3.875;
        debug_msg = "Voltaje Servicio:";

        alarma_bs = 1;
        if (v < MIN_VOL && batt <> "motor")
        {
           debug("Servicio voltaje bajo !!", 2);
           alarma_bs = 1; //Bateria servicio baja
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
// Chequea sensor de agua sentina
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
   alarma_ga = 0;
   gasState = digitalRead(GAS);

  // Chequea el sensor de gas.
  // si el estado es HIGH entonces hay gas
  if (gasState == HIGH) {     
    alarma_ga = 1;
    debug_msg = "Detecta gas butano";
    debug(debug_msg, 1);
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Gestion detector gas butano

//
// Status
//

void LCDStatus()
{
  if (millis() > page_d_milis)
  {
    page_d_milis = millis() + LCD_TIME_PAG; // Nuevo tiempo para cambiar pagina display
    lcd.clear();
    lcd.home();
    if (num_dis==0)
    {
      // Pagina 1
      lcd.print("B.Motor:");
      lcd.setCursor(9,0);
      lcd.print(checkBatt("motor"));
      lcd.setCursor(0,1);
      lcd.print("B.Serv.:");
      lcd.setCursor(9,1);
      lcd.print(checkBatt("servicio"));
      num_dis = 1;
    } else if (num_dis==1)
    {
      // Pagina 2    
      if (digitalRead(WATER)==LOW)
        lcd.print("Sentina VACIA");
      else
        lcd.print("Sentina LLENA");
      lcd.setCursor(0,1);
      comm_msg = "Achiques: ";
      comm_msg.concat(num_starts);
      lcd.print(comm_msg);
      num_dis = 2;
    } else if (num_dis==2)
    {
      // Pagina 3    
      lcd.print("Modem GSM");
      lcd.setCursor(0,1);
      if (netGSM==0)
        lcd.print("No conectado");
      else
        lcd.print("Conectado");
      if (num_starts > PUMP_MAX_START) // Alarma Demasidas marchas de bomba
        num_dis = 3;
      else
        num_dis = 0;
      
    } else if (num_dis==3)
    {
      // Pagina 4    
      lcd.print("ALARMA ACHIQUE!");
      num_dis=0;
    }
    
  }
}

void status()
{
  
  debug_msg = "Bat. Motor: ";
  debug_msg.concat(checkBatt("motor"));
  debug_msg.concat("\n\r");
  debug_msg.concat("Bat. Servicio: ");
  debug_msg.concat(checkBatt("servicio"));
  debug_msg.concat("\n\r");
  if (digitalRead(WATER)==LOW)
    debug_msg.concat("Sentina: vacia\n\r");
  else
    debug_msg.concat("Sentina: llena\n\r");
  debug_msg.concat("\n\r");
  debug_msg.concat("Achiques: ");
  debug_msg.concat(num_starts);
  debug_msg.concat("\n\r");
  if (alarma_se == 1) // Demasidas marchas de bomba
  {
    debug_msg.concat("ALARMA ACHIQUE!\n\r");
  }
  if (alarma_bm == 1) // voltaje bat. motor bajo
  {
    debug_msg.concat("VOLTAJE BATERIA MOTOR BAJO!\n\r");
  }
  if (alarma_bs == 1) // voltaje bat. servicio bajo
  {
    debug_msg.concat("VOLTAJE BATERIA SERVICIO BAJO!\n\r");
  }

  debug(debug_msg, 0);
}


//
//  Procesar comando recibido
//
void processCommand(String cmd) {
    //Procesa comandos recibidos
    debug_msg = "Recibido cmd:";
    debug_msg.concat(cmd);
    debug(debug_msg, 1);
    // Comando lectura nivel sentina ->
    if (cmd.indexOf(WATER_HEADER) == 0&&cmd.length()==WATER_MSG_LEN)
    {
      debug_msg = "Valor nivel agua sentina: ";
      if (digitalRead(WATER) == HIGH)
      {
        debug_msg.concat("LLENO");
        debug(debug_msg, 0);
      }
      else
      {
        debug_msg.concat("VACIO");
        debug(debug_msg, 0);
      }
    }// <-- Comando lectura nivel sentina

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
    
    // Comando status ->
    if (cmd.indexOf(STATUS_HEADER) == 0 && cmd.length()==STATUS_MSG_LEN)
    {
      status();
    }// <- Comando status
    
    // Comando reset alarmas ->
    if (cmd.indexOf(SETUP_HEADER)==0 && cmd.length()== SETUP_MSG_LEN)
    {
      char c = cmd[1];
      
      if (c == 'A') // Acciones ->
      {
        num_starts = 0;  // Resetea contador bomba
        SEND_SMS_se = 0; // Habilita envio SMS alerta
        SEND_SMS_bm = 0; // Habilita envio SMS alerta
	SEND_SMS_bs = 0; // Habilita envio SMS alerta
        SEND_SMS_ga = 0; // Habilita envio SMS alerta
      }
      
    }// <- Comando reset alarmas
    
    // Comando para el modem GSM ->
    if (cmd.indexOf(MODEM_HEADER)==0)
    {
      int i = cmd.indexOf('#');
      GSM.println(cmd.substring(0,i));
      ProcessGSM();
      delay(200);
    }// <- Comando para el modem GSM
    
    // Comando para shield GSM ->
    if (cmd.indexOf(GSM_HEADER)==0)
    {
      char c = cmd[1];
      
      if (c == 'A') // Acciones ->
      {
        GSMPower();
      }
      
    }// <- Comando para shield GSM
    
}


//
// Gestion de depuración
//
void debug(String msg, int tipo)
{
    if (INFO_ENABLED and tipo == 0)
    {
      Serial.println("INFO------------------------>");
      Serial.println(msg);
      Serial.println("<------------------------INFO");
    } 
    else if (DEBUG_ENABLED and tipo == 1)
    {
      Serial.println("DEBUG----------------------->");
      Serial.println(msg);
      Serial.println("<-----------------------DEBUG");
    } 
    else if (ALERT_ENABLED and tipo == 2)
    {
      Serial.println("ALERT----------------------->");
      Serial.println(msg);
      Serial.println("<-----------------------ALERT");  
    }
}
