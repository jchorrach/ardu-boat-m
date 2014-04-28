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
*/

#include <Time.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>



#define END_CMD '\r'       // Fin linea de comando
#define DEBUG_ENABLED true // Activa/desactiva debug

//--------------------------------------------------------------
// Display LCD
// 
// LCD1602 controlado via adaptador I2C
//
#define LCD_TIME_PAG 3500   // ms para cambio de pagina display LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
byte num_dis = 0;            // Pagina que se enseña en el display
long page_d_milis;           // Tiempo para que se cambie pagina el LCD


#define TIME_HEADER "T"  // Tag del comando Fecha/hora
#define TIME_MSG_LEN 11  // Longitud del comando Fecha/hora

//--------------------------------------------------------------
// Comunicaciones GSM
//
// SIM900 shield
//
#define MODEM_HEADER "$"  // Reenvio de comando al modem GSM
#define MODEM_TIME_CHK 14000 // ms de funcionamiento en modo automatico
#define TLF_CALL "639635751" // Telefono permitido requerimiento SMS
#define TLF_SMS "+34639635751" // Telefono al que se envian los SMS
SoftwareSerial GSM(7, 8); // Usa sensores digitales 7RX y 8TX
String buffer;          // buffer array para recibir datos
long chk_m_milis = 0;   // Tiempo en que deberia pararse automaticamente la bomba
byte netGSM = 0;        // Indica si hay conexion de red GSM (0 No, 1 Si)
byte flag_SMSAlert = 0; // Si 1 entonces ya se ha enviado SMSAlert


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
// S
//
#define STATUS_HEADER "S" // Tag del comando status
#define STATUS_MSG_LEN 1  // Longitud del comando status

//--------------------------------------------------------------
// Nivel del agua en sentina
// W
//
#define WATER 11         // Nivel del agua (Digital In)
#define WATER_HEADER "W" // Tag del comando nivel de agua sentina
#define WATER_MSG_LEN 1  // Longitud del comando nivel de agua sentina
#define WATER_INTERVAL_VAL 5000  // Intervalo ms para comparar niveles

int waterState = 0; // Estado del sensor boya

//--------------------------------------------------------------
// Comado bomba de achique
// A<valor>
// AS -> En marcha bomba de achique 
// AA -> En marcha bomba de achique con stop automatico
// AN -> Para bomba de achique
//
#define PUMP 12          // Control rele bomba (Digital Out)
#define PUMP_HEADER "A"  // Tag del comado Bomba achique
#define PUMP_MSG_LEN 2   // Longitud del comando Bomba achique
#define PUMP_TIME_AUTO 12000 // ms de funcionamiento en modo automatico
#define PUMP_MAX_START 4     // Maximo numero de activaciones por periodo
#define PUMP_TIME_MAX_START 400000 // Periodo en el que se evalua el numero de marchas de la bomba

byte flag_a = 0;     // Si 0 sin activacion 1 inicio bomba si 2 esperando duracion para apagar
long stop_b_milis;   // Tiempo en que deberia pararse automaticamente la bomba
byte num_starts = 0; // Conteo de marchas automaticas bomba
String debug_msg;    // Mensaje de debug
String comm_msg;     // Mensaje de comunicaciones
//--------------------------------------------------------------

char c_string[20]; // String que contendra el comando recibido
int sindex = 0;    // Indice del proximo caracter en el string

long fin_time_starts; // Tiempo de inicio de evaluacin marchas bomba

void setup()
{
  Serial.begin(9600);   // Inicializa monitor Serie
  GSM.begin(19200);     // Inicializa Modem GSM
  lcd.init();           // Inicializa el lcd 
  lcd.backlight();
  lcd.home ();               // Primera linea y columna del display
  lcd.print("ARDU-BOAT-M");
  GSMPower();                // Activa shiled GSM
  lcd.setCursor ( 0, 1 );    // Segunda linea del display
  lcd.print ("Ok");  
  pinMode(PUMP, OUTPUT);
  pinMode(WATER, INPUT);
  digitalWrite(WATER, LOW);  // Habilita internal pullup
  digitalWrite(PUMP, HIGH);
  setTime(12,0,0,1,1,12);    // Fecha por defecto sin sincronizar
  
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
  // Lee comandos modem GSM
  ProcessGSM();
  // Chequeo modem GSM
  ChkGSM();
  // Display
  LCDStatus();
  
  delay(500);
}


//------------------------------------------------------

void debug(String msg)
{
    if (DEBUG_ENABLED)
    {
      Serial.print(formatDate());
      Serial.print(": ");
      Serial.println(msg);
    }
}

String formatDate(){
  // Formatea la fecha/hora "hh:mi:ss dd/mm/yy"
  String fecFmt = "";
  fecFmt.concat(lpadDigits(hour())+":");
  fecFmt.concat(lpadDigits(minute())+":");
  fecFmt.concat(lpadDigits(second())+" ");  
  fecFmt = fecFmt + lpadDigits(day())+"/";    
  fecFmt = fecFmt + lpadDigits(month())+"/";
  fecFmt = fecFmt + year();
  return fecFmt;
}


//
// utilidad para incluir 0 a la izquierda en valores numericos
//
String lpadDigits(int digits){
  String sdigits = String(digits);
  if(digits < 10)
    sdigits.concat("0");
  return sdigits;
}

//
// Comunicaciones GSM
//

void SMSAlert()
{
  if (flag_SMSAlert == 0)
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
  GSM.println("Demasiados achiques!");
  ProcessGSM();
  delay(300);
  comm_msg = " Achiques: ";
  comm_msg.concat(num_starts);
  GSM.print(comm_msg);
  ProcessGSM();
  delay(300);
  GSM.println((char)26);//ASCII para ctrl+z
  ProcessGSM();
  delay(500);  
  flag_SMSAlert = 1;
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
    buffer = "";
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
    if (buffer.indexOf("+COPS: 0)")>0)      
      if (buffer.length()>10)
        netGSM = 1;
      else
        netGSM = 0;    
    // <-- Revisa mensaje de operador conectado
    
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
  Serial.println("Enviar mensaje!!!");
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
  if (num_starts > PUMP_MAX_START) // Alarma Demasidas marchas de bomba
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


//
//  Procesar comando recibido
//
void processCommand(String cmd) {
    //Procesa comandos recibidos
    debug_msg = "Recibido cmd:";
    debug_msg.concat(cmd);
    debug(debug_msg);
    // Comando lectura nivel sentina ->
    if (cmd.indexOf(WATER_HEADER) == 0&&cmd.length()==WATER_MSG_LEN)
    {
      comm_msg = "Valor nivel agua sentina: ";
      if (digitalRead(WATER) == HIGH)
      {
        comm_msg.concat("LLENO");
        sendAlert(comm_msg);
      }
      else
      {
        comm_msg.concat("VACIO");
        sendAlert(comm_msg);
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
        num_starts = 0;    // Resetea contador bomba
        flag_SMSAlert = 0; // Habilitab envio SMSAlert
      }
      
    }// <- Comando reset alarmas
    
    // Comando para el modem GSM
    if (cmd.indexOf(MODEM_HEADER)==0)
    {
      int i = cmd.indexOf('#');
      GSM.println(cmd.substring(0,i));
      ProcessGSM();
      delay(200);
    }
}
   
  
//
//  Gestion bomba achique
//
void startPump()
{
  digitalWrite(PUMP, LOW);
  debug_msg = "Bomba se activada";
  debug(debug_msg);
}

void stopPump()
{
  digitalWrite(PUMP, HIGH);
  debug_msg = "Bomba se apaga";
  debug(debug_msg);
}


void autoPump(){
  if (flag_a == 1)
  {
    // Revisa el numero maximo de puestas en marcha automaticas
    // si se supera el umbral enviar una alarma
    if (millis() > fin_time_starts) // Periodo evaluacion marchas bomba ->
    {
      debug_msg = "Empieza periodo evaluacion marchas bomba";
      debug(debug_msg);

      fin_time_starts = millis() + PUMP_TIME_MAX_START; // Establece nuevo limite para revisar
      num_starts = 0;
    }
    else
    {
      if (num_starts > PUMP_MAX_START) // Demasiadas marchas de bomba ->
      {
        debug_msg = "ALARMA DEMASIADAS ACTIVACIONES AUTOMATICAS DE BOMBA ";
        debug_msg.concat(num_starts);
        debug(debug_msg);
        comm_msg = debug_msg;
        sendAlert(comm_msg);
        SMSAlert();
        flag_a = 0; // Impide marcha bomba
        digitalWrite(PUMP, HIGH);
        return;
       } // <- Demasiadas marchas de bomba
    } // <- Periodo evaluacion marchas bomba

    debug_msg = "Bomba Activada con auto paro en ";
    debug_msg.concat(PUMP_TIME_AUTO/1000);
    debug_msg.concat(" segundos");
    debug(debug_msg);

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
    debug(debug_msg);

    flag_a = 0;
    digitalWrite(PUMP, HIGH);
  }
 
}


//
// Gestion Carga bateria
//

//-> TO DO Leer valor analgico tension bateria

String checkBatt(String batt)
{
  if (batt == "motor")
  {
  	return "12.32";
  } else if (batt == "servicio")
  {
  	return "12.38";
  }
  return "0.0";  
}

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
  
  comm_msg = "Bat. Motor: ";
  comm_msg.concat(checkBatt("motor"));
  comm_msg.concat("\n\r");
  comm_msg.concat("Bat. Servicio: ");
  comm_msg.concat(checkBatt("servicio"));
  comm_msg.concat("\n\r");
  if (digitalRead(WATER)==LOW)
    comm_msg.concat("Sentina: vacia\n\r");
  else
    comm_msg.concat("Sentina: llena\n\r");
  comm_msg.concat("\n\r");  comm_msg.concat("Achiques: ");
  comm_msg.concat(num_starts);
  comm_msg.concat("\n\r");
  if (num_starts > PUMP_MAX_START) // Demasidas marchas de bomba
  {
    comm_msg.concat("ALARMA ACHIQUE!");
  }
  sendAlert(comm_msg);
}

//
// Gestion agua en la sentina
//

void checkWater()
{
   waterState = digitalRead(WATER);

  // Chequea el estado de la boya.
  // si el estado es HIGH entonces hay agua
  if (waterState == HIGH) {     
    debug_msg = "Hay agua en la sentina";
    debug(debug_msg);
    
    if (flag_a == 0) // Mira si bomba no activa y existe agua ->
    {
      flag_a = 1; // Activacion con parada automatica de bomba achique
    }// <- Mira si bomba no activa y existe agua
  } /*
  else {
    debug_msg = "Sentina vacia";
    debug(debug_msg);
  }*/
}

//
// Mensajes en puerto serie
//

void sendAlert(String msg)
{
  Serial.println("-Comm------------------------------");
  Serial.println(formatDate());
  Serial.println(msg);
  Serial.println("-----------------------------------");  
}
