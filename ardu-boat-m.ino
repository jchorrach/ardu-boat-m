Skip to content
 
This repository
Explore
Gist
Blog
Help
jchorrach jchorrach
 
 
1  Unwatch
Star 0 Fork 0PUBLICjchorrach/ardu-boat-m
 branch: master  ardu-boat-m / monitor.ino 
jchorrach jchorrach 29 minutes ago Mejora protocolo llamadas perdidas
1 contributor
 file  634 lines (553 sloc)  15.661 kb EditRawBlameHistory Delete
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
61
62
63
64
65
66
67
68
69
70
71
72
73
74
75
76
77
78
79
80
81
82
83
84
85
86
87
88
89
90
91
92
93
94
95
96
97
98
99
100
101
102
103
104
105
106
107
108
109
110
111
112
113
114
115
116
117
118
119
120
121
122
123
124
125
126
127
128
129
130
131
132
133
134
135
136
137
138
139
140
141
142
143
144
145
146
147
148
149
150
151
152
153
154
155
156
157
158
159
160
161
162
163
164
165
166
167
168
169
170
171
172
173
174
175
176
177
178
179
180
181
182
183
184
185
186
187
188
189
190
191
192
193
194
195
196
197
198
199
200
201
202
203
204
205
206
207
208
209
210
211
212
213
214
215
216
217
218
219
220
221
222
223
224
225
226
227
228
229
230
231
232
233
234
235
236
237
238
239
240
241
242
243
244
245
246
247
248
249
250
251
252
253
254
255
256
257
258
259
260
261
262
263
264
265
266
267
268
269
270
271
272
273
274
275
276
277
278
279
280
281
282
283
284
285
286
287
288
289
290
291
292
293
294
295
296
297
298
299
300
301
302
303
304
305
306
307
308
309
310
311
312
313
314
315
316
317
318
319
320
321
322
323
324
325
326
327
328
329
330
331
332
333
334
335
336
337
338
339
340
341
342
343
344
345
346
347
348
349
350
351
352
353
354
355
356
357
358
359
360
361
362
363
364
365
366
367
368
369
370
371
372
373
374
375
376
377
378
379
380
381
382
383
384
385
386
387
388
389
390
391
392
393
394
395
396
397
398
399
400
401
402
403
404
405
406
407
408
409
410
411
412
413
414
415
416
417
418
419
420
421
422
423
424
425
426
427
428
429
430
431
432
433
434
435
436
437
438
439
440
441
442
443
444
445
446
447
448
449
450
451
452
453
454
455
456
457
458
459
460
461
462
463
464
465
466
467
468
469
470
471
472
473
474
475
476
477
478
479
480
481
482
483
484
485
486
487
488
489
490
491
492
493
494
495
496
497
498
499
500
501
502
503
504
505
506
507
508
509
510
511
512
513
514
515
516
517
518
519
520
521
522
523
524
525
526
527
528
529
530
531
532
533
534
535
536
537
538
539
540
541
542
543
544
545
546
547
548
549
550
551
552
553
554
555
556
557
558
559
560
561
562
563
564
565
566
567
568
569
570
571
572
573
574
575
576
577
578
579
580
581
582
583
584
585
586
587
588
589
590
591
592
593
594
595
596
597
598
599
600
601
602
603
604
605
606
607
608
609
610
611
612
613
614
615
616
617
618
619
620
621
622
623
624
625
626
627
628
629
630
631
632
633
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
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
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
#define TLF_CALL "+***********" // Telefono permitido requerimiento SMS
#define TLF_SMS "+***********" // Telefono al que se envian los SMS
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
  Serial.begin(9600);    // Inicializa monitor Serie
  GSM.begin(19200);     // Inicializa Modem GSM
  GSMPower();           // Activa shiled GSM
  lcd.begin(16,2);           // Inicializa el lcd 
  lcd.backlight();
  lcd.home ();               // Primera linea y columna del display
  lcd.print("ARDU-BOAT-M");  
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
  GSM.println("AT+CMGS=\""+TLF_SMS+"\"");
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
      if (buffer.indexOf("+CLIP: \""+TLF_CALL+"\"")>0)
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
  GSM.println("AT+CMGS=\""+TLF_SMS+"\"");
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
  GSM.print("AT+CNMI=2,2,0,0,0"); // Visualizar SMS entrantes
  ProcessGSM();
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
 
Status API Training Shop Blog About © 2014 GitHub, Inc. Terms Privacy Security Contact
