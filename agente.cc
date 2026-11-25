/*
 ROBOT LABERINTO 4x4 - DOBLE CRUCE DE CINTA POR ACCION
 =====================================================
 Arduino UNO + L298N + 3 FC-51 digitales.
 IN1 D8, IN2 D7, IN3 D6, IN4 D5, ENA D9, ENB D10 (PWM).
 FC-51: izquierda A1, central A2, derecha A0.
 Alimentacion motores independiente adecuada; GND comun. Quitar jumpers ENA/ENB.

 RUTA A*: 14 -> 13 -> 9 -> 5 -> 6 -> 2 -> 1, mirando norte al inicio.
 Se agrupan avances consecutivos en un mismo rumbo (5 avances + 5 giros).

 CADA ACCION (giro o avance) ES ATOMICA:
   IDA     BLANCO -> NEGRO -> BLANCO     (1er cruce completo)
   VUELTA  BLANCO -> NEGRO -> BLANCO     (2do cruce completo)
 SOLO al completar ambos cruces se consume la accion y empieza la siguiente.
 Para giros: SOLO FC-51 del lado del giro, ignora otros sensores.
 Para avance: central decide el final; laterales corrigen continuamente el rumbo.

 ADVERTENCIA: doble cruce COMPLETO de la pared frontal implica salir
 DEL OTRO LADO de la cinta antes de regresar; esto puede ser peligroso en
 bordes de mesa o barreras reales. Probar SOBRE EL SUELO, no una mesa,
 a baja velocidad. Siempre debe quedar superficie segura tras la cinta.
 Si se desea evitar cruzar la pared, poner PARED_FRONTAL_DOBLE_COMPLETO=false:
 se tocara negro de ida y se saldra de negro en retroceso, sin cruzarla entera.
 Esa alternativa segura NO es equivalente a dos cruces completos.

 Los FC-51 no miden distancia, angulo ni posicion; una cinta distinta puede
 confundirse con la esperada. Limites de seguridad detienen ante fallo.
*/
#include <Arduino.h>
enum TipoAccion : uint8_t { GIRAR_IZQ, GIRAR_DER, AVANZAR };
enum FaseCruce : uint8_t {
  ESPERAR_BLANCO_INICIAL,
  IDA_BUSCAR_NEGRO, IDA_SALIR_NEGRO,
  VUELTA_BUSCAR_NEGRO, VUELTA_SALIR_NEGRO,
  COMPLETO
};
// ===================== HARDWARE ==============================
const uint8_t IN1=8, IN2=7, IN3=6, IN4=5, ENA=9, ENB=10;
const uint8_t IR_IZQ=A1, IR_CEN=A2, IR_DER=A0;
const bool MOTOR_A_ES_IZQUIERDO=true;
const bool INVERTIR_MOTOR_A=false, INVERTIR_MOTOR_B=false;
const uint8_t NIVEL_NEGRO=HIGH; // Verificar en monitor serie

// ===================== CONFIGURACION =========================
const uint8_t PWM_AVANCE=85;
const uint8_t PWM_GIRO=130;
const uint8_t PWM_RETORNO_GIRO=110;
const uint8_t PWM_RETORNO_RECTO=100;
const int8_t COMPENSA_DER=0;
const uint8_t CONTROL_MS=18;
const uint8_t CONFIRMAR_NEGRO=1, CONFIRMAR_BLANCO=3;

// true = dos cruces COMPLETOS incluso al avanzar hacia una pared.
// false = acercarse hasta negro y salir en reversa (opcion mas segura).
const bool PARED_FRONTAL_DOBLE_COMPLETO=true;  // Pedido: ida y vuelta sobre la MISMA cinta

// Limites de seguridad por fase (NO se usan como criterio de exito).
const unsigned long LIMITE_BUSCAR_GIRO_MS=7000UL;
const unsigned long LIMITE_BUSCAR_PARED_MS=11500UL;
const unsigned long LIMITE_CRUZAR_NEGRO_MS=650UL;  // Evitar avanzar demasiado fuera del borde
const unsigned long LIMITE_RETORNO_HASTA_NEGRO_MS=2500UL;
const unsigned long LIMITE_SALIR_NEGRO_MS=2000UL;
const unsigned long LIMITE_DESPEGUE_INICIAL_MS=1200UL;
const unsigned long LIMITE_TOTAL_MS=150000UL;
const unsigned long LIMITE_CORRECCION_LATERAL_MS=2700UL;
const uint16_t EXTRA_RETRO_PARED_MS=200; // tras volver a blanco, separarse un poco de la cinta

// PID digital con memoria de la ultima cinta lateral.
const float KP=33.0f, KI=0.35f, KD=14.0f;
const int16_t CORRECCION_MINIMA=55;
const int16_t CORRECCION_FUERTE=78;
const int16_t CORRECCION_MAXIMA=90;
const uint16_t ESCALAR_CORRECCION_MS=350;
const uint8_t PWM_BASE_CORRECCION=120;

// 0=laberinto; 1=avanzar hasta cinta dos veces; 2=girar izquierda;
// 3=girar derecha; 4=prueba giro DERECHO + avance (zona 13->9).
const uint8_t MODO_PRUEBA=0;

// ===================== MAPA 4x4 / A* ========================
const uint8_t N_CELDAS=16, MN=1, ME=2, MS=4, MW=8;
const uint8_t BIT_MURO[4]={MN,ME,MS,MW};
const int8_t DR[4]={-1,0,1,0}, DC[4]={0,1,0,-1};
const uint8_t INICIO=14, META=0;
uint8_t muros[N_CELDAS];
uint8_t rumbo=0; // 0=N,1=E,2=S,3=W
unsigned long inicioGlobal=0;

void ponerMuro(uint8_t f,uint8_t c,uint8_t d) {
  muros[f*4+c]|=BIT_MURO[d];
  const int8_t nf=(int8_t)f+DR[d], nc=(int8_t)c+DC[d];
  if(nf>=0 && nf<4 && nc>=0 && nc<4)
    muros[nf*4+nc]|=BIT_MURO[(d+2)%4];
}
void cargarMapa() {
  for(uint8_t i=0;i<N_CELDAS;i++) muros[i]=0;
  for(uint8_t i=0;i<4;i++) {
    ponerMuro(0,i,0); ponerMuro(3,i,2);
    ponerMuro(i,0,3); ponerMuro(i,3,1);
  }
  ponerMuro(0,0,1); ponerMuro(0,1,2);
  ponerMuro(0,2,1); ponerMuro(1,2,1);
  ponerMuro(1,0,2); ponerMuro(1,2,2);
  ponerMuro(2,1,1); ponerMuro(3,0,1);
  ponerMuro(2,3,2);
}
uint8_t heuristica(uint8_t a,uint8_t b) {
  return abs((int)a/4-(int)b/4)+abs((int)a%4-(int)b%4);
}
uint8_t aEstrella(uint8_t inicio,uint8_t meta,uint8_t camino[]) {
  uint8_t g[N_CELDAS], tmp[N_CELDAS];
  int8_t padre[N_CELDAS];
  bool abierto[N_CELDAS], cerrado[N_CELDAS];
  for(uint8_t i=0;i<N_CELDAS;i++) {
    g[i]=255;padre[i]=-1;abierto[i]=false;cerrado[i]=false;
  }
  g[inicio]=0;abierto[inicio]=true;
  while(true) {
    int8_t actual=-1; uint16_t mejor=65535;
    for(uint8_t i=0;i<N_CELDAS;i++)
      if(abierto[i] && (uint16_t)g[i]+heuristica(i,meta)<mejor) {
        mejor=(uint16_t)g[i]+heuristica(i,meta);actual=i;
      }
    if(actual<0) return 0;
    if((uint8_t)actual==meta) break;
    abierto[actual]=false;cerrado[actual]=true;
    for(uint8_t d=0;d<4;d++) {
      if(muros[actual]&BIT_MURO[d]) continue;
      const int8_t f=(int8_t)(actual/4)+DR[d];
      const int8_t c=(int8_t)(actual%4)+DC[d];
      if(f<0 || f>3 || c<0 || c>3) continue;
      const uint8_t v=(uint8_t)(f*4+c);
      if(cerrado[v]) continue;
      if(g[actual]+1<g[v]) {
        g[v]=g[actual]+1;padre[v]=actual;abierto[v]=true;
      }
    }
  }
  uint8_t n=0;
  for(int8_t u=(int8_t)meta;u!=-1;u=padre[u]) {
    if(n>=N_CELDAS) return 0;
    tmp[n++]=(uint8_t)u;
  }
  for(uint8_t i=0;i<n;i++) camino[i]=tmp[n-1-i];
  return n;
}

// =============== COLA DE ACCIONES ATOMICAS ===================

struct Accion { TipoAccion tipo; uint8_t desde, hasta; };
const uint8_t MAX_ACCIONES=32;
Accion acciones[MAX_ACCIONES];
uint8_t totalAcciones=0, accionActual=0;
uint8_t direccionPaso(uint8_t a,uint8_t b) {
  int8_t d=(int8_t)b-(int8_t)a;
  return (d==-4) ? 0 : (d==1) ? 1 : (d==4) ? 2 : 3;
}

bool agregarAccion(TipoAccion tipo,uint8_t desde,uint8_t hasta) {
  if(totalAcciones>=MAX_ACCIONES) return false;
  acciones[totalAcciones].tipo=tipo;
  acciones[totalAcciones].desde=desde;
  acciones[totalAcciones].hasta=hasta;
  ++totalAcciones;
  return true;
}
bool planificarAcciones(const uint8_t camino[],uint8_t n) {
  totalAcciones=0;accionActual=0;
  uint8_t rumboPlan=rumbo;
  for(uint8_t i=0;i+1<n;) {
    const uint8_t dir=direccionPaso(camino[i],camino[i+1]);
    uint8_t fin=i+1;
    while(fin+1<n && direccionPaso(camino[fin],camino[fin+1])==dir) ++fin;
    const uint8_t giro=(dir+4-rumboPlan)%4;
    if(giro==1 && !agregarAccion(GIRAR_DER,camino[i],camino[i])) return false;
    if(giro==3 && !agregarAccion(GIRAR_IZQ,camino[i],camino[i])) return false;
    if(giro==2) {
      if(!agregarAccion(GIRAR_DER,camino[i],camino[i])) return false;
      if(!agregarAccion(GIRAR_DER,camino[i],camino[i])) return false;
    }
    if(!agregarAccion(AVANZAR,camino[i],camino[fin])) return false;
    rumboPlan=dir;i=fin;
  }
  return true;
}
const __FlashStringHelper* nombreAccion(TipoAccion t) {
  if(t==GIRAR_IZQ) return F("GIRAR IZQUIERDA");
  if(t==GIRAR_DER) return F("GIRAR DERECHA");
  return F("AVANZAR HASTA PARED");
}

// ========================== MOTORES ==========================
void canalMotor(uint8_t a,uint8_t b,uint8_t en,int16_t pwm,bool invertir) {
  pwm=(int16_t)constrain((int)pwm,-255,255);
  if(invertir) pwm=-pwm;
  if(pwm>0) {digitalWrite(a,HIGH);digitalWrite(b,LOW);}
  else if(pwm<0) {digitalWrite(a,LOW);digitalWrite(b,HIGH);}
  else {digitalWrite(a,LOW);digitalWrite(b,LOW);}
  analogWrite(en,(uint8_t)abs(pwm));
}
void motores(int16_t izq,int16_t der) {
  const int16_t a=MOTOR_A_ES_IZQUIERDO?izq:der;
  const int16_t b=MOTOR_A_ES_IZQUIERDO?der:izq;
  canalMotor(IN1,IN2,ENA,a,INVERTIR_MOTOR_A);
  canalMotor(IN3,IN4,ENB,b,INVERTIR_MOTOR_B);
}
void detener() {
  // Frenado electrico corto para disminuir inercia del giro o avance.
  analogWrite(ENA,255); analogWrite(ENB,255);
  digitalWrite(IN1,HIGH);digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);digitalWrite(IN4,HIGH);
  delay(30);
  motores(0,0);
}
void pivot(int8_t signo,uint8_t pwm) {
  // +1: derecha; -1: izquierda. Motor izquierdo y derecho opuestos.
  motores((int16_t)signo*pwm,-(int16_t)signo*pwm);
}
bool tiempoGlobalAgotado() {
  return (unsigned long)(millis()-inicioGlobal)>LIMITE_TOTAL_MS;
}

// =================== LECTURAS FILTRADAS =====================
struct Sens {bool izq,cen,der;};
struct Sens leerSensores() {
  uint8_t izq=0,cen=0,der=0;
  for(uint8_t i=0;i<3;i++) {
    if(digitalRead(IR_IZQ)==NIVEL_NEGRO) ++izq;
    if(digitalRead(IR_CEN)==NIVEL_NEGRO) ++cen;
    if(digitalRead(IR_DER)==NIVEL_NEGRO) ++der;
    delay(1);
  }
  Sens s={izq>=2,cen>=2,der>=2};
  return s;
}
// En giros SOLO se lee el sensor elegido. No central, no opuesto.
bool leerLateralNegro(bool derecha) {
  const uint8_t pin=derecha?IR_DER:IR_IZQ;
  uint8_t cuenta=0;
  for(uint8_t i=0;i<3;i++) {
    if(digitalRead(pin)==NIVEL_NEGRO) ++cuenta;
    delay(1);
  }
  return cuenta>=2;
}

// =================== CORRECCION LATERAL =====================
struct Correccion {
  int8_t sentido; // +1 corregir a DERECHA (cinta IZQ), -1 a IZQUIERDA
  int8_t previo;
  uint8_t blancos;
  float integral;
  unsigned long inicio;
};
void reiniciarCorreccion(struct Correccion &c) {
  c.sentido=0;c.previo=0;c.blancos=0;c.integral=0;c.inicio=millis();
}
// Usar durante avance Y retroceso lineal sin interferir con el sensor central.
// Devuelve false si un lateral nunca se despeja (seguridad).
bool conducirRectoConCorreccion(bool retrocediendo,Correccion &c) {
  const Sens s=leerSensores();
  const unsigned long ahora=millis();
  // Ambos laterales sobre negro pueden indicar que el robot cruza una
  // banda frontal ancha. No aplicar un giro lateral arbitrario en ese caso.
  if(s.izq && s.der) {
    reiniciarCorreccion(c);
    const int16_t base=PWM_BASE_CORRECCION;
    if(retrocediendo) motores(-base,-base-COMPENSA_DER);
    else motores(base,base+COMPENSA_DER);
    return true;
  }
  int8_t nuevo=0;
  if(s.izq && !s.der) nuevo=+1;
  if(s.der && !s.izq) nuevo=-1;

  if(nuevo!=0 && nuevo!=c.sentido) {
    c.sentido=nuevo;c.blancos=0;c.integral=0;c.previo=0;c.inicio=ahora;
    Serial.println(nuevo>0 ? F("Lateral IZQ negro: corregir DERECHA")
                           : F("Lateral DER negro: corregir IZQUIERDA"));
  } else if(c.sentido!=0) {
    if(s.izq || s.der) c.blancos=0;
    else if(c.blancos<CONFIRMAR_BLANCO) ++c.blancos;
    if(c.blancos>=CONFIRMAR_BLANCO) {
      reiniciarCorreccion(c);
      Serial.println(F("Laterales blancos: retomar recto"));
    }
  }

  if(c.sentido!=0 && (s.izq || s.der) &&
     (unsigned long)(ahora-c.inicio)>LIMITE_CORRECCION_LATERAL_MS) return false;

  int16_t base=retrocediendo?PWM_RETORNO_RECTO:PWM_AVANCE;
  int16_t ajuste=0;
  if(c.sentido!=0) {
    c.integral=constrain(c.integral+c.sentido,-8.0f,8.0f);
    const float u=KP*c.sentido+KI*c.integral+KD*(c.sentido-c.previo);
    int16_t mag=(int16_t)abs((int)u);
    if(mag<CORRECCION_MINIMA) mag=CORRECCION_MINIMA;
    if((unsigned long)(ahora-c.inicio)>ESCALAR_CORRECCION_MS && mag<CORRECCION_FUERTE)
      mag=CORRECCION_FUERTE;
    if(mag>CORRECCION_MAXIMA) mag=CORRECCION_MAXIMA;
    ajuste=c.sentido*mag;
    base=PWM_BASE_CORRECCION;
    c.previo=c.sentido;
  }
  // El mismo signo de ajuste gira el CHASIS en la misma direccion tanto
  // marcha adelante como marcha atras (aunque cambie el vector de avance).
  if(retrocediendo) motores(-base+ajuste,-base-ajuste-COMPENSA_DER);
  else motores(base+ajuste,base-ajuste+COMPENSA_DER);
  return true;
}

// ============= MAQUINA DE ESTADOS DOBLE CRUCE ================

struct Cruce {
  FaseCruce fase;
  uint8_t repetidas;
  unsigned long faseDesde;
};
void cambiarFase(struct Cruce &c,FaseCruce fase) {
  c.fase=fase;c.repetidas=0;c.faseDesde=millis();
}
// Devuelve true solo al completar los DOS cruces.
// Retorna false al agotar el plazo: caller detiene el robot.
bool ejecutarDobleCruce(TipoAccion tipo) {
  const bool giro=(tipo!=AVANZAR);
  const bool derecha=(tipo==GIRAR_DER);
  const int8_t signoGiro=derecha?+1:-1;
  const bool centralSolo=(tipo==AVANZAR);
  const bool cruceCompleto= giro || PARED_FRONTAL_DOBLE_COMPLETO;
  Correccion lateral;reiniciarCorreccion(lateral);

  // Solo el sensor central decide finalizacion de avance.
  // En giros no se consultan los otros dos FC-51.
  bool negro=centralSolo ? leerSensores().cen : leerLateralNegro(derecha);
  Cruce c;
  cambiarFase(c,negro?ESPERAR_BLANCO_INICIAL:IDA_BUSCAR_NEGRO);
  bool retrocediendo=false;

  Serial.println(giro?F("Sensado del giro: solo el lateral correspondiente")
                     :F("Avance: central para pared; laterales para PID"));
  if(!cruceCompleto) Serial.println(F("MODO SEGURO FRONTAL: contacto negro -> reversa -> blanco"));

  while(!tiempoGlobalAgotado()) {
    delay(CONTROL_MS);
    negro=centralSolo ? leerSensores().cen : leerLateralNegro(derecha);

    // La lectura debe mantenerse varias muestras consecutivas.
    // No es posible completar dos fases con una sola lectura.
    bool esperadoNegro=(c.fase==IDA_BUSCAR_NEGRO || c.fase==VUELTA_BUSCAR_NEGRO);
    if(c.fase==ESPERAR_BLANCO_INICIAL || c.fase==IDA_SALIR_NEGRO ||
       c.fase==VUELTA_SALIR_NEGRO) esperadoNegro=false;
    if(negro==esperadoNegro) {
      if(c.repetidas<255) ++c.repetidas;
    } else c.repetidas=0;
    const bool confirmado=c.repetidas >= (esperadoNegro?CONFIRMAR_NEGRO:CONFIRMAR_BLANCO);

    if(confirmado) {
      switch(c.fase) {
        case ESPERAR_BLANCO_INICIAL:
          Serial.println(F("Blanco inicial: habilitar primera deteccion"));
          cambiarFase(c,IDA_BUSCAR_NEGRO);
          break;
        case IDA_BUSCAR_NEGRO:
          Serial.println(F("IDA: NEGRO detectado (entrando a cinta)"));
          if(!cruceCompleto) {
            // Variante segura: primero contacto, luego retorno a blanco.
            detener();retrocediendo=true;reiniciarCorreccion(lateral);
            cambiarFase(c,VUELTA_SALIR_NEGRO);
          } else cambiarFase(c,IDA_SALIR_NEGRO);
          break;
        case IDA_SALIR_NEGRO:
          Serial.println(F("CRUCE 1/2: salio a BLANCO del otro lado"));
          detener();retrocediendo=true;reiniciarCorreccion(lateral);
          cambiarFase(c,VUELTA_BUSCAR_NEGRO);
          break;
        case VUELTA_BUSCAR_NEGRO:
          Serial.println(F("VUELTA: NEGRO detectado (recruzando cinta)"));
          cambiarFase(c,VUELTA_SALIR_NEGRO);
          break;
        case VUELTA_SALIR_NEGRO:
          // Solo despues del SEGUNDO cruce completo se puede finalizar.
          // En linea recta, separarse un poco mas de la pared antes del giro.
          if(!giro) {
            const unsigned long inicioExtra = millis();
            while((unsigned long)(millis()-inicioExtra)<EXTRA_RETRO_PARED_MS &&
                  !tiempoGlobalAgotado()) {
              // Velocidad moderada; se conserva el sentido de retroceso.
              motores(-PWM_RETORNO_RECTO, -PWM_RETORNO_RECTO-COMPENSA_DER);
              delay(CONTROL_MS);
            }
          }
          detener();
          Serial.println(F("CRUCE 2/2: BLANCO recuperado. ACCION COMPLETA."));
          return true;
        case COMPLETO: break;
      }
    }

    unsigned long limite=0;
    switch(c.fase) {
      case ESPERAR_BLANCO_INICIAL: limite=LIMITE_DESPEGUE_INICIAL_MS;break;
      case IDA_BUSCAR_NEGRO: limite=giro?LIMITE_BUSCAR_GIRO_MS:LIMITE_BUSCAR_PARED_MS;break;
      case IDA_SALIR_NEGRO: limite=LIMITE_CRUZAR_NEGRO_MS;break;
      case VUELTA_BUSCAR_NEGRO: limite=LIMITE_RETORNO_HASTA_NEGRO_MS;break;
      case VUELTA_SALIR_NEGRO: limite=LIMITE_SALIR_NEGRO_MS;break;
      case COMPLETO: limite=0;break;
    }
    if((unsigned long)(millis()-c.faseDesde)>limite) {
      detener();
      Serial.print(F("ERROR: timeout en fase "));
      Serial.println((int)c.fase);
      return false;
    }

    if(giro) {
      // Deteccion del giro solo A0 o A1, NO se leen otros sensores.
      pivot(retrocediendo?-signoGiro:signoGiro,
            retrocediendo?PWM_RETORNO_GIRO:PWM_GIRO);
    } else {
      // El sensor CENTRAL decide cuando terminar el tramo, mientras
      // las lecturas laterales generan correcciones de trayectoria.
      if(!conducirRectoConCorreccion(retrocediendo,lateral)) {
        detener();Serial.println(F("ERROR: lateral no libera negro"));
        return false;
      }
    }
  }
  detener();Serial.println(F("ERROR: limite global de navegacion"));
  return false;
}

// ========================= PROGRAMA ===========================
void mostrarPlan() {
  Serial.print(F("ACCIONES PROGRAMADAS: "));Serial.println(totalAcciones);
  for(uint8_t i=0;i<totalAcciones;i++) {
    Serial.print(i+1);Serial.print(F("/"));Serial.print(totalAcciones);
    Serial.print(F(" "));Serial.print(nombreAccion(acciones[i].tipo));
    Serial.print(F(" ["));Serial.print(acciones[i].desde);
    Serial.print(F(" -> "));Serial.print(acciones[i].hasta);
    Serial.println(F("]"));
  }
}
bool ejecutarPlan() {
  while(accionActual<totalAcciones) {
    const Accion &a=acciones[accionActual];
    Serial.print(F("INICIA ACCION "));Serial.print(accionActual+1);
    Serial.print(F("/"));Serial.print(totalAcciones);
    Serial.print(F(" "));Serial.println(nombreAccion(a.tipo));
    if(!ejecutarDobleCruce(a.tipo)) {
      detener();
      Serial.print(F("DETENIDO: accion NO consumida: "));
      Serial.println(accionActual+1);
      return false;
    }
    // SOLO se incrementa despues de completar dos cruces confirmados.
    ++accionActual;
    Serial.print(F("ACCION CONSUMIDA: "));Serial.print(accionActual);
    Serial.print(F("/"));Serial.println(totalAcciones);
    delay(80);
  }
  return true;
}
void setup() {
  Serial.begin(115200);
  pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);pinMode(ENB,OUTPUT);
  pinMode(IR_IZQ,INPUT);pinMode(IR_CEN,INPUT);pinMode(IR_DER,INPUT);
  motores(0,0);
  cargarMapa();
  uint8_t camino[N_CELDAS];
  const uint8_t n=aEstrella(INICIO,META,camino);
  if(n==0 || !planificarAcciones(camino,n)) {
    Serial.println(F("ERROR: no se pudo calcular el plan"));return;
  }
  Serial.print(F("A*: "));
  for(uint8_t i=0;i<n;i++) {Serial.print(camino[i]);if(i+1<n) Serial.print(F(" -> "));}
  Serial.println();
  mostrarPlan();
  Serial.println(F("INICIO en celda 14 mirando al NORTE. Arranca en 3s."));
  delay(3000);
  inicioGlobal=millis();
  bool ok=false;
  if(MODO_PRUEBA==1) ok=ejecutarDobleCruce(AVANZAR);
  else if(MODO_PRUEBA==2) ok=ejecutarDobleCruce(GIRAR_IZQ);
  else if(MODO_PRUEBA==3) ok=ejecutarDobleCruce(GIRAR_DER);
  else if(MODO_PRUEBA==4) ok=ejecutarDobleCruce(GIRAR_DER) && ejecutarDobleCruce(AVANZAR);
  else ok=ejecutarPlan();
  detener();
  Serial.println(ok?F("LISTO: todas las acciones confirmadas.")
                   :F("PARADA DE SEGURIDAD: revisar fase y cinta."));
}
void loop() {}
