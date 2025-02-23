#include <Wire.h> 
#include "RTClib.h" 
#include <Adafruit_PWMServoDriver.h> 
// Variaveis 

#define SERVOMIN  217 // VALOR PARA UM PULSO MAIOR QUE 1 mS, 317 = valor intermediário -> motor parado 
#define SERVOMAX  417 // VALOR PARA UM PULSO MENOR QUE 2 mS 
#define STEP_MILLIS = 3600000;// ms -> 1hr é necessario fazer a conversão do passo para milissegundos 

const int MPU = 0x69; 
float AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;// variáveis acelerometro e giroscopio 
float posArray[12] = {0}; // 12 horas de operaçao 
float potArray[24] = {0}; // o dobro pois está sendo armazenado a potência na calibração a cada 30 min 
float vArray[24] = {0};  
int tArray[12] = {7,8,9,10,11,12,13,14,15,16,17,18}; // horas de operação 
unsigned long currentTime = 0; // variavel usado para manipular o valor da função millis() 
int calibration = 1; // se 1, a calibraçao ocorre. 
int t = 0; // variavel usado no loop no modo normal de operação 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 
RTC_DS1307 rtc; 

//HEADER DAS FUNCOES 
float read_accel(); 
float mAccel(); 
void writeServos(int posicao, int tempo); 
void beginServos(); 
void move_panel(float position); 
float mVolt(); 
float mAmp(); 
float mPot(); 
float find_ligth(float inicial_pos); 
void calibrate(); 

void setup() { 
// Inicializa Serial 
    Serial.begin(9600); 
    beginServos();// inicializa o objeto servomotor 
    Wire.beginTransmission(0x69); // configurando o mpu 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
    Wire.beginTransmission(0x69); // endereço padrao é x98, porém com o AD0, o 
    MPU operou no endereço x69 
    Wire.write(0x1B); 
 
 
    Wire.write(0b00000000); //Configura Giroscópio para escala de +/-250°/s 
    Wire.beginTransmission(0x69); 
    Wire.write(0x1C); 
    Wire.write(0b00000000); //Configura Acelerometro para escala de +/-2g 
    Wire.endTransmission(); 
    writeServos(0, 0); // garantindo que o servo inicie parado 
    Serial.println("initialling..."); 
} 
// ----ACELEROMETRO---- 
float read_accel() { 
    Wire.beginTransmission(MPU); 
    Wire.write(0x3B); 
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU, 14, true); 
    // Armazena o valor dos sensores nas variaveis correspondentes 
    AccX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
    AccY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L) 
    AccZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
    Temp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) 
    GyrX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) 
    GyrY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
    GyrZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
    
    /*  fundo de escala escolhido: 
            Acelerômetro 
            +/- 2g = 16384 
    */ 
    Wire.endTransmission(); 
    return (AccY / 16384); // os outros eixos sao irrelevantes para o projeto 
} 
 
float mAccel(){ // Media simples do acelerometro (filtro) 
    float soma = 0; 
    for(int i = 0; i < 100; i++){ 
        soma = soma + read_accel(); 
    } 
    float media = soma/100; 
    return media; 
} 
 
 
// SERVO MOTOR 
void writeServos(int nServo, int posicao) {  
    int pos = map ( posicao , -50 , 50 , SERVOMIN, SERVOMAX); 
    pwm.setPWM(nServo , 0, pos); 
} 
 
void beginServos() { 
    pwm.begin(); // INICIA O OBJETO PWM 
    pwm.setPWMFreq(60); // DEFINE A FREQUENCIA DE TRABALHO DO SERVO 
} 
 
void move_panel(float position) { 
    unsigned long previousTime = 0; 
    bool verify = true; 
    while(verify){ 
        float y = mAccel(); 
        float u = 0; 
        float p = 0; 
        float i = 0; 
        float sp = position; 
        float kp = 250;        // ganho Proporcional. 
        float ti = 2000;        // temp intg. 
        float ki = kp/ti;     //ganho int. 
        float e = sp - y; 
        while( -0.01 >= e || e >= 0.01){ 
            currentTime = millis(); 
            if((currentTime-previousTime)>100){ 
                previousTime=currentTime; 
                y  = mAccel(); 
                e = sp - y; // erro 
                p = kp*e;         // controle proporcional 
                i = i + (ki*e);   // controle integral 
                u = p+i;          // sinal de controle 
                if(u > 0.1 ){ 
                if (u < 25){ u = 25;}} 
                if(u < 0.1){  
                if(u > -25){ u = -25;}} 
                writeServos(0, u); 
            } 
        } 
        writeServos(0, 0); 
        float soma = 0; 
        float media = 0; 
        delay(1000); 
        float accel_vf = mAccel(); 
        if( position - 0.02 <= accel_vf && accel_vf <= position + 0.02){ 
            verify = false; 
        } 
    } 
    writeServos(0, 50); 
} 
 
float mVolt(){ 
    float R1 = 30000; 
    float R2 = 7500; 
    float vin = 0; 
    float vout = 0; 
    double soma = 0; 
    for (int i = 0; i < 1000; i++){ 
        vout = (analogRead(2)*5.0)/1024.0; 
        vin = vout / (R2/(R1+R2)); // sensor mede usando divisor de tensão 
        vin = vin - 0.4; 
        if(vin < 0) vin = 0; 
        soma += vin; 
        delay(1); 
    }
    return (soma/1000); 
} 
 
float mAmp() { // corrente média 
  float sensibilidade = 0.185; //- Para 5A,  sensibilidade = 0.185; 
  float volts = 0; 
  float Amp = 0; 
  for(int i = 0; i < 1000; i++){ 
    volts = (analogRead(1) - 510); 
    Amp += pow(volts,2); 
  } 
  Amp = (sqrt(Amp/1000))*0.004887586; 
  float mAmp = Amp/sensibilidade; 
  if (mAmp < 0.08) mAmp = 0; 
  return mAmp; 
} 
 
float mPot(){ // potencia media calculada 
  float volts = mVolt(); 
  float current = mAmp(); 
  float media = volts*current; 
  return media; 
} 
 
float find_light(float initial_pos){ // funcao principal que faz varredura, armazena o ponto de maior tensao. 
    float max_pot_pos = -0.45; 
    float end = 0.55; 
    float aux = initial_pos; 
    float max_pot = 0; 
    while(aux <= 0.55){ 
        move_panel(aux); 
        float pot = mPot(); 
        if(max_pot < pot){  
        max_pot = pot; 
        max_pot_pos = aux; 
        } 
        aux += 0.05; 
        delay(1000); 
    } 
    return max_pot_pos; 
} 
 
void calibrate(){ 
    Serial.println ("CALIBRANDO..."); 
    unsigned long previousTime2 = 0; 
    unsigned long previousTime3 = 0; 
    int c = 1; 
    int k = 1; 
    int j; 
    float inicial_pos = -0.45; 
    posArray[0] = find_light(inicial_pos); // primeira interação movendo o painel para a posição inicial 
    move_panel(posArray[0]); 
    potArray[0] = mPot(); 
    vArray[0] = mVolt(); 
    while (c < 12){ // quantia de horas de operação -1 (pois a primeira verificação ja aconteceu no inicio) 
        currentTime = millis(); 
        if((currentTime - previousTime2) > STEP_MILLIS){ // timer, step_millis é o tempo declarado no inicio do programa 
            previousTime2 = currentTime; // timer 
            posArray[c] = find_light(posArray[c-1]); // faz a varredura a partir da melhor posiçao passada 
            move_panel(posArray[c]); // move o painel para a posiçao ideal para continuar o processo de geração 
            c++; 
            j = 0; 
            for (int i = 0; i < c; i++){ // print das medições de potência 
                Serial.print(tArray[i]); 
                Serial.print(":00 | POSICAO: "); 
                Serial.print(posArray[i]); 
                Serial.print(" | tensao: "); 
                Serial.print(vArray[j]); 
                Serial.print("V"); 
                Serial.print(" | Potencia: "); 
                Serial.print(potArray[j]); 
                Serial.println("W"); 
                j++; 

                Serial.print(tArray[i]); 
                Serial.print(":30 | POSICAO: "); 
                Serial.print(posArray[i]); 
                Serial.print(" | tensao: "); 
                Serial.print(vArray[j]); 
                Serial.print("V"); 
                Serial.print(" | Potencia: "); 
                Serial.print(potArray[j]); 
                Serial.println("W"); 
                j++; 
        }
    } 
    if((currentTime - previousTime3) > (step_millis/2)){ // leitura dos sensores a cada 30 min 
        previousTime3 = currentTime; 
        potArray[k] = mPot(); 
        vArray[k] = mVolt(); 
        k++; 
    } 
    delay(100); // delay padrao 
  } 
} 
 
void loop() { 
    if(calibration){ 
        calibrate(); 
        calibration = 0; 
    } 
    DateTime now = rtc.now(); 
    if(now.hour() == tArray[t]){ // modo normal de operação 
        if(t > tArray[12]){t = 0;} 
        move_panel(posArray[t]); 
        t++; 
    } 
    else { 
        float accel = mAccel(); // informações de desempenho 
        float teste = mAmp(); 
        float teste1 = mVolt(); 
        float teste2 = mPot(); 
        Serial.println(teste); 
        Serial.println(teste1); 
        Serial.println(teste2); 
        Serial.print("POSICAO: "); 
        Serial.print(accel); 
        Serial.print(" | tensao: "); 
        Serial.print(teste1); 
        Serial.print("V"); 
        Serial.print(" | Potencia: "); 
        Serial.print(teste2); 
        Serial.println("W"); 
    } 
    delay (1000); 
}