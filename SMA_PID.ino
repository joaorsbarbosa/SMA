#include <Servo.h>

//Array dos pinos ADC usados para ler a  tensão nos LDR's
const int analog_pin_sensor [4] = {A0, A1, A2, A3};


/*arrays e matrix pertencentes à moving average
 * A moving average consistem é uma média que se "move"
 * à media que se faz medidas nos LDR's. Isto é, em vez de se
 * obter 10 valores, realizar-se a media deles, e usar essa média para as operações,  
 * sendo necessário obter 10 novos valores para repetir o processo.
 * 
 * Isto é uma forma bastante lenta de se realizar uma média, abrandando bastante
 * a velocidade do projeto. Para evitar este problema a moving average tira os 10 valores,
 * realiza a média aritmetica, e no proximo loop, retira o valor mais "antigo" do conjunto 
 * e adiciona um novo valor. Desta forma apenas se espera o tempo de obtenção de medida em vez de 10.
 */
int current_read [4] = {0, 0, 0, 0};
float average_sensor[4] = {0, 0, 0, 0} ;
float total[4] = {0, 0, 0, 0};
const int readings = 50; //numero de leituras na moving average. quanto maior, mais smooth, mas mais lento o arduino fica e rapidamente fica sem memoria
float sensor_matrix [4][readings];
int j = 0;


/*faz-se ainda a soma de ambos os sensores do mesmo lado
 * É mais facil para mover o projeto se decompormos em 2 direções para cada servo
 * em vez de termos 4 "vetores" diferentes" 
 * 
 * I.E: para medidas na horizontal usa-se a info de ambos os LDR's no lado 
 * esquerdo, topo e baixo. O mesmo se aplica nas medidas verticais.
 */
float average_left = 0;
float average_right = 0;
float average_up = 0;
float average_down = 0;

/* Para evitar que a esquerda e a direta fiquem "trocadas" quando os painis
 *  ultrapassam os 90º, faz-se a inversão dos dados, podendo assim o tracking
 *  ser correto. 
 */
float average_left_inv = 0;
float average_right_inv = 0;


//posição inicial dos servos. 
float posH = 60;
float posV = 120;


/* Visto que os paineis tem que fazer tracking em dois eixos
 *  utiliza-se um controlo PID para cada eixo
 */
 
//---Vertical---//
float SV_V = 0;
float error_V = 0;
float Delta_t_V = 0;
float PID_V = 0;
float Derror_V = 0;
float previous_error_V = 0;
float current_time_V = 0;
float past_time_V = 0;
float P_V = 0, I_V = 0, D_V = 0;
//---------//

//---Horizontal---//
float SV_H = 0;
float error_H = 0;
float Delta_t_H = 0;
float PID_H = 0;
float Derror_H = 0;
float previous_error_H = 0;
float current_time_H = 0;
float past_time_H = 0;
float P_H = 0, I_H = 0, D_H = 0;
//---------//

//Constantes de tunning
/*Com estas obti a menor oscilação que consegui
   Parece funcionar melhor com valores baixos na P, altos na D
   e sem a parte integral no controlo. Quando colocava algum
   valor aqui o sistema começava a oscilar bastante outra vez
*/
float Kp_H = 0.3, Ki_H = 0, Kd_H = 40;
float Kp_V = 0.15, Ki_V = 0, Kd_V = 90;



//quantas medidas faz inicialmente para encher os arrays e estabilizar as médias 
int first_measures = 300;
int x = 0; // var usada no while loop


//codigo necessário para os servos
Servo S_horizontal;
Servo S_vertical;

/*-------------------------------------------------------------------------*/
void setup() {

  //para encher com 0's a matrix da moving average
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < readings; j++) {
      sensor_matrix[i][j] = 0;
    }
  }

  S_horizontal.attach(9);
  S_vertical.attach(10);

  //PID
  past_time_V = millis();
  past_time_H = millis();


  //corre a moving average para quando chegar ao PID já ter valores estaveis
  while ( x < first_measures) {

    for (int i = 0; i <= 3; i++) {
      j = current_read [i];
      if (j >= readings - 1) {
        for (int k = 0; k <= 3; k++) {
          current_read[k] = 0;
        }
        j = 0;
      }
      total[i] = total[i] - sensor_matrix [i][j];
      sensor_matrix [i][j] = analogRead(analog_pin_sensor[i]);
      total[i] = total [i] + sensor_matrix [i][j];
      average_sensor[i] = total[i] / readings;
      current_read [i] = j + 1;
      x++;

    }
  }
}

void loop() {

  /* moving average
   *  Como temos 4 sensores, e cada um com X medidas, criou-se uma matriz para o armazenamento 
   *  dos dados, em vez de quadruplicar todas as variaveis necessárias. 
   *  as variaveis i,j, e k sao apenas para controlo dos loops.
   */
  for (int i = 0; i <= 3; i++) {
    j = current_read [i];
    if (j >= readings - 1) {
      for (int k = 0; k <= 3; k++) {
        current_read[k] = 0;
      }
      j = 0;
    }
    total[i] = total[i] - sensor_matrix [i][j];
    sensor_matrix [i][j] = analogRead(analog_pin_sensor[i]);
    total[i] = total [i] + sensor_matrix [i][j];
    average_sensor[i] = total[i] / readings;
    current_read [i] = j + 1;
  }

  //faz as medias usando as 4 sensores de cada "lado"
  //vertical e horizontal

  average_left = (average_sensor[2] + average_sensor[0]) / 2;
  average_right = (average_sensor[1] + average_sensor[3]) / 2;
  average_up = (average_sensor[2] + average_sensor[3]) / 2;
  average_down = (average_sensor[1] + average_sensor[0]) / 2;

  if (posV <= 105) { // 105º fica perpendicular com a base devido à rotação no servo quando se montou a estrutura
    average_left_inv = average_left;
    average_right_inv = average_right;
    average_left = average_right;
    average_right = average_left_inv;
  }

  current_time_V = millis();

  /***********************PID VERTICAL **********************************************/
  Delta_t_V = current_time_V - past_time_V;
  past_time_V = current_time_V;

  error_V = SV_V - (average_down - average_up);

  Derror_V = error_V - previous_error_V;
  previous_error_V = error_V;

  P_V = Kp_V * error_V;

  I_V = I_V + Ki_V * error_V * Delta_t_V;
  I_V = constrain(I_V, -55, 55);

  D_V = Kd_V * Derror_V / Delta_t_V;
  PID_V = P_V + I_V + D_V;

//  ...........................................................................
  /*O algoritmo PID "puro" revelou-se nao ser adequado para o controlo direto dos servos 
   * neste trabalho visto não haver um set-point
   * concentro, além de que, mesmo tentando calibrar o Kp, Ki e Kd, o movimento era bastante errático. 
   * Isto resultava nos paineis a oscilar severamente sem nunca atingir equilibrio
   * Para corrigir esse problema, e de forma a reduzir a velocidade de movimento dos paineis.
   * É necessária uma velocidade reduzida nos movimentos devido à inercia da estrutura. Quando os servos
   * se movimentavam mais rapidamente, a estrutura acabava por ultrapassar o ponto de equilibrio. Para compensar 
   * isto os servos moviam mais uma vez na direção oposta, mas volta a acontecer "overshoot". Isto repetia-se 
   */
   
  if (PID_V > 0.0 && abs(PID_V) > 25) { //para evitar oscilações quando o PID se encontrava proximo de 0, apenas se o deixava controlar
    //o sistema acima de certo valor
    posV = constrain(posV + 0.2, 44, 169); //a velocidade é controlada pelo valor somado à var PosV/H
  }//devido à estrutura fisica, o movimento do servo vertical teve que ser limitado
  else if (PID_V < 0.0 && abs(PID_V) > 25) {
    posV = constrain(posV - 0.2, 44, 169);
  }
  else {
    posV = posV;
  }

  /*------------------------------------------------------------------------------*/

  //na secção horizontal, aplica-se o mesmo que foi explicado na vertical

  /***********************PID HORIZONTAL********************************************/
  current_time_H = millis();

  Delta_t_H = current_time_H - past_time_H;
  past_time_H = current_time_H;

  error_H = SV_H - (average_left - average_right);

  Derror_H = error_H - previous_error_H;
  previous_error_H = error_H;

  P_H = Kp_H * error_H;

  I_H = I_H + Ki_H * error_H * Delta_t_H;
  I_H = constrain(I_H, -55, 55);

  D_H = Kd_H * Derror_H / Delta_t_H;
  PID_H = P_H + I_H + D_H;

  if (PID_H > 0.0 && abs(PID_H) > 30) {
    posH = constrain(posH + 0.2, 0, 180);
  }
  else if (PID_H < 0.0 && abs(PID_H) > 30) {
    posH = constrain(posH - 0.2, 0, 180);
  }
  else {
    posH = posH;
  }
/*NOTA: existe sempre alguma oscilação na parte horizontal
 * Isto deve-se simplesmente à inercia mais uma vez
 * O arduino diz ao servo para se manter num angulo especifico 
 * mas o servo nunca consegue ficar exato nesse angulo. Como este tenta sempre manter o angulo que lhe 
 * foi instruido, ele tenta corrigir. Isso resulta em oscilações. No entanto, estas não são responsaveis
 * por qualquer algoritmo de controlo
 */

  /*------------------------------------------------------------------------------*/
  //Envia-se as instruçõesp para os servos
  S_horizontal.write(posH);
  S_vertical.write(posV); 


  delay(8);
/*O delay serve apenas para dar algum tempo para a estrutura "estabilizar". Sem este o sistema reagia 
 * rápido demais, não deixando os movimentos acontecer de forma fluida e criando oscilações.
 */

}
