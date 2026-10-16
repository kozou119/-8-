void linetrace_bang_bang()
{
  static float lightMin =37; // 各自で設定
  static float lightMax = 255; // 各自で設定 （わざとエラーが出るようにしてある）
  static float speed = 100; // パラメーター
  static float Kp = 2.0; // パラメーター
  float lightNow;
  float speedDiff;

  lightNow = (red_G + green_G + blue_G) / 3.0;
  if (lightNow < (lightMin + lightMax) / 2.0) // 右回転
    speedDiff = -Kp * speed;
  else // 左回転
    speedDiff = Kp * speed;

  motorL_G = speed - speedDiff;
  motorR_G = speed + speedDiff;
}


void linetrace_P()
{
  static float lightMin = 37;    
  static float lightMax = 255;   
  static float speed = 100;
  static float Kp = 1.75;

  float lightNow;
  float target;
  float speedDiff;

  // RGB値の平均
  lightNow = (red_G + green_G + blue_G) / 3.0;

  // 白と黒の中間値を目標値にする
  target = (lightMin + lightMax) / 2.0;

  // 目標値からのずれに比例して回転力を決める
  speedDiff = Kp * speed * (lightNow - target)
              / (lightMax - lightMin) * 2.0;

  // 左右モーターの速度を決定
  motorL_G = speed - speedDiff;
  motorR_G = speed + speedDiff;
}

void task_A_simplified()
{
   switch (mode_G)
  {
    case 0:
      mode_G = 1;
      break;

    case 1:
      // 青線に沿ってライントレース
      linetrace_P();
      // 赤線を検出
      if (identify_color(255, 37, 37))
      {
        mode_G = 2;
      }
      break;

    case 2:
      // 赤線上で停止

      motors.setSpeeds(0, 0);
      // 1秒間停止
      delay(1000);

      // 赤線から離れるためmode 3へ
      mode_G = 3;
      break;

    case 3:
      // 赤線の右縁に沿ってライントレース
      linetrace_P();

      // 青線を検出したら通常のライントレースへ
      if (identify_color(0, 0, 255))
      {
        mode_G = 1;
      }
      break;
  }
}

void task_A()
{
  static unsigned long startTime; // static変数，時間計測ははunsigned long
  // この変数は1秒停止の開始時間を覚えておくために使用

  switch (mode_G) 
  {
    case 0:
      mode_G = 1;
      break;  // break文を忘れない（忘れるとその下も実行される）

    case 1:
      linetrace_P(); // ライントレース（各自で作成）
      if (identify_color( 255, 0, 0 )) 
      { 
        // 赤かどうか　（引数の値は各自で設定）
        mode_G = 2;
      }
      break;
     
  }
}

void task_B()
{
    // TODO (各自で作成)
}


int identify_color(int red, int green, int blue)
{
  float d2;
  float d2_max = 100; // パラメーター（適宜調整）

  d2 = pow(red - red_G, 2) + 
       pow(green - green_G, 2) + 
       pow(blue - blue_G, 2);
       
  if ( d2 < d2_max * d2_max )
    return 1;
  else
    return 0;
}

int maintainState(unsigned long period)
{
  static int flagStart = 0; // 0:待ち状態，1:現在計測中
  static unsigned long startTime = 0;

  if (flagStart == 0) 
  {
    startTime = timeNow_G; // 計測を開始したtimeNow_Gの値を覚えておく
    flagStart = 1; // 現在計測中にしておく
  }

  if (timeNow_G - startTime > period) 
  { 
    // 計測開始からの経過時間が指定時間を越えた
    flagStart = 0; // 待ち状態に戻しておく
    startTime = 0; // なくても良いが，形式的に初期化
    return 1;
  }
  else
    return 0;
}
