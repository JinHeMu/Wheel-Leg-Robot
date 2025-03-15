/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  鈥斺€斺€斺€斺€斺€斺€�
 *  as + 1
 ******************************************************************************
 */
#include "QuaternionEKF.h"

QEKF_INS_t QEKF_INS;

const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {1, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 1};
float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 1e-8;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    // 鍒濆鍖栫煩闃电淮搴︿俊鎭�
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    // 濮挎€佸垵濮嬪寲
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    // 鑷畾涔夊嚱鏁板垵濮嬪寲,鐢ㄤ簬鎵╁睍鎴栧鍔爇f鐨勫熀纭€鍔熻兘
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 璁惧畾鏍囧織浣�,鐢ㄨ嚜瀹氬嚱鏁版浛鎹f鏍囧噯姝ラ涓殑SetK(璁＄畻澧炵泭)浠ュ強xhatupdate(鍚庨獙浼拌/铻嶅悎)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s虏
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,鐢ㄤ簬鏇存柊宸ヤ綔鐐瑰鐨勭姸鎬佽浆绉籉鐭╅樀
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    if (!QEKF_INS.Initialized)
    {
        IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 姝ら儴鍒嗚瀹氱姸鎬佽浆绉荤煩闃礔鐨勫乏涓婅閮ㄥ垎 4x4瀛愮煩闃�,鍗�0.5(Ohm-Ohm^bias)*deltaT,鍙充笅瑙掓湁涓€涓�2x2鍗曚綅闃靛凡缁忓垵濮嬪寲濂戒簡
    // 娉ㄦ剰鍦╬redict姝鐨勫彸涓婅鏄�4x2鐨勯浂鐭╅樀,鍥犳姣忔predict鐨勬椂鍊欓兘浼氳皟鐢╩emcpy鐢ㄥ崟浣嶉樀瑕嗙洊鍓嶄竴杞嚎鎬у寲鍚庣殑鐭╅樀
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,鍔犻€熷害杩囦竴涓嬩綆閫氭护娉㈠钩婊戞暟鎹�,闄嶄綆鎾炲嚮鍜屽紓甯哥殑褰卞搷
    if (QEKF_INS.UpdateCount == 0) // 濡傛灉鏄涓€娆¤繘鍏�,闇€瑕佸垵濮嬪寲浣庨€氭护娉�
    {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    // set z,鍗曚綅鍖栭噸鍔涘姞閫熷害鍚戦噺
    accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm; // 鐢ㄥ姞閫熷害鍚戦噺鏇存柊閲忔祴鍊�
    }

    // get body state
    QEKF_INS.gyro_norm = 1.0f / invSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                        QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                        QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0f / accelInvNorm;

    // 濡傛灉瑙掗€熷害灏忎簬闃堝€间笖鍔犻€熷害澶勪簬璁惧畾鑼冨洿鍐�,璁や负杩愬姩绋冲畾,鍔犻€熷害鍙互鐢ㄤ簬淇瑙掗€熷害
    // 绋嶅悗鍦ㄦ渶鍚庣殑濮挎€佹洿鏂伴儴鍒嗕細鍒╃敤StableFlag鏉ョ‘瀹�
    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f && QEKF_INS.accl_norm < 9.8f + 0.5f)
    {
        QEKF_INS.StableFlag = 1;
    }
    else
    {
        QEKF_INS.StableFlag = 0;
    }

    // set Q R,杩囩▼鍣０鍜岃娴嬪櫔澹扮煩闃�
    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    // 璋冪敤kalman_filter.c灏佽濂界殑鍑芥暟,娉ㄦ剰鍑犱釜User_Funcx_f鐨勮皟鐢�
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 鑾峰彇铻嶅悎鍚庣殑鏁版嵁,鍖呮嫭鍥涘厓鏁板拰xy闆堕鍊�
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // 澶ч儴鍒嗘椂鍊檢杞撮€氬ぉ,鏃犳硶瑙傛祴yaw鐨勬紓绉�

    // 鍒╃敤鍥涘厓鏁板弽瑙ｆ鎷夎
    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f) * 57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f) * 57.295779513f;
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * 57.295779513f;

    // get Yaw total, yaw鏁版嵁鍙兘浼氳秴杩�360,澶勭悊涓€涓嬫柟渚垮叾浠栧姛鑳戒娇鐢�(濡傚皬闄€铻�)
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f)
    {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f)
    {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++; // 鍒濆鍖栦綆閫氭护娉㈢敤,璁℃暟娴嬭瘯鐢�
}

/**
 * @brief 鐢ㄤ簬鏇存柊绾挎€у寲鍚庣殑鐘舵€佽浆绉荤煩闃礔鍙充笂瑙掔殑涓€涓�4x2鍒嗗潡鐭╅樀,绋嶅悗鐢ㄤ簬鍗忔柟宸煩闃礟鐨勬洿鏂�;
 *        骞跺闆舵紓鐨勬柟宸繘琛岄檺鍒�,闃叉杩囧害鏀舵暃骞堕檺骞呴槻姝㈠彂鏁�
 *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;

    // fading filter,闃叉闆堕鍙傛暟杩囧害鏀舵暃
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    // 闄愬箙,闃叉鍙戞暎
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 鍦ㄥ伐浣滅偣澶勮绠楄娴嬪嚱鏁癶(x)鐨凧acobi鐭╅樀H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 鍒╃敤瑙傛祴鍊煎拰鍏堥獙浼拌寰楀埌鏈€浼樼殑鍚庨獙浼拌
 *        鍔犲叆浜嗗崱鏂规楠屼互鍒ゆ柇铻嶅悎鍔犻€熷害鐨勬潯浠舵槸鍚︽弧瓒�
 *        鍚屾椂寮曞叆鍙戞暎淇濇姢淇濊瘉鎭跺姡宸ュ喌涓嬬殑蹇呰閲忔祴鏇存柊
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H路P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H路P'(k)路HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H路P'(k)路HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 璁＄畻棰勬祴寰楀埌鐨勯噸鍔涘姞閫熷害鏂瑰悜(閫氳繃濮挎€佽幏鍙栫殑)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 璁＄畻棰勬祴鍊煎拰鍚勪釜杞寸殑鏂瑰悜浣欏鸡
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 鍒╃敤鍔犻€熷害璁℃暟鎹慨姝�
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,鍗℃柟妫€楠�
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H路P'(k)路HT + R)路(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
    // rk is small,filter converged/converging
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++; // 杞戒綋闈欐鏃朵粛鏃犳硶閫氳繃鍗℃柟妫€楠�
        }
        else
        {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            // 婊ゆ尝鍣ㄥ彂鏁�
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            //  娈嬪樊鏈€氳繃鍗℃柟妫€楠� 浠呴娴�
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk瓒婂皬鍒欏鐩婅秺澶�,鍚﹀垯鏇寸浉淇￠娴嬪€�
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
        {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)路HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)路(z(k) - H路xhat'(k))

    // 闆舵紓淇闄愬箙,涓€鑸笉浼氭湁杩囧ぇ鐨勬紓绉�
    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 涓嶄慨姝aw杞存暟鎹�
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF瑙傛祴鐜妭,鍏跺疄灏辨槸鎶婃暟鎹鍒朵竴涓�
 *
 * @param kf kf绫诲瀷瀹氫箟
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
 * @brief 鑷畾涔�1/sqrt(x),閫熷害鏇村揩
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
