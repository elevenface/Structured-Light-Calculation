#include "StaticParameters.h"

// 投影仪的分辨率
const int PROJECTOR_RESLINE = 1280;
const int PROJECTOR_RESROW = 800;

// 相机的分辨率
const int CAMERA_RESLINE = 1280;
const int CAMERA_RESROW = 1024;

// 计算机偏移量
const int PC_BIASLINE = 1366;
const int PC_BIASROW = 0;

// 格雷码和PhaseShifting的数目
const int GRAY_V_NUMDIGIT = 6;
const int GRAY_H_NUMDIGIT = 5;
const int PHASE_NUMDIGIT = 4;

// Visualization 相关参数
const int SHOW_PICTURE_TIME = 500;
const bool VISUAL_DEBUG = false;

// 棋盘格相关参数
const int CHESS_FRAME_NUMBER = 15;
const int CHESS_LINE = 13;
const int CHESS_ROW = 8;

// 数据读取相关
const string DATA_PATH = "D:\\Structured_Light_Data\\";
const int DYNAFRAME_MAXNUM = 100;

// 数据输出相关
const int FOV_MIN_DISTANCE = 10;
const int FOV_MAX_DISTANCE = 100;

// 计算相关
const int RECO_WINDOW_SIZE = 21;