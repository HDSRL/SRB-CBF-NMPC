/*****************************************************************
Randy Fawcett, HDSRL 
*****************************************************************/

struct buttons {
    int R1          = 0;
    int L1          = 0;
    int start       = 0;
    int select      = 0;
    int R2          = 0;
    int L2          = 0;
    int F1          = 0;
    int F2          = 0;
    int A           = 0;
    int B           = 0;
    int X           = 0;
    int Y           = 0;
    int up          = 0;
    int right       = 0;
    int down        = 0;
    int left        = 0;
    int32_t RX      = 0;
    int32_t RY      = 0;
    int32_t LX      = 0;
    int32_t LY      = 0;
};


/*// 16B*/
/*typedef union {*/
/*    struct {*/
/*        uint8_t R1          :1;*/
/*        uint8_t L1          :1;*/
/*        uint8_t start       :1;*/
/*        uint8_t select      :1;*/
/*        uint8_t R2          :1;*/
/*        uint8_t L2          :1;*/
/*        uint8_t F1          :1;*/
/*        uint8_t F2          :1;*/
/*        uint8_t A           :1;*/
/*        uint8_t B           :1;*/
/*        uint8_t X           :1;*/
/*        uint8_t Y           :1;*/
/*        uint8_t up          :1;*/
/*        uint8_t right       :1;*/
/*        uint8_t down        :1;*/
/*        uint8_t left        :1;*/
/*    } components;*/
/*    uint16_t value;*/
/*} xKeySwitchUnion;*/

/*// 40 Byte (now used 24B)*/
/*typedef struct {*/
/*    uint8_t head[2];*/
/*    xKeySwitchUnion btn;*/
/*    float lx;*/
/*    float rx;*/
/*    float ry;*/
/*    float L2;*/
/*    float ly;*/

/*} xRockerBtnDataStruct;*/
