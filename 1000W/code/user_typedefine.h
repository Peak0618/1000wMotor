#ifndef __USER_TYPEDEFINE_H__
#define __USER_TYPEDEFINE_H__
/*----------------------------- 宏定义 -------------------------------------*/
typedef           char        char_t;   

typedef signed    char        int8_t;
typedef signed    short       int16_t;
typedef signed    long        int32_t;

typedef signed    long long   int64_t;


typedef unsigned  char        uint8_t;
typedef unsigned  short       uint16_t;
typedef unsigned  long        uint32_t;

typedef unsigned  long long   uint64_t;

typedef       float           float32_t;
typedef       double          float64_t;
//-----------------------------------------------------------------------------
//little-endian  (数据的LSB存放在低地址，数据的MSB存放在高地址)
//(freescale 8013,microchip 3010,Panasonic MN103SFJ9D)
//-----------------------------------------------------------------------------
typedef union
{
   int64_t dlword;     //按有符号双长字访问
   
   struct 
   {
   	  int32_t low;
      int32_t high;
   }lword;            //按有符号长字访问
   
   uint64_t udlword;      //按无符号双长字访问
   
   struct 
   {
      uint32_t low;
      uint32_t high;
   }ulword;                   //按无符号长字访问
   
}dlword_type;       //定义双长字结构体的类型
//----------------------------------
typedef union
{
   int32_t  lword;               //按有符号长字访问
   struct 
   {
      int16_t low;
      int16_t high;
   }word;                    //按有符号字访问
   struct 
   {
      int8_t LL;
      int8_t LH;
      int8_t HL;
      int8_t HH;      
   }byte;                    //按有符号字节访问
   uint32_t ulword;      //按无符号长字访问
   struct 
   {
      uint16_t low;
      uint16_t high;
   }uword;                   //按无符号字访问
   struct 
   {
      uint8_t LL;
      uint8_t LH;
      uint8_t HL;
      uint8_t HH;     
   }ubyte;                   //按无符号字节访问
}lword_type;       //定义长字结构体的类型
//-------------------------------------------
typedef union
{   
   int16_t word;              //按有符号字访问
   struct 
   {
       int8_t  low;
       int8_t  high;
   }byte;                  //按有符号字节访问
   
   uint16_t uword;    //按无符号字访问
   struct 
   {
       uint8_t low;
       uint8_t high;
   }ubyte;                  //按无符号字节访问
}word_type;       //定义字结构体的类型

//-------------------------------------------
typedef union 
{
        uint16_t  uword;
        struct
        {
            uint16_t   bit0:1;
            uint16_t   bit1:1;
            uint16_t   bit2:1;
            uint16_t   bit3:1;
            uint16_t   bit4:1;
            uint16_t   bit5:1;
            uint16_t   bit6:1;
            uint16_t   bit7:1;
            uint16_t   bit8:1;
            uint16_t   bit9:1;
            uint16_t   bit10:1;
            uint16_t   bit11:1;
            uint16_t   bit12:1;
            uint16_t   bit13:1;
            uint16_t   bit14:1;
            uint16_t   bit15:1;
         }bits;
} flag_type;

  //定义一个参数体类型
  typedef struct
  {
     int16_t   value;
     int16_t   max;
     int16_t   min;
     int16_t   editrate;
     int8_t    disprate;
  } para_style;
  //-----------------------------------

//-------------------------------------------------------------------------
//big-endian (数据的LSB存放在高地址，数据的MSB存放在低地址)
//-----------------------------------------------
/*
//-------------------------------------------------------
typedef union
{
   int32_t  lword;               //按有符号长字访问
   struct 
   {
      int16_t high;
      int16_t low;
   }word;                    //按有符号字访问
   struct 
   {
      int8_t HH;
      int8_t HL;
      int8_t LH;
      int8_t LL;      
   }byte;                    //按有符号字节访问
   uint32_t ulword;      //按无符号长字访问
   struct 
   {
      uint16_t high;
      uint16_t low;
   }uword;                   //按无符号字访问
   struct 
   {
      uint8_t HH;
      uint8_t HL;
      uint8_t LH;
      uint8_t LL;     
   }ubyte;                   //按无符号字节访问
}lword_type;       //定义长字结构体的类型
//-------------------------------------------
typedef union
{   
   int16_t word;              //按有符号字访问
   struct 
   {
       int8_t  high;
       int8_t  low;
   }byte;                  //按有符号字节访问
   
   uint16_t uword;    //按无符号字访问
   struct 
   {
       uint8_t high;
       uint8_t low;
   }ubyte;                  //按无符号字节访问
}word_type;       //定义字结构体的类型
*/
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
#endif   //__USER_TYPEDEFINE_H__
