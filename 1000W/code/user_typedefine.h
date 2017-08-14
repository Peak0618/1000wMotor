#ifndef __USER_TYPEDEFINE_H__
#define __USER_TYPEDEFINE_H__
/*----------------------------- �궨�� -------------------------------------*/
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
//little-endian  (���ݵ�LSB����ڵ͵�ַ�����ݵ�MSB����ڸߵ�ַ)
//(freescale 8013,microchip 3010,Panasonic MN103SFJ9D)
//-----------------------------------------------------------------------------
typedef union
{
   int64_t dlword;     //���з���˫���ַ���
   
   struct 
   {
   	  int32_t low;
      int32_t high;
   }lword;            //���з��ų��ַ���
   
   uint64_t udlword;      //���޷���˫���ַ���
   
   struct 
   {
      uint32_t low;
      uint32_t high;
   }ulword;                   //���޷��ų��ַ���
   
}dlword_type;       //����˫���ֽṹ�������
//----------------------------------
typedef union
{
   int32_t  lword;               //���з��ų��ַ���
   struct 
   {
      int16_t low;
      int16_t high;
   }word;                    //���з����ַ���
   struct 
   {
      int8_t LL;
      int8_t LH;
      int8_t HL;
      int8_t HH;      
   }byte;                    //���з����ֽڷ���
   uint32_t ulword;      //���޷��ų��ַ���
   struct 
   {
      uint16_t low;
      uint16_t high;
   }uword;                   //���޷����ַ���
   struct 
   {
      uint8_t LL;
      uint8_t LH;
      uint8_t HL;
      uint8_t HH;     
   }ubyte;                   //���޷����ֽڷ���
}lword_type;       //���峤�ֽṹ�������
//-------------------------------------------
typedef union
{   
   int16_t word;              //���з����ַ���
   struct 
   {
       int8_t  low;
       int8_t  high;
   }byte;                  //���з����ֽڷ���
   
   uint16_t uword;    //���޷����ַ���
   struct 
   {
       uint8_t low;
       uint8_t high;
   }ubyte;                  //���޷����ֽڷ���
}word_type;       //�����ֽṹ�������

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

  //����һ������������
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
//big-endian (���ݵ�LSB����ڸߵ�ַ�����ݵ�MSB����ڵ͵�ַ)
//-----------------------------------------------
/*
//-------------------------------------------------------
typedef union
{
   int32_t  lword;               //���з��ų��ַ���
   struct 
   {
      int16_t high;
      int16_t low;
   }word;                    //���з����ַ���
   struct 
   {
      int8_t HH;
      int8_t HL;
      int8_t LH;
      int8_t LL;      
   }byte;                    //���з����ֽڷ���
   uint32_t ulword;      //���޷��ų��ַ���
   struct 
   {
      uint16_t high;
      uint16_t low;
   }uword;                   //���޷����ַ���
   struct 
   {
      uint8_t HH;
      uint8_t HL;
      uint8_t LH;
      uint8_t LL;     
   }ubyte;                   //���޷����ֽڷ���
}lword_type;       //���峤�ֽṹ�������
//-------------------------------------------
typedef union
{   
   int16_t word;              //���з����ַ���
   struct 
   {
       int8_t  high;
       int8_t  low;
   }byte;                  //���з����ֽڷ���
   
   uint16_t uword;    //���޷����ַ���
   struct 
   {
       uint8_t high;
       uint8_t low;
   }ubyte;                  //���޷����ֽڷ���
}word_type;       //�����ֽṹ�������
*/
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
#endif   //__USER_TYPEDEFINE_H__
