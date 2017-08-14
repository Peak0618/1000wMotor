#================================================================
# DebugFactory makefile
# 
# Author   : Panasonic株式会社
# 
# FileName :DB6-S2.mak
#================================================================
 
#================================================================
#       关于DebugFactory的Make规约，
#
# DebugFactory中，对于以下的宏定义、可以从ＩＤＥ进行
#的编辑。
#请不要直接编辑makefile。ＩＤＥ中设置更改的情况下
#，正在编辑中的内容将被废弃。
#
#PROJECT    : 目标
#SOURCEFILE : 源代码文件
#TOOLDIR    : 保存工具的文件夹
#CC         : 编辑器名
#ASM        : 汇编程序名
#LINK       : 链接器名 
#CINC       : 包含文件路径(编译器用)
#AINC      : 包含文件路径(汇编程序用)
#LIBPATH   : 库文件路径
#LIBFILES  : 库文件名
#CFLAGS    : 编译选项
#AFLAGS    : 汇编器选项
#LFLAGS    : 链接器/库选项
#OBJECTOUTPUTDIR : 对象文件(*.rf)输出文件夹
#
 
#================================================================
 
#================================================================
# 目标设置
#================================================================
PROJECT	= DB6-S2.x
 
#================================================================
# 用户宏定义
#================================================================
#========================DFUSRDEFMACRO_S==============================
#========================DFUSRDEFMACRO_E==============================
 
#================================================================
# 源代码文件设置
#================================================================
SOURCEFILE	= code\startup.as \
		code\vector.c \
		code\mainprog.c \
		code\SIN_TABLE.c \
		code\AD_Convert.c \
		code\EEPROM_PARA.c \
		code\EEPROM_RW.c \
		code\Hallsignal_deal.c \
		code\Host_comm.c \
		code\panel_comm.c \
		code\protect_deal.c \
		code\PWM_CTRL.c \
		code\SWI2C_EEPROM_Drive.c \
		code\user_math.c \
		code\Tsink_tab.c \
 
#================================================================
# 中间文件设置
#================================================================
OBJFILES	= output\startup.ro \
		output\vector.ro \
		output\mainprog.ro \
		output\SIN_TABLE.ro \
		output\AD_Convert.ro \
		output\EEPROM_PARA.ro \
		output\EEPROM_RW.ro \
		output\Hallsignal_deal.ro \
		output\Host_comm.ro \
		output\panel_comm.ro \
		output\protect_deal.ro \
		output\PWM_CTRL.ro \
		output\SWI2C_EEPROM_Drive.ro \
		output\user_math.ro \
		output\Tsink_tab.ro \
 
#================================================================
# 工具设置
#================================================================
CC	= CC103S.exe
ASM	= AS103S.exe
LINK	= LD103S.exe
 
#================================================================
# Ｃ源代码的包含文件路径设置
#================================================================
CINC	= 
 
#================================================================
# 汇编程序的包含文件路径设置
#================================================================
AINC	= 
 
#================================================================
# 库文件路径设置
#================================================================
LIBPATH	=
 
#================================================================
# 库设置
#================================================================
LIBFILES	= 
 
#================================================================
# 编译器选项设置
#================================================================
CFLAGS	=-e  -mdivs -mmulq -g -flong-bfield -fchar-bfield -O -fenable-longlong -fc9x-comment -fenable-asm -c -L -w2702 $(CINC)
 
#================================================================
# 本地选项设置
#================================================================
 
#================================================================
# 汇编器选项设置
#================================================================
AFLAGS	= -e  -g -l -W2702 $(AINC)
CAFLAGS	= -e -g -l -W2702 $(AINC) -O
 
#================================================================
# 链接器选项设置
#================================================================
LFLAGS	= -e  -m -g
 
#================================================================
# 对象文件(*.RF)输出文件夹设置
#================================================================
OBJECTOUTPUTDIR	= output\\
 
#================================================================
# 目标（EX）
#================================================================
$(PROJECT): $(OBJFILES)
 
#================================================================
# 链接前的用户定义处理
#================================================================
#========================DFUSRBDEF_S==============================
#========================DFUSRBDEF_E==============================
 
#================================================================
# 目标（EX）
#================================================================
	$(LINK) @&&|
		-o.\$(PROJECT)
		$(LFLAGS)
		$(LIBPATH)
		-T@CODE=40000000
		-T@DATA=00000000
		$(OBJECTOUTPUTDIR)startup.ro
		$(OBJECTOUTPUTDIR)vector.ro
		$(OBJECTOUTPUTDIR)mainprog.ro
		$(OBJECTOUTPUTDIR)SIN_TABLE.ro
		$(OBJECTOUTPUTDIR)PWM_CTRL.ro
		$(OBJECTOUTPUTDIR)AD_Convert.ro
		$(OBJECTOUTPUTDIR)Hallsignal_deal.ro
		$(OBJECTOUTPUTDIR)EEPROM_PARA.ro
		$(OBJECTOUTPUTDIR)EEPROM_RW.ro
		$(OBJECTOUTPUTDIR)panel_comm.ro
		$(OBJECTOUTPUTDIR)protect_deal.ro
		$(OBJECTOUTPUTDIR)SWI2C_EEPROM_Drive.ro
		$(OBJECTOUTPUTDIR)user_math.ro
		$(OBJECTOUTPUTDIR)Tsink_tab.ro
		$(OBJECTOUTPUTDIR)Host_comm.ro
		$(LIBFILES)
|
 
#================================================================
# 文件变换
#================================================================
	XCV103.EXE -S3 .\$(PROJECT)
	CRCCalc.exe "DB6-S2.mot" /TMOT /CCRC16-LSB "/FCRC.txt" /I0xFFFF /X0x0
 
#================================================================
# Make后的用户定义处理
#================================================================
#========================DFUSRADEF_S==============================
#========================DFUSRADEF_E==============================
 
#================================================================
# 执行命令
#================================================================
$(OBJECTOUTPUTDIR)startup.ro: code\startup.as
	$(ASM) $(AFLAGS) -o $(OBJECTOUTPUTDIR)$(@F) code\startup.as
	MOVE /Y startup.lst $(OBJECTOUTPUTDIR)startup.lst
$(OBJECTOUTPUTDIR)vector.ro: code\vector.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)vector.ro -c code\vector.c
$(OBJECTOUTPUTDIR)mainprog.ro: code\mainprog.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)mainprog.ro -c code\mainprog.c
$(OBJECTOUTPUTDIR)SIN_TABLE.ro: code\SIN_TABLE.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)SIN_TABLE.ro -c code\SIN_TABLE.c
$(OBJECTOUTPUTDIR)AD_Convert.ro: code\AD_Convert.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)AD_Convert.ro -c code\AD_Convert.c
$(OBJECTOUTPUTDIR)EEPROM_PARA.ro: code\EEPROM_PARA.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)EEPROM_PARA.ro -c code\EEPROM_PARA.c
$(OBJECTOUTPUTDIR)EEPROM_RW.ro: code\EEPROM_RW.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)EEPROM_RW.ro -c code\EEPROM_RW.c
$(OBJECTOUTPUTDIR)Hallsignal_deal.ro: code\Hallsignal_deal.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)Hallsignal_deal.ro -c code\Hallsignal_deal.c
$(OBJECTOUTPUTDIR)Host_comm.ro: code\Host_comm.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)Host_comm.ro -c code\Host_comm.c
$(OBJECTOUTPUTDIR)panel_comm.ro: code\panel_comm.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)panel_comm.ro -c code\panel_comm.c
$(OBJECTOUTPUTDIR)protect_deal.ro: code\protect_deal.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)protect_deal.ro -c code\protect_deal.c
$(OBJECTOUTPUTDIR)PWM_CTRL.ro: code\PWM_CTRL.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)PWM_CTRL.ro -c code\PWM_CTRL.c
$(OBJECTOUTPUTDIR)SWI2C_EEPROM_Drive.ro: code\SWI2C_EEPROM_Drive.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)SWI2C_EEPROM_Drive.ro -c code\SWI2C_EEPROM_Drive.c
$(OBJECTOUTPUTDIR)user_math.ro: code\user_math.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)user_math.ro -c code\user_math.c
$(OBJECTOUTPUTDIR)Tsink_tab.ro: code\Tsink_tab.c
	$(CC) $(CFLAGS) -o $(OBJECTOUTPUTDIR)Tsink_tab.ro -c code\Tsink_tab.c
