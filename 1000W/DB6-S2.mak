#================================================================
# DebugFactory makefile
# 
# Author   : Panasonic��ʽ����
# 
# FileName :DB6-S2.mak
#================================================================
 
#================================================================
#       ����DebugFactory��Make��Լ��
#
# DebugFactory�У��������µĺ궨�塢���ԴӣɣģŽ���
#�ı༭��
#�벻Ҫֱ�ӱ༭makefile���ɣģ������ø��ĵ������
#�����ڱ༭�е����ݽ���������
#
#PROJECT    : Ŀ��
#SOURCEFILE : Դ�����ļ�
#TOOLDIR    : ���湤�ߵ��ļ���
#CC         : �༭����
#ASM        : ��������
#LINK       : �������� 
#CINC       : �����ļ�·��(��������)
#AINC      : �����ļ�·��(��������)
#LIBPATH   : ���ļ�·��
#LIBFILES  : ���ļ���
#CFLAGS    : ����ѡ��
#AFLAGS    : �����ѡ��
#LFLAGS    : ������/��ѡ��
#OBJECTOUTPUTDIR : �����ļ�(*.rf)����ļ���
#
 
#================================================================
 
#================================================================
# Ŀ������
#================================================================
PROJECT	= DB6-S2.x
 
#================================================================
# �û��궨��
#================================================================
#========================DFUSRDEFMACRO_S==============================
#========================DFUSRDEFMACRO_E==============================
 
#================================================================
# Դ�����ļ�����
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
# �м��ļ�����
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
# ��������
#================================================================
CC	= CC103S.exe
ASM	= AS103S.exe
LINK	= LD103S.exe
 
#================================================================
# ��Դ����İ����ļ�·������
#================================================================
CINC	= 
 
#================================================================
# ������İ����ļ�·������
#================================================================
AINC	= 
 
#================================================================
# ���ļ�·������
#================================================================
LIBPATH	=
 
#================================================================
# ������
#================================================================
LIBFILES	= 
 
#================================================================
# ������ѡ������
#================================================================
CFLAGS	=-e  -mdivs -mmulq -g -flong-bfield -fchar-bfield -O -fenable-longlong -fc9x-comment -fenable-asm -c -L -w2702 $(CINC)
 
#================================================================
# ����ѡ������
#================================================================
 
#================================================================
# �����ѡ������
#================================================================
AFLAGS	= -e  -g -l -W2702 $(AINC)
CAFLAGS	= -e -g -l -W2702 $(AINC) -O
 
#================================================================
# ������ѡ������
#================================================================
LFLAGS	= -e  -m -g
 
#================================================================
# �����ļ�(*.RF)����ļ�������
#================================================================
OBJECTOUTPUTDIR	= output\\
 
#================================================================
# Ŀ�꣨EX��
#================================================================
$(PROJECT): $(OBJFILES)
 
#================================================================
# ����ǰ���û����崦��
#================================================================
#========================DFUSRBDEF_S==============================
#========================DFUSRBDEF_E==============================
 
#================================================================
# Ŀ�꣨EX��
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
# �ļ��任
#================================================================
	XCV103.EXE -S3 .\$(PROJECT)
	CRCCalc.exe "DB6-S2.mot" /TMOT /CCRC16-LSB "/FCRC.txt" /I0xFFFF /X0x0
 
#================================================================
# Make����û����崦��
#================================================================
#========================DFUSRADEF_S==============================
#========================DFUSRADEF_E==============================
 
#================================================================
# ִ������
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
