
;-- #include files  ------------------

;-- #define macro definition    ------------------
;--       valid only in this file
;control registers

P6OUT	 equ  0x0000A006

IVAR0   equ 0x00008000
IVAR1   equ 0x00008004
IVAR2   equ 0x00008008
IVAR3   equ 0x0000800c
IVAR4   equ 0x00008010
IVAR5   equ 0x00008014
IVAR6   equ 0x00008018

CPUM    equ 0x00008040
ROMCTR  equ 0x00008078
NMICR   equ 0x00008900
IAGR    equ 0x00008a00
CKCTR   equ 0x00008280
OSCCNT	equ 0x0000A800
OSCLOCK	equ 0x0000A804
PCNT    equ 0x0000AFF2

IRQ_DISABLE   equ 0x0000      ;IRQ disable (set mask level 0) 0x0700
RAM_SIZE      equ 0x1000      ;MASK  RAM size 4K byte
STACK_SIZE    equ 0x300       ;stack size 768 byte
START_VECTOR  equ 0x40000000
OSC_SEL       equ 0x0000      ; operation clock select : 0x0000 = Internal oscillation  0x0002 = External oscillation
PLL_SEL       equ 0x0000      ; PLL multiply select   : 0x0000= 4 multiply , 0x0001= 6multiply, 0x0002= 8 multiply
MCLK_SEL      equ 0x0003      ; MCLK frequency select : 0x0001= 1/4, 0x0002= 1/2, 0x0003= 1/1
IOCLK_SEL     equ 0x0030      ; IOCLK frequency select: 0x0010= 1/8, 0x0020= 1/4, 0x0030= 1/2
WAIT_200US    equ 200         ; Plase select 0.0002 * (oscillating frequency /2) / 5.
                              ; (WARNING :It might actually take time more than 200us by the alignment.)

;-- function prototype declaration  ------------------
;--       valid only in this file
  global  _main, _0main, _irq_vct_tbl
  global  _enable_irq, _0enable_irq
  global  _disable_irq, _0disable_irq
  global  _set_imask, _0set_imask
  global  _irq_group00
  ;将自定义的汇编语言函数声明为全局函数
  global  _dmuls_h,_0dmuls_h
  global  _dmuls_l,_0dmuls_l
  
  global  _dmulu_h,_0dmulu_h
  global  _dmulu_l,_0dmulu_l

;-- RAM/structure declaration   ------------------
;--       valid only in this file
;section
.text            SECTION "ax",PROGBITS,PUBLIC  ;text area
.rodata          SECTION "a",PROGBITS,PUBLIC ;constant area
           align 4
.MN.data         SECTION "aw",NOBITS,PUBLIC  ;initialized data area
           align 4
.MN.romdata      SECTION "a",PROGBITS,PUBLIC ;ROM data area
           align 4
.MN.romdataend   SECTION "a",NOBITS,PUBLIC ;end of ROM data area
           align 4
.bss             SECTION "aw",NOBITS,PUBLIC  ;non-initialized external data area
           align 4
.MN.bssend       SECTION "aw",NOBITS,PUBLIC  ;end of non-initialized data area
           align 4


.text  SECTION

;stack define
.bss    SECTION "aw", NOBITS,PUBLIC
  ALIGN 4

stack   ds  STACK_SIZE
stack_end

;---------------------------------------------------------------------------------------------------
;---------------------------------------------------------------------------------------------------

.text   SECTION "ax", PROGBITS,PUBLIC
    ALIGN 1
_reset              ;branch here after reset (set 0x40000000 when linking!)
    jmp _start

    offset  0x00000008      ;branch when NMI occurs (set 0x40000008 when linking!)

;---------------------------------------------------------------------------------------------------
;-  Function name   : usr_nmi, irq_lev0-6
;-  Content : interrupt vector control
;-      : 
;-
;-  argument    : none
;-      : 
;-
;-  Return value    : none
;-      : 
;-
;-  Date/Author : 
;-  Change Log  : 
;-      : 
;---------------------------------------------------------------------------------------------------
;-- NMI program ------------------------------------------------------------------------------------
usr_nmi   
    movm  [other],(sp)      ;back up of d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp  
      
    movhu (NMICR),d0        ;NMI detection
    and 0x0007,d0           ;NMI?
    beq endnmi
    mov _irq_group00,a0     ;
    calls (a0)
endnmi    
    mov 0x0000ffff,d0       ;clear ID
    movhu d0,(NMICR)        ;clear NMI factor flag
    movhu (NMICR),d0        ;read as dummy

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)
    
    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti                     ;end of NMI

;-- Lv.0 -------------------------------------------------------------------------------------------
irq_lev0            ;branch here when interruption at level 0. set this address to (IVAR0).
    movm  [other],(sp)      ;back up d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp 

    movhu (IAGR),d0     ;read group number
    and 0x007c,d0
    mov _irq_vct_tbl,a0
    mov (d0,a0),a0
    calls (a0)        ;call interrupt hundler of each group

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)

    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti

;-- Lv.1 -------------------------------------------------------------------------------------------
irq_lev1            ;branch here when interruption at level 1. set this address to (IVAR1).
    movm  [other],(sp)      ;back up d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp 

    movhu (IAGR),d0     ;read group number
    and 0x007c,d0
    mov _irq_vct_tbl,a0
    mov (d0,a0),a0
    calls (a0)        ;call interrupt hundler of each group

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)

    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti

;-- Lv.2 -------------------------------------------------------------------------------------------
irq_lev2            ;branch here when interruption at level 2. set this address to (IVAR2).
    movm  [other],(sp)      ;back up d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp 

    movhu (IAGR),d0     ;read group number
    and 0x007c,d0
    mov _irq_vct_tbl,a0
    mov (d0,a0),a0
    calls (a0)        ;call interrupt hundler of each group

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)

    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti

;-- Lv.3 -------------------------------------------------------------------------------------------
irq_lev3            ;branch here when interruption at level 3. set this address to (IVAR3).
    movm  [other],(sp)      ;back up d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp 

    movhu (IAGR),d0     ;read group number
    and 0x007c,d0
    mov _irq_vct_tbl,a0
    mov (d0,a0),a0
    calls (a0)        ;call interrupt hundler of each group

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)

    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti

;-- Lv.4 -------------------------------------------------------------------------------------------
irq_lev4            ;branch here when interruption at level 4. set this address to (IVAR4).
    movm  [other],(sp)      ;back up d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp 

    movhu (IAGR),d0     ;read group number
    and 0x007c,d0
    mov _irq_vct_tbl,a0
    mov (d0,a0),a0
    calls (a0)        ;call interrupt hundler of each group

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)

    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti

;-- Lv.5 -------------------------------------------------------------------------------------------
irq_lev5            ;branch here when interruption at level 5. set this address to (IVAR5).
    movm  [other],(sp)      ;back up d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp 

    movhu (IAGR),d0     ;read group number
    and 0x007c,d0
    mov _irq_vct_tbl,a0
    mov (d0,a0),a0
    calls (a0)        ;call interrupt hundler of each group

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)

    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti

;-- Lv.6 -------------------------------------------------------------------------------------------
irq_lev6            ;branch here when interruption at level 6. set this address to (IVAR6).
    movm  [other],(sp)      ;back up d0,d1,a0,a1,mdr,lir,lar

    udf15 d0,d0             ;mdrq->d0 (udf15 = getx)  
    mov d0,(sp)             ;d0 -> stack
    add -4,sp               ;sp - 4 -> sp 

    movhu (IAGR),d0     ;read group number
    and 0x007c,d0
    mov _irq_vct_tbl,a0
    mov (d0,a0),a0
    calls (a0)        ;call interrupt hundler of each group

    add 4,sp                ;sp + 4 -> sp
    mov (sp),d0             ;stack -> d0
    udf20 d0,d0             ;d0 -> mdrq  (udf20 = putx)

    movm  (sp),[other]      ;repair d0,d1,a0,a1,mdr,lir,lar
    rti

;---------------------------------------------------------------------------------------------------
;-  Function name : enable_irq, disable_irq, set_imask
;-  Content     : PSW control
;-          : If you change LV or IE flags of ICR register, make interrupts enabled after finishing the change of flags.
;-          : In manual, Wait exists to change value certainly by synchronizing store buffer.
;-          : But value changes certainly without synchronization because of enable() task
;-
;-  Argument    : none
;-          : 
;-
;-  Return value  : none
;-          : 
;-
;-  Date/Author   : 
;-  Change Log    : 
;-      : 
;---------------------------------------------------------------------------------------------------
_enable_irq             ;PSW access
_0enable_irq  funcinfo _enable_irq,0,[]
    or  0x0800,psw      ;enable all interrupt
    nop                 ;nop * 2 wait for loading to psw
    nop
    ret

_disable_irq            ;PSW access
_0disable_irq   funcinfo _disable_irq,0,[]
    and 0xF7FF,psw      ;disable interrupt
    nop                 ;nop * 2 wait for loading to psw
    nop
    ret

;set interrupt level to register D0
_set_imask            ;PSW access
_0set_imask funcinfo _set_imask,0,[]
    mov psw,d1
    and 0xf8ff,d1
    and 0x0007,d0
    asl 8,d0          ;bp10-bp8 = IM
    or  d0,d1
    mov d1,psw
    nop
    nop
    ret
;---------------------------------------------------------------------------------------------------
;自定义的汇编语句的函数
;-----------------------------------------------------------
;有符号的32bit*32bit得到higher 32bit的程序
_dmuls_h
_0dmuls_h   funcinfo _dmuls_h,0,[]
   udf00 d1,d0   ; d0 * d1 -> mdr(higher 32bit), d1(lower 32bit)
   udf15 d0,d0   ; mdrq -> d0
   rts
;-----------------------------------------------------------
;有符号的32bit*32bit得到lower 32bit的程序   
_dmuls_l
_0dmuls_l   funcinfo _dmuls_l,0,[]
   udf00 d1,d0   ; d0 * d1 -> mdr(higher 32bit), d0(lower 32bit)
   rts
;-----------------------------------------------------------
;无符号的32bit*32bit得到higher 32bit的程序
_dmulu_h
_0dmulu_h   funcinfo _dmulu_h,0,[]
   udf01 d1,d0   ; d0 * d1 -> mdr(higher 32bit), d1(lower 32bit)
   udf15 d0,d0   ; mdrq -> d0
   rts
;-----------------------------------------------------------
;无符号的32bit*32bit得到lower 32bit的程序   
_dmulu_l
_0dmulu_l   funcinfo _dmulu_l,0,[]
   udf01 d1,d0   ; d0 * d1 -> mdr(higher 32bit), d0(lower 32bit)
   rts

;---------------------------------------------------------------------------------------------------
;-  Function name   : start
;-  Content : reset start
;-      : program initialize routine
;-
;-  Argument    : none
;-      : 
;-
;-  Return value    : none
;-      : 
;-
;-  Date/Author : 
;-  Change Log  : 
;-      : 
;---------------------------------------------------------------------------------------------------
_start

;*--- init control registers ---*
    clr d0
    mov d0,(CPUM)       ;CPUM NORMAL mode

    mov 0x0104,d0      ;40MHz ROMCTR 2 cycle access
    ;mov 0x0204,d0       ;60MHz ROMCTR 3 cycle access
    mov d0,(ROMCTR)

;*--- set Interrupt Enable Flag ---*
    mov IRQ_DISABLE,d0      ;disable all interrupt
    mov d0,psw
    nop
    nop

; set ivar
    mov irq_lev0,d0     ;set interrupt vector register of level 0
    movhu d0,(IVAR0)
    mov irq_lev1,d0     ;set interrupt vector register of level 1
    movhu d0,(IVAR1)
    mov irq_lev2,d0     ;set interrupt vector register of level 2
    movhu d0,(IVAR2)
    mov irq_lev3,d0     ;set interrupt vector register of level 3
    movhu d0,(IVAR3)
    mov irq_lev4,d0     ;set interrupt vector register of level 4
    movhu d0,(IVAR4)
    mov irq_lev5,d0     ;set interrupt vector register of level 5
    movhu d0,(IVAR5)
    mov irq_lev6,d0     ;set interrupt vector register of level 6
    movhu d0,(IVAR6)

;*--- stack clear ---*
    mov stack+STACK_SIZE, a0    ;set stack pointer
    mov a0, sp

    add -4, sp
    
;*--- oscillation setting---*
    mov OSC_SEL,d0        
    cmp 0,d0
	  beq osc_ok
    mov 0x0000,d1     
	  mov d1,(OSCLOCK)     ;Oscillation control register (OSCCNT) Writing enable
    mov 0x0001,d0     
	  mov d0,(OSCCNT)      ; External oscillation feedback ON        
	  mov WAIT_200US,d1    ;(1)The counter for the loop is set. 
ext_wait
	  sub 1,d1             ;(2)The counter for the loop is decreased. 
	  bne ext_wait         ;(3/1)(branched/not branched)It loops when the condition is unapproval.
	  nop                  ;(1)correction for branch.
    nop                  ;(1)correction for branch.
	  or OSC_SEL,d0        ;OSCSEL is selected.
 	  mov d0,(OSCCNT)      ;operation clock  to the External oscillation  
osc_ok
    mov 0x0001,d1     
    mov d1,(OSCLOCK)     ;Oscillation control register (OSCCNT) Writing disable
    nop
    nop   
	  
;*--- clock setting---*

    mov PLL_SEL,d0        ;PLL multiply ratio is selected.
    movhu d0,(PCNT)       ;The setting is forwarded to the PCNT register.
    mov WAIT_200US,d1     ;(1)The counter for the loop is set. 
cloop
    sub 1,d1              ;(2)The counter for the loop is decreased. 
    bne cloop             ;(3/1)(branched/not branched)It loops when the condition is unapproval.
    nop                   ;(1)correction for branch.
    nop                   ;(1)correction for branch.
    or  0x0020,d0         ;(1)PLL output is selected.
    movhu d0,(PCNT)       ;The setting is forwarded to the PCNT register.
    mov IOCLK_SEL,d0      ;IOCLK is selected.
    or  MCLK_SEL,d0       ;MCLK is selected.
    or  0x0080,d0         ;bit7-6 is set to 0x10.
    movhu d0,(CKCTR)      ;The setting is forwarded to the CKCTR register.

;*--- RAM clear ---*
;clear _BSS, _GBSS        ;clear RAM of non-initialized area
    mov RAM_SIZE, d1      ;clear all RAM area
    cmp 0, d1
    ble bc_skip           ;skip if there are no BSS,GBSS.
    
    mov 0, a0             ;address of top of RAM
    clr d0                ;clear register D0
bc_loop
    mov d0, (a0)          ;clear RAM
    inc4  a0              ;address decrement(by 4 Byte)
    add -4, d1            ;update loop counter
    bgt bc_loop           ;jump to bc_loop if it is not end of RAM area
bc_skip

;*--- RAM init ---*
; copy _ROMDATA, _GROMDATA -> _DATA, _GDATA ;
    mov .MN.romdataend - .MN.romdata,d1 ;set size of initialized data area
    cmp 0, d1
    ble dc_skip       ;skip if there are no initialized data

    mov .MN.romdata, a0     ;top address of initialized data area
    mov .MN.data, a1      ;top address of initialized data area (destination)
dc_loop
    mov (a0), d0      ;read initialized data
    inc4  a0          ;address increment(by 4 Byte)
    mov d0, (a1)      ;write initialized data
    inc4  a1          ;address increment(by 4 Byte)
    add -4, d1        ;update loop counter
    bgt dc_loop       ;jump to bc_loop if it is not end of RAM area

dc_skip
    clr d0            ;clear register DO
    mov d0, d1        ;clear register D1
    mov d0, d2        ;clear register D2
    mov d0, d3        ;clear register D3
    mov d0, a0        ;clear register A0
    mov d0, a1        ;clear register A1
    mov d0, a2        ;clear register A2
    mov d0, a3        ;clear register A3
    call  _0main        ;branch to main routine

    bra dc_skip

    end
