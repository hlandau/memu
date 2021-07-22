#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <exception>
#include <tuple>

#define PP_CAT_(X,Y) X##Y
#define PP_CAT(X,Y) PP_CAT_(X,Y)

#define ARRAYLEN(X) (sizeof(X)/sizeof((X)[0]))

#define CTZL(N)               __builtin_ctzl(N)
#define MASK_TO_SHIFT(Mask)   ((Mask) ? CTZL(Mask) : 0)

#define BIT(N)                (1UL<<(N))
#define BITS(Lo,Hi)           ((UINTPTR_MAX<<(Lo)) & (UINTPTR_MAX>>((sizeof(uintptr_t)*8-1)-(Hi))))
                              // bits [X:Y] (e.g. BITS(2,15))
// Get bit N from a field.
#define GETBIT(Value, N)                    (((Value) & BIT(N)       ) >> (N))
// Get bits X:Y from a field.
#define GETBITS(Value, X, Y)                (((Value) & BITS((X),(Y))) >> (X))
// Get bits from a field, where the bits to retrieve are defined by a mask
// value.
#define GETBITSM(Value, Mask)               (((Value) & (Mask))        >> MASK_TO_SHIFT(Mask))
// Shift bits into bits X:Y and ensure they do not exceed the size of the
// field.
#define PUTBITS(Value, X, Y)                (((Value) << (X)) & BITS((X),(Y)))
// Shift bits into the field defined by the given mask and ensure they do not
// exceed the mask.
#define PUTBITSM(Value, Mask)               (((Value) << MASK_TO_SHIFT(Mask)) & (Mask))
// Change bits X:Y in a field to the new value.
#define CHGBITS(OldValue, X, Y, NewValue)   (((OldValue) & ~BITS((X),(Y)))|PUTBITS((NewValue), (X), (Y)))
// Change bits in the field defined by the given mask to the new value.
#define CHGBITSM(OldValue, Mask, NewValue)  (((OldValue) & ~(Mask))|PUTBITSM((NewValue), (Mask)))

#define likely(Expr)      (__builtin_expect(!!(Expr), 1))
#define unlikely(Expr)    (__builtin_expect(!!(Expr), 0))
#define UNREACHABLE()     do { __builtin_unreachable(); } while(0)

#define NUM_EXC 512

#define UNDEFINED_VAL(X)  (X)
#define UNKNOWN_VAL(X)    (X)

#ifndef EMU_TRACE
#  define EMU_TRACE 0
#endif

#if EMU_TRACE
int g_emuTraceIndent;
struct EmuTraceBlock {
  EmuTraceBlock() { ++g_emuTraceIndent; }
  ~EmuTraceBlock() { --g_emuTraceIndent; }
};

#  define TRACE_BLOCK() EmuTraceBlock PP_CAT(_traceblk_,__COUNTER__)
#  define TRACE(...) do { printf("T: "); for (size_t _traceI=0; _traceI<g_emuTraceIndent; ++_traceI) printf("  "); printf(__VA_ARGS__); } while(0)
#else
#  define TRACE(...) do {} while (0)
#endif

#define ASSERT(Cond) do { if unlikely (!(Cond)) { printf("assertion fail on line %u: %s\n", __LINE__, #Cond); abort(); } } while(0)

enum {
  NoFault      =  0,
  Reset        =  1,
  NMI          =  2,
  HardFault    =  3,
  MemManage    =  4,
  BusFault     =  5,
  UsageFault   =  6,
  SecureFault  =  7,
  SVCall       = 11,
  DebugMonitor = 12,
  PendSV       = 14,
  SysTick      = 15,
};

struct ExcInfo {
  int fault;
  int origFault;
  bool isSecure;
  bool origFaultIsSecure;
  bool isTerminal;
  bool inExcTaken;
  bool lockup;
  bool termInst;
};

enum class ExceptionType {
  SEE,
  UNDEFINED,
  EndOfInstruction,
  UNPREDICTABLE,
  Internal,
};

struct Exception :std::exception {
  Exception(ExceptionType type) :_type(type) {}

  ExceptionType GetType() const { return _type; }

private:
  ExceptionType _type;
};

#define IMPL_DEF_DECODE_CP_SPACE            1
#define IMPL_DEF_EARLY_SG_CHECK             1
#define IMPL_DEF_SPLIM_CHECK_UNPRED_INSTR   1
#define IMPL_DEF_IDAU_PRESENT               0
#define IMPL_DEF_PUSH_NON_VIOL_LOCATIONS    0
#define IMPL_DEF_OVERRIDDEN_EXCEPTIONS_PENDED   1 /*TODO*/
#define IMPL_DEF_TAIL_CHAINING_SUPPORTED    1
#define IMPL_DEF_DROP_PREV_GEN_EXC          1

#define SECREG(Name) (_IsSecure() ? Name##_S : Name##_NS)

#define REG_DHCSR     0xE000'EDF0
#define REG_DHCSR_NS  0xE002'EDF0
#define   REG_DHCSR__S_HALT     BIT(17)
#define   REG_DHCSR__S_LOCKUP   BIT(19)
#define   REG_DHCSR__S_SDE      BIT(20)
#define REG_DEMCR     0xE000'EDFC
#define REG_DEMCR_NS  0xE002'EDFC
#define   REG_DEMCR__MON_EN     BIT(16)
#define   REG_DEMCR__MON_PEND   BIT(17)
#define   REG_DEMCR__MON_STEP   BIT(18)
#define   REG_DEMCR__SDME       BIT(20)
#define   REG_DEMCR__TRCENA     BIT(24)
#define REG_FPDSCR_S  0xE000'EF3C
#define REG_FPDSCR_NS 0xE002'EF3C
#define   REG_FPDSCR__RMODE     BITS(22,23)
#define   REG_FPDSCR__FZ        BIT (24)
#define   REG_FPDSCR__DN        BIT (25)
#define   REG_FPDSCR__AHP       BIT (26)

#define REG_AIRCR     SECREG(REG_AIRCR)
#define REG_AIRCR_S   0xE000'ED0C
#define REG_AIRCR_NS  0xE002'ED0C
#define   REG_AIRCR__VECTCLRACTIVE  BIT ( 1)
#define   REG_AIRCR__SYSRESETREQ    BIT ( 2)
#define   REG_AIRCR__SYSRESETREQS   BIT (3)
#define   REG_AIRCR__PRIGROUP       BITS( 8,10)
#define   REG_AIRCR__BFHFNMINS      BIT (13)
#define   REG_AIRCR__PRIS           BIT (14)
#define   REG_AIRCR__ENDIANNESS     BIT (15)
#define   REG_AIRCR__VECTKEY        BITS(16,31)
#define REG_DAUTHCTRL 0xE000'EE04
#define   REG_DAUTHCTRL__SPIDENSEL  BIT ( 0)
#define   REG_DAUTHCTRL__INTSPIDEN  BIT ( 1)
#define   REG_DAUTHCTRL__SPNIDENSEL BIT ( 2)
#define   REG_DAUTHCTRL__INTSPNIDEN BIT ( 3)

#define REG_UFSR      SECREG(REG_UFSR)
#define REG_UFSR_S    0xE000'ED2A
#define REG_UFSR_NS   0xE002'ED2A
#define   REG_UFSR__UNDEFINSTR      BIT ( 0)
#define   REG_UFSR__INVSTATE        BIT ( 1)
#define   REG_UFSR__INVPC           BIT ( 2)
#define   REG_UFSR__NOCP            BIT ( 3)
#define   REG_UFSR__STKOF           BIT ( 4)
#define   REG_UFSR__UNALIGNED       BIT ( 8)
#define   REG_UFSR__DIVBYZERO       BIT ( 9)
#define REG_DFSR      0xE000'ED30
#define   REG_DFSR__HALTED          BIT ( 0)
#define   REG_DFSR__BKPT            BIT ( 1)
#define   REG_DFSR__DWTTRAP         BIT ( 2)
#define   REG_DFSR__VCATCH          BIT ( 3)
#define   REG_DFSR__EXTERNAL        BIT ( 4)

#define REG_FPCCR     SECREG(REG_FPCCR)
#define REG_FPCCR_S   0xE000'EF34
#define REG_FPCCR_NS  0xE002'EF34
#define   REG_FPCCR__LSPACT         BIT ( 0)
#define   REG_FPCCR__USER           BIT ( 1)
#define   REG_FPCCR__S              BIT ( 2)
#define   REG_FPCCR__THREAD         BIT ( 3)
#define   REG_FPCCR__HFRDY          BIT ( 4)
#define   REG_FPCCR__TS             BIT (26)
#define   REG_FPCCR__CLRONRET       BIT (28)
#define   REG_FPCCR__LSPENS         BIT (29)
#define   REG_FPCCR__LSPEN          BIT (30)
#define   REG_FPCCR__ASPEN          BIT (31)

#define REG_FPCAR_S   0xE000'EF38
#define REG_FPCAR_NS  0xE002'EF38

#define REG_DHCSR_S   0xE000'EDF0
#define REG_DHCSR_NS  0xE002'EDF0
#define   REG_DHCSR__C_DEBUGEN      BIT ( 0)
#define   REG_DHCSR__C_HALT         BIT ( 1)
#define   REG_DHCSR__C_STEP         BIT ( 2)
#define   REG_DHCSR__C_MASKINTS     BIT ( 3)

#define REG_CCR       SECREG(REG_CCR)
#define REG_CCR_S     0xE000'ED14
#define REG_CCR_NS    0xE002'ED14
#define   REG_CCR__UNALIGN_TRP        BIT ( 3)
#define   REG_CCR__BFHFNMIGN          BIT ( 8)
#define   REG_CCR__STKOFHFNMIGN       BIT (10)

#define REG_VTOR_S    0xE000'ED08
#define REG_VTOR_NS   0xE002'ED08

#define REG_HFSR      SECREG(REG_HFSR)
#define REG_HFSR_S    0xE000'ED2C
#define REG_HFSR_NS   0xE002'ED2C
#define REG_HFSR__VECTTBL   BIT ( 1)
#define REG_HFSR__FORCED    BIT (30)

#define REG_SFSR      SECREG(REG_SFSR)
#define REG_SFSR_S    0xE000'EDE4
#define REG_SFSR_NS   0xE002'EDE4
#define REG_SFSR__INVEP     BIT ( 0)
#define REG_SFSR__INVIS     BIT ( 1)
#define REG_SFSR__INVER     BIT ( 2)
#define REG_SFSR__AUVIOL    BIT ( 3)
#define REG_SFSR__INVTRAN   BIT ( 4)
#define REG_SFSR__LSPERR    BIT ( 5)
#define REG_SFSR__SFARVALID BIT ( 6)
#define REG_SFSR__LSERR     BIT ( 7)

#define REG_SFAR      SECREG(REG_SFAR)
#define REG_SFAR_S    0xE000EDE8
#define REG_SFAR_NS   0xE002EDE8

#define REG_BFSR      SECREG(REG_BFSR)
#define REG_BFSR_S    0xE000ED29
#define REG_BFSR_NS   0xE002ED29
#define   REG_BFSR__IBUSERR     BIT ( 0)
#define   REG_BFSR__PRECISERR   BIT ( 1)
#define   REG_BFSR__IMPRECISERR BIT ( 2)
#define   REG_BFSR__UNSTKERR    BIT ( 3)
#define   REG_BFSR__STKERR      BIT ( 4)
#define   REG_BFSR__LSPERR      BIT ( 5)
#define   REG_BFSR__BFARVALID   BIT ( 7)

#define REG_BFAR      SECREG(REG_BFAR)
#define REG_BFAR_S    0xE000ED38
#define REG_BFAR_NS   0xE002ED38
#define   REG_BFAR__ADDRESS BITS( 0,31)

#define REG_MMFSR     SECREG(REG_MMFSR)
#define REG_MMFSR_S   0xE000'ED28
#define REG_MMFSR_NS  0xE002'ED28
#define   REG_MMFSR__IACCVIOL   BIT ( 0)
#define   REG_MMFSR__DACCVIOL   BIT ( 1)
#define   REG_MMFSR__MUNSTKERR  BIT ( 3)
#define   REG_MMFSR__MSTKERR    BIT ( 4)
#define   REG_MMFSR__MLSPERR    BIT ( 5)
#define   REG_MMFSR__MMARVALID  BIT ( 7)

#define REG_MMFAR     SECREG(REG_MMFAR)
#define REG_MMFAR_S   0xE000'ED34
#define REG_MMFAR_NS  0xE002'ED34
#define   REG_MMFAR__ADDRESS    BITS ( 0,31)

#define REG_MPU_TYPE    SECREG(REG_MPU_TYPE)
#define REG_MPU_TYPE_S  0xE000ED90
#define REG_MPU_TYPE_NS 0xE002ED90
#define   REG_MPU_TYPE__DREGION   BITS( 8,15)

#define REG_MPU_CTRL    SECREG(REG_MPU_CTRL)
#define REG_MPU_CTRL_S  0xE000ED94
#define REG_MPU_CTRL_NS 0xE002ED94
#define   REG_MPU_CTRL__ENABLE      BIT ( 0)
#define   REG_MPU_CTRL__HFNMIENA    BIT ( 1)
#define   REG_MPU_CTRL__PRIVDEFENA  BIT ( 2)

#define REG_MPU_RBAR__XN        BIT ( 0)
#define REG_MPU_RBAR__AP        BITS( 1, 2)
#define REG_MPU_RBAR__SH        BITS( 3, 4)
#define REG_MPU_RBAR__BASE      BITS( 5,31)

#define REG_MPU_RLAR__EN        BIT ( 0)
#define REG_MPU_RLAR__ATTR_IDX  BITS( 1, 3)
#define REG_MPU_RLAR__LIMIT     BITS(5,31)

#define REG_MPU_MAIR0     SECREG(REG_MPU_MAIR0)
#define REG_MPU_MAIR0_S   0xE000EDC0
#define REG_MPU_MAIR0_NS  0xE002EDC0

#define REG_MPU_MAIR1     SECREG(REG_MPU_MAIR1)
#define REG_MPU_MAIR1_S   0xE000EDC4
#define REG_MPU_MAIR1_NS  0xE002EDC4

#define REG_DWT_CTRL      SECREG(REG_DWT_CTRL)
#define REG_DWT_CTRL_S    0xE000'1000
#define REG_DWT_CTRL_NS   0xE002'1000
#define   REG_DWT_CTRL__NUMCOMP   BITS(28,31)

#define REG_DWT_FUNCTION(N) (0xE000'1028 + 16*(N))
#define   REG_DWT_FUNCTION__MATCH   BITS( 0, 3)
#define   REG_DWT_FUNCTION__ACTION  BITS( 4, 5)
#define   REG_DWT_FUNCTION__MATCHED BIT (24)

#define REG_FP_CTRL       0xE000'2000
#define   REG_FP_CTRL__ENABLE       BIT ( 0)
#define   REG_FP_CTRL__KEY          BIT ( 1)
#define   REG_FP_CTRL__NUM_CODE_LO  BITS( 4, 7)
#define   REG_FP_CTRL__NUM_CODE_HI  BITS(12,14)
#define   REG_FP_CTRL__NUM_LIT      BITS( 8,11)
#define   REG_FP_CTRL__REV          BITS(28,31)

#define REG_SHCSR         SECREG(REG_SHCSR)
#define REG_SHCSR_S       0xE000'ED24
#define REG_SHCSR_NS      0xE002'ED24
#define   REG_SHCSR__MEMFAULTACT      BIT ( 0)
#define   REG_SHCSR__BUSFAULTACT      BIT ( 1)
#define   REG_SHCSR__HARDFAULTACT     BIT ( 2)
#define   REG_SHCSR__USGFAULTACT      BIT ( 3)
#define   REG_SHCSR__SECUREFAULTACT   BIT ( 4)
#define   REG_SHCSR__NMIACT           BIT ( 5)
#define   REG_SHCSR__SVCALLACT        BIT ( 7)
#define   REG_SHCSR__MONITORACT       BIT ( 8)
#define   REG_SHCSR__PENDSVACT        BIT (10)
#define   REG_SHCSR__SYSTICKACT       BIT (11)
#define   REG_SHCSR__USGFAULTPENDED   BIT (12)
#define   REG_SHCSR__MEMFAULTPENDED   BIT (13)
#define   REG_SHCSR__BUSFAULTPENDED   BIT (14)
#define   REG_SHCSR__SVCALLPENDED     BIT (15)
#define   REG_SHCSR__MEMFAULTENA      BIT (16)
#define   REG_SHCSR__BUSFAULTENA      BIT (17)
#define   REG_SHCSR__USGFAULTENA      BIT (18)
#define   REG_SHCSR__SECUREFAULTENA   BIT (19)
#define   REG_SHCSR__SECUREFAULTPENDED  BIT (20)
#define   REG_SHCSR__HARDFAULTPENDED    BIT (21)

#define REG_FP_COMP(N)     (0xE000'2008+4*(N))
#define   REG_FP_COMPn__BE             BIT ( 0)
#define   REG_FP_COMPn__BPADDR         BITS( 1,31)

#define REG_SAU_CTRL      0xE000'EDD0
#define   REG_SAU_CTRL__ENABLE        BIT ( 0)
#define   REG_SAU_CTRL__ALLNS         BIT ( 1)
#define REG_SAU_TYPE      0xE000'EDD4
#define   REG_SAU_TYPE__SREGION       BITS( 0, 7)

#define REG_SAU_RBAR__BADDR   BITS( 5,31)
#define REG_SAU_RLAR__ENABLE  BIT ( 0)
#define REG_SAU_RLAR__NSC     BIT ( 1)
#define REG_SAU_RLAR__LADDR   BITS( 5,31)

#define REG_NSACR         0xE000'ED8C
#define   REG_NSACR__CP(N) BIT(N)

#define REG_CPACR         SECREG(REG_CPACR)
#define REG_CPACR_S       0xE000'ED88
#define REG_CPACR_NS      0xE002'ED88
#define   REG_CPACR__CPn(N)   BITS(2*(N),2*(N)+1)

#define REG_CPPWR         SECREG(REG_CPPWR)
#define REG_CPPWR_S       0xE000'E00C
#define REG_CPPWR_NS      0xE002'E00C
#define   REG_CPPWR__SUn(N)   BIT (N*2)
#define   REG_CPPWR__SUSn(N)  BIT (N*2+1)

#define REG_ICSR_S        0xE000'ED04
#define REG_ICSR_NS       0xE002'ED04
#define   REG_ICSR__VECTACTIVE  BITS( 0, 8)
#define   REG_ICSR__RETTOBASE   BIT (11)
#define   REG_ICSR__VECTPENDING BITS(12,20)
#define   REG_ICSR__ISRPENDING  BIT (22)
#define   REG_ICSR__ISRPREEMPT  BIT (23)
#define   REG_ICSR__STTNS       BIT (24)
#define   REG_ICSR__PENDSTCLR   BIT (25)
#define   REG_ICSR__PENDSTSET   BIT (26)
#define   REG_ICSR__PENDSVCLR   BIT (27)
#define   REG_ICSR__PENDSVSET   BIT (28)
#define   REG_ICSR__PENDNMICLR  BIT (30)
#define   REG_ICSR__PENDNMISET  BIT (31)

#define REG_SCR           SECREG(REG_SCR)
#define REG_SCR_S         0xE000'ED10
#define REG_SCR_NS        0xE000'ED10
#define   REG_SCR__SLEEPONEXIT    BIT ( 1)

#define REG_NVIC_ITNSn(N) (0xE000'E380+4*(N))

#define EXC_RETURN__ES      BIT ( 0)
#define EXC_RETURN__SPSEL   BIT ( 2)
#define EXC_RETURN__MODE    BIT ( 3)
#define EXC_RETURN__FTYPE   BIT ( 4)
#define EXC_RETURN__DCRS    BIT ( 5)
#define EXC_RETURN__S       BIT ( 6)
#define EXC_RETURN__PREFIX  BITS(24,31)

#define REG_SHPR1_S       0xE000'ED18
#define REG_SHPR1_NS      0xE002'ED18
#define   REG_SHPR1__PRI_4  BITS( 0, 7)
#define   REG_SHPR1__PRI_5  BITS( 8,15)
#define   REG_SHPR1__PRI_6  BITS(16,23)
#define   REG_SHPR1__PRI_7  BITS(24,31)
#define REG_SHPR2_S       0xE000'ED1C
#define REG_SHPR2_NS      0xE002'ED1C
#define   REG_SHPR2__PRI_8  BITS( 0, 7)
#define   REG_SHPR2__PRI_9  BITS( 8,15)
#define   REG_SHPR2__PRI_10 BITS(16,23)
#define   REG_SHPR2__PRI_11 BITS(24,31)
#define REG_SHPR3_S       0xE000'ED20
#define REG_SHPR3_NS      0xE002'ED20
#define   REG_SHPR3__PRI_12 BITS( 0, 7)
#define   REG_SHPR3__PRI_13 BITS( 8,15)
#define   REG_SHPR3__PRI_14 BITS(16,23)
#define   REG_SHPR3__PRI_15 BITS(24,31)

#define REG_NVIC_IPRn_S(N)  (0xE000'E400 + 4*(N))
#define REG_NVIC_IPRn_NS(N) (0xE002'E400 + 4*(N))

#define REG_NVIC_ISPRn_S(N)  (0xE000'E200 + 4*(N))
#define REG_NVIC_ISPRn_NS(N) (0xE002'E200 + 4*(N))

#define PRIMASK__PM     BIT(0)
#define FAULTMASK__FM   BIT(0)

using phys_t = uint32_t;

enum PEMode {
  PEMode_Thread,
  PEMode_Handler,
};

enum SecurityState {
  SecurityState_NonSecure,
  SecurityState_Secure,
};

enum RName {
  RName0, RName1, RName2, RName3, RName4, RName5, RName6, RName7, RName8, RName9, RName10, RName11, RName12,
  RNameSP_Main_NonSecure, RNameSP_Process_NonSecure, RName_LR, RName_PC, RNameSP_Main_Secure, RNameSP_Process_Secure,
  _RName_Max
};

enum SRType {
  SRType_LSL,
  SRType_LSR,
  SRType_ASR,
  SRType_ROR,
  SRType_RRX,
};

struct CpuState {
  union {
    struct {
      uint32_t r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,spMainNS,spProcessNS,lr,pc,spMainS,spProcessS;
    };
    uint32_t r[_RName_Max];
  };
  uint32_t xpsr, spNS, psplimNS, psplimS, msplimNS, msplimS, fpscr;
  uint32_t primaskNS, primaskS, faultmaskNS, faultmaskS, basepriNS, basepriS, controlNS, controlS;
  SecurityState curState;
  uint8_t excActive[NUM_EXC];
  uint8_t excPending[NUM_EXC];
  uint64_t d[16];
  bool event, pendingReturnOperation;
  bool itStateChanged;
  bool pcChanged;
  uint8_t nextInstrITState;
  uint32_t nextInstrAddr;
  uint32_t thisInstr;
  uint8_t  thisInstrLength;
  uint32_t thisInstrDefaultCond;

  // Implementation-specific state
  char curCondOverride; // _CurrentCond()
};

struct CpuNest {
  uint32_t fpdscrS{}, fpdscrNS{}, fpccrS{BIT(2)|BIT(30)|BIT(31)}, fpccrNS{BIT(2)|BIT(30)|BIT(31)}, fpcarS{}, fpcarNS{}, vtorS{0x2000'4000}, vtorNS{0x2000'4000}, sauCtrl{}, mpuTypeS{}, mpuTypeNS{}, mpuCtrlS{}, mpuCtrlNS{}, mpuMair0S{}, mpuMair0NS{}, mpuMair1S{}, mpuMair1NS{}, aircrS{}, aircrNS{}, demcrS{}, demcrNS{}, dhcsrS{}, dhcsrNS{}, dauthCtrl{}, mmfsrS{}, mmfsrNS{}, shcsrS{}, shcsrNS{}, shpr1S{}, shpr1NS{}, hfsrS{}, hfsrNS{}, ufsrS{}, ufsrNS{}, fpCtrl{}, nvicNonSecure[16]{}, nvicIntrPrio[124]{};
};

#define CONTROL__NPRIV    BIT(0)
#define CONTROL__SPSEL    BIT(1)
#define CONTROL__FPCA     BIT(2)
#define CONTROL__SFPA     BIT(3)

#define XPSR__EXCEPTION   BITS( 0, 8)
#define XPSR__T           BIT (24)
#define XPSR__N           BIT (31)
#define XPSR__Z           BIT (30)
#define XPSR__C           BIT (29)
#define XPSR__V           BIT (28)
#define XPSR__Q           BIT (27)
#define XPSR__IT_ICI_LO   BITS(10,15)
#define XPSR__IT_ICI_HI   BITS(25,26)
// IT  = (IT_ICI_LO<<2) | IT_ICI_HI
// ICI = (IT_ICI_HI<<6) | IT_ICI_LO
#define XPSR__GE          BITS(16,19)

#define RETPSR__EXCEPTION BITS( 0, 8)
#define RETPSR__SPREALIGN BIT ( 9)
#define RETPSR__IT_ICI_LO BITS(10,15)
#define RETPSR__IT_ICI_HI BITS(25,26)
#define RETPSR__GE        BITS(16,19)
#define RETPSR__SFPA      BIT (20)
#define RETPSR__T         BIT (24)

struct IDevice {
  virtual void InternalReset() = 0;
  virtual uint32_t InternalLoad32(phys_t addr) = 0;
  virtual void InternalStore32(phys_t addr, uint32_t v) = 0;
  virtual std::tuple<uint32_t,uint32_t> InternalLoadMpuSecureRegion(size_t idx) = 0; // {RBAR, RLAR}
  virtual std::tuple<uint32_t,uint32_t> InternalLoadMpuNonSecureRegion(size_t idx) = 0; // {RBAR, RLAR}
  virtual std::tuple<uint32_t,uint32_t> InternalLoadSauRegion(size_t idx) = 0; // {RBAR, RLAR}

  virtual int Load32(phys_t addr, uint32_t &v) = 0;
  virtual int Load16(phys_t addr, uint16_t &v) = 0;
  virtual int Load8(phys_t addr, uint8_t &v) = 0;
  virtual int Store32(phys_t addr, uint32_t v) = 0;
  virtual int Store16(phys_t addr, uint16_t v) = 0;
  virtual int Store8(phys_t addr, uint8_t v) = 0;
};

enum MemType {
  MemType_Normal,
  MemType_Device,
};

enum DeviceType {
  DeviceType_GRE,
  DeviceType_nGRE,
  DeviceType_nGnRE,
  DeviceType_nGnRnE,
};

struct MemoryAttributes {
  MemType     memType;
  DeviceType  device;
  uint8_t     innerAttrs, outerAttrs, innerHints, outerHints;
  bool        ns, innerTransient, outerTransient, shareable, outerShareable;
};

enum AccType {
  AccType_NORMAL,
  AccType_ORDERED,
  AccType_STACK,
  AccType_LAZYFP,
  AccType_IFETCH,
  AccType_VECTABLE,
};

struct AccessAttributes {
  bool isWrite, isPriv;
  AccType accType;
};

struct AddressDescriptor {
  MemoryAttributes  memAttrs;
  uint32_t          physAddr;
  AccessAttributes  accAttrs;
};

struct SAttributes {
  bool    nsc, ns;
  uint8_t sregion;
  bool    srvalid;
  uint8_t iregion;
  bool    irvalid;
};

struct Permissions {
  bool    apValid;
  uint8_t ap;
  bool    xn;
  bool    regionValid;
  uint8_t region;
};

struct Emulator {
  Emulator(IDevice &dev) :_dev(dev) {
    _TakeReset();
  }

  void TopLevel() { _TopLevel(); }

private:
  void _TopLevel();
  ExcInfo _CreateException(int exc, bool forceSecurity, bool isSecure, bool isSync=true);
  void _EndOfInstruction();
  static bool _IsSEE(const Exception &e) { return e.GetType() == ExceptionType::SEE; }
  static bool _IsUNDEFINED(const Exception &e) { return e.GetType() == ExceptionType::UNDEFINED; }
  static bool _IsExceptionTaken(const Exception &e) { return e.GetType() == ExceptionType::EndOfInstruction; }

  void _NestReset() {
  }

  // {targetNS, targetRAZWI, targetFault}
  std::tuple<bool,bool,bool> _NestAccessClassify(phys_t addr, bool isPriv, bool isSecure) {
    bool isAltSpace = !!(addr & 0x2'0000);
    uint8_t code =
      (((uint32_t)isSecure  ) << 2)
    | (((uint32_t)isPriv    ) << 1)
    | (((uint32_t)isAltSpace) << 0)
      ;

    bool targetNS     = false;
    bool targetRAZWI  = false;
    bool targetFault  = false;
    bool isStir       = ((addr & ~0x2'0000U) == 0xE000'EF00);

    // RAZ/WI: Read as Zero, Writes Ignored
    // RES0:   Should be Zero or Preserved (SBZP)
    //
    //                                        SE                          !SE
    //                                        --------------------------  --------------------------
    // SECURE     PRIV  ACCESS TO 0xE000_xxxx Secure space                Secure space
    // SECURE     PRIV  ACCESS TO 0xE002_xxxx Non-Secure space            RAZ/WI
    // SECURE     NPRIV ACCESS TO 0xE000_xxxx BusFault (STIR: Secure sp)  BusFault (STIR: Secure sp)
    // SECURE     NPRIV ACCESS TO 0xE002_xxxx BusFault                    BusFault
    // NON-SECURE PRIV  ACCESS TO 0xE000_xxxx Non-Secure space            Secure space
    // NON-SECURE PRIV  ACCESS TO 0xE002_xxxx RES0                        RAZ/WI
    // NON-SECURE NPRIV ACCESS TO 0xE000_xxxx BusFault (STIR: NS sp)      BusFault (STIR: Secure sp)
    // NON-SECURE NPRIV ACCESS TO 0xE002_xxxx BusFault                    BusFault
    //
    // INTERNAL         ACCESS TO 0xE000_xxxx Secure space                Secure space
    // INTERNAL         ACCESS TO 0xE002_xxxx Non-Secure space            [TODO] Non-Secure space

    //       SPA    S=Secure?  P=Privileged?  A=Alt Space?
    switch (code) {
      case 0b110: targetNS = false; break;
      case 0b111: if (_HaveSecurityExt()) targetNS = true; else targetRAZWI = true; break;
      case 0b100: if (isStir) targetNS = false; else targetFault = true ; break;
      case 0b101: targetFault = true; break; // ?
      case 0b010: targetNS = _HaveSecurityExt(); break;
      case 0b011: targetRAZWI = true; break;
      case 0b000: if (isStir) targetNS = _HaveSecurityExt(); else targetFault = true; break;
      case 0b001: targetFault = true; break;
    }

    return {targetNS, targetRAZWI, targetFault};
  }

  // Returns nonzero for BusFault.
  int _NestLoad32(phys_t addr, bool isPriv, bool isSecure, uint32_t &v) {
    auto [targetNS, targetRAZWI, targetFault] = _NestAccessClassify(addr, isPriv, isSecure);

    if (targetFault)
      return -1;

    if (targetRAZWI)
      v = 0;
    else if (targetNS)
      v = _NestLoad32Actual(addr |  0x2'0000U);
    else
      v = _NestLoad32Actual(addr & ~0x2'0000U);

    return 0;
  }

  uint32_t _NestLoad32Actual(phys_t addr) {
    phys_t baddr = addr & ~0x2'0000U;

    switch (addr) {
      case REG_FPDSCR_S:      return _n.fpdscrS;
      case REG_FPDSCR_NS:     return _n.fpdscrNS;
      case REG_FPCCR_S:       return _n.fpccrS;
      case REG_FPCCR_NS:      return _n.fpccrNS;
      case REG_FPCAR_S:       return _n.fpcarS;
      case REG_FPCAR_NS:      return _n.fpcarNS;
      case REG_VTOR_S:        return _n.vtorS;
      case REG_VTOR_NS:       return _n.vtorNS;
      case REG_SAU_CTRL:      return _n.sauCtrl;
      case REG_MPU_TYPE_S:    return _n.mpuTypeS;
      case REG_MPU_TYPE_NS:   return _n.mpuTypeNS;
      case REG_MPU_CTRL_S:    return _n.mpuCtrlS;
      case REG_MPU_CTRL_NS:   return _n.mpuCtrlNS;
      case REG_MPU_MAIR0_S:   return _n.mpuMair0S;
      case REG_MPU_MAIR0_NS:  return _n.mpuMair0NS;
      case REG_MPU_MAIR1_S:   return _n.mpuMair1S;
      case REG_MPU_MAIR1_NS:  return _n.mpuMair1NS;
      case REG_AIRCR_S:       return _n.aircrS;
      case REG_AIRCR_NS:      return _n.aircrNS;
      case REG_DEMCR:         return _n.demcrS;
      case REG_DEMCR_NS:      return _n.demcrNS;
      case REG_DHCSR:         return _n.dhcsrS;
      case REG_DHCSR_NS:      return _n.dhcsrNS;
      case REG_DAUTHCTRL:     return _n.dauthCtrl;
      case REG_MMFSR_S:       return _n.mmfsrS;
      case REG_MMFSR_NS:      return _n.mmfsrNS;
      case REG_SHCSR_S:       return _n.shcsrS;
      case REG_SHCSR_NS:      return _n.shcsrNS;
      case REG_SHPR1_S:       return _n.shpr1S;
      case REG_SHPR1_NS:      return _n.shpr1NS;
      case REG_HFSR_S:        return _n.hfsrS;
      case REG_HFSR_NS:       return _n.hfsrNS;
      case REG_UFSR_S:        return _n.ufsrS;
      case REG_UFSR_NS:       return _n.ufsrNS;
      case REG_FP_CTRL:       return _n.fpCtrl;
      default:
        if (baddr >= 0xE000E200 && baddr < 0xE000E240)
          return _NestLoadNvicPendingReg((addr/4)&0xF, /*secure=*/!(addr & 0x2'0000));

        if (addr >= 0xE000E380 && addr < 0xE000E3C0)
          return _n.nvicNonSecure[(addr/4)&0xF];

        if (addr >= 0xE000E400 && addr < 0xE000E5F0)
          return _n.nvicIntrPrio[(addr - 0xE000E400)/4];

        printf("Unsupported nest load 0x%08x\n", addr);
        abort();
    }
  }

  uint32_t _NestLoadNvicPendingReg(uint32_t groupNo, bool isSecure) {
    uint32_t v      = 0;
    uint32_t itns   = _n.nvicNonSecure[groupNo];
    int      limit  = (groupNo == 15) ? 15 : 32;
    for (int i=0; i<limit; ++i)
      if (_s.excPending[16+groupNo*32+i] && (isSecure || GETBIT(itns, i)))
        v |= BIT(i);
    return v;
  }

  int _NestStore32(phys_t addr, bool isPriv, bool isSecure, uint32_t v) {
    auto [targetNS, targetRAZWI, targetFault] = _NestAccessClassify(addr, isPriv, isSecure);

    if (targetFault)
      return -1;

    if (targetRAZWI)
      return 0;

    if (targetNS)
      _NestStore32Actual(addr |  0x2'0000U, v);
    else
      _NestStore32Actual(addr & ~0x2'0000U, v);

    return 0;
  }

  void _NestStore32Actual(phys_t addr, uint32_t v) {
    phys_t baddr = addr & ~0x2'0000U;

    switch (addr) {
      case REG_FPDSCR_S:  _n.fpdscrS  = v; break;
      case REG_FPDSCR_NS: _n.fpdscrNS = v; break;
      case REG_FPCCR_S:   _n.fpccrS   = v; break;
      case REG_FPCCR_NS:  _n.fpccrNS  = v; break;
      case REG_FPCAR_S:   _n.fpcarS   = v; break;
      case REG_FPCAR_NS:  _n.fpcarNS  = v; break;
      case REG_VTOR_S:    _n.vtorS    = v; break;
      case REG_VTOR_NS:   _n.vtorNS   = v; break;
      case REG_DEMCR:     _n.demcrS   = v; break;
      case REG_DEMCR_NS:  _n.demcrNS  = v; break;
      case REG_DHCSR:     _n.dhcsrS   = v; break;
      case REG_DHCSR_NS:  _n.dhcsrNS  = v; break;
      case REG_MMFSR_S:   _n.mmfsrS   = v; break;
      case REG_MMFSR_NS:  _n.mmfsrNS  = v; break;
      case REG_HFSR_S:    _n.hfsrS    = v; break;
      case REG_HFSR_NS:   _n.hfsrNS   = v; break;
      case REG_UFSR_S:    _n.ufsrS    = v; break;
      case REG_UFSR_NS:   _n.ufsrNS   = v; break;
      default:
        printf("Unsupported nest store 0x%08x <- 0x%08x\n", addr, v);
        abort();
    }
  }

  uint32_t InternalLoad32(phys_t addr) {
    //return _dev.InternalLoad32(addr);
    ASSERT(addr >= 0xE000'0000);
    return _NestLoad32Actual(addr);
  }
  void InternalStore32(phys_t addr, uint32_t v) {
    //_dev.InternalStore32(addr, v);
    ASSERT(addr >= 0xE000'0000);
    _NestStore32Actual(addr, v);
  }
  void InternalOr32(phys_t addr, uint32_t x) {
    InternalStore32(addr, InternalLoad32(addr) | x);
  }
  void InternalMask32(phys_t addr, uint32_t x) {
    InternalStore32(addr, InternalLoad32(addr) & ~x);
  }
  uint32_t _ThisInstrAddr() { return _s.pc; }
  bool _IsSecure() { return _HaveSecurityExt() && _s.curState == SecurityState_Secure; }
  bool _HaveMainExt() { return true; }
  bool _HaveSecurityExt() { return true; }
  void _SetThisInstrDetails(uint32_t opcode, int len, uint32_t defaultCond) {
    _s.thisInstr            = opcode;
    _s.thisInstrLength      = len;
    _s.thisInstrDefaultCond = defaultCond;
    _s.curCondOverride      = -1;
  }
  void _UpdateSecureDebugEnable();
  void _TakeReset();
  bool _VFPSmallRegisterBank() { return false; }
  bool _FPB_CheckBreakPoint(uint32_t iaddr, int size, bool isIFetch, bool isSecure);
  bool _FPB_CheckMatchAddress(uint32_t iaddr);
  bool _HaveDebugMonitor() { return _HaveMainExt(); }
  int _MaxExceptionNum() { return _HaveMainExt() ? 511 : 47; }
  uint64_t _GetD(int n) {
    assert(n >= 0 && n <= 31);
    assert(!(n >= 16 && _VFPSmallRegisterBank())); // UNDEFINED
    return _s.d[n];
  }
  void _SetD(int n, uint64_t value) {
    assert(n >= 0 && n <= 31);
    assert(!(n >= 16 && _VFPSmallRegisterBank())); // UNDEFINED
    _s.d[n] = value;
  }
  uint32_t _GetS(int n) {
    assert(n >= 0 && n <= 31);
    if (!(n%2))
      return (uint32_t)_GetD(n/2);
    else
      return (uint32_t)(_GetD(n/2)>>32);
  }
  void _SetS(int n, uint32_t value) {
    assert(n >= 0 && n <= 31);
    if (!(n%2))
      _SetD(n/2, (_GetD(n/2) & ~UINT64_C(0xFFFF'FFFF)) | uint64_t(value));
    else
      _SetD(n/2, (_GetD(n/2) & ~UINT64_C(0xFFFF'FFFF'0000'0000)) | (uint64_t(value)<<32));
  }
  void _ClearExclusiveLocal(int procID) {}
  int _ProcessorID() { return 0; }
  void _SetEventRegister() { _s.event = true; }
  void _ClearEventRegister() { _s.event = false; }
  void _InstructionSynchronizationBarrier(uint8_t option) {
    // TODO
  }
  bool _SteppingDebug();
  std::tuple<uint32_t,bool> _FetchInstr(uint32_t addr);
  bool _HaveFPB() { return true; }
  void _FPB_BreakpointMatch() {
    _GenerateDebugEventResponse();
  }
  bool _GenerateDebugEventResponse();
  ExcInfo _DefaultExcInfo() {
    ExcInfo exc = {};
    exc.fault       = NoFault;
    exc.origFault   = NoFault;
    exc.isSecure    = true;
    exc.isTerminal  = false;
    exc.inExcTaken  = false;
    exc.lockup      = false;
    exc.termInst    = true;
    return exc;
  }
  bool _HaveDWT() { return true; }
  bool _HaveFPExt() { return true; }
  bool _NoninvasiveDebugAllowed() {
    return _ExternalNoninvasiveDebugEnabled() || _HaltingDebugAllowed();
  }
  bool _HaltingDebugAllowed() {
    return _ExternalInvasiveDebugEnabled() || GETBITSM(InternalLoad32(REG_DHCSR), REG_DHCSR__S_HALT);
  }
  bool _ExternalInvasiveDebugEnabled() {
    return false; // TODO: DBGEN == HIGH
  }
  bool _ExternalNoninvasiveDebugEnabled() {
    return _ExternalInvasiveDebugEnabled() || false; // TODO: NIDEN == HIGH
  }
  bool _IsDWTEnabled() {
    return _HaveDWT() && GETBITSM(InternalLoad32(REG_DEMCR), REG_DEMCR__TRCENA) && _NoninvasiveDebugAllowed();
  }
  bool _SecureHaltingDebugAllowed() {
    if (!_HaltingDebugAllowed())
      return false;
    else if (InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__SPIDENSEL)
      return !!(InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__INTSPIDEN);
    else
      return _ExternalSecureInvasiveDebugEnabled();
  }
  bool _ExternalSecureInvasiveDebugEnabled() {
    return _ExternalInvasiveDebugEnabled() && false; // TODO: SPIDEN == HIGH
  }
  std::tuple<bool,bool> _ExceptionDetails(int exc, bool isSecure, bool isSync);
  void _HandleException(const ExcInfo &excInfo);
  void _InstructionAdvance(bool instExecOk);

  uint32_t _CurrentCond() {
    // Defined in ISA manual (ARMv8-M § C1.6.1). This is
    // based on ITSTATE for most instructions but is specially overriden for
    // branch instructions. We implement this by always getting this field from
    // ITSTATE unless this special field is set, which is to be set by our
    // branch instruction decode.
    if (_s.curCondOverride >= 0)
      return _s.curCondOverride & 0xF;

    return _s.thisInstrDefaultCond;
  }

  bool _ConditionHolds(uint32_t cond);
  void _SetMonStep(bool monStepActive);
  bool _ExceptionTargetsSecure(int excNo, bool isSecure);
  bool _SecureDebugMonitorAllowed() {
    if (InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__SPIDENSEL)
      return !!(InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__INTSPIDEN);
    else
      return _ExternalSecureSelfHostedDebugEnabled();
  }
  bool _ExternalSecureSelfHostedDebugEnabled() {
    return false && false; // DBGEN == HIGH && SPIDEN == HIGH // TODO
  }
  void _ResetSCSRegs() {
    _dev.InternalReset();
  }
  std::tuple<bool, int> _IsCPInstruction(uint32_t instr);
  void _DWT_InstructionMatch(uint32_t iaddr);
  std::tuple<bool, bool> _IsCPEnabled(int cp, bool priv, bool secure);
  std::tuple<bool, bool> _IsCPEnabled(int cp) {
    return _IsCPEnabled(cp, _CurrentModeIsPrivileged(), _IsSecure());
  }
  bool _CurrentModeIsPrivileged() {
    return _CurrentModeIsPrivileged(_IsSecure());
  }
  bool _CurrentModeIsPrivileged(bool isSecure) {
    bool npriv = isSecure ? GETBITSM(_s.controlS, CONTROL__NPRIV) : GETBITSM(_s.controlNS, CONTROL__NPRIV);
    return (_CurrentMode() == PEMode_Handler || !npriv);
  }
  PEMode _CurrentMode() {
    return GETBITSM(_s.xpsr, XPSR__EXCEPTION) == NoFault ? PEMode_Thread : PEMode_Handler;
  }

  bool _ConditionPassed() {
    return _ConditionHolds(_CurrentCond());
  }

  uint32_t _GetPC() {
    return _GetR(15);
  }

  // This function is not defined by the ISA definition and must be generated from all of the
  // instruction definitions. Our actual implementation is in _DecodeExecuteActual
  // and wrapped by this.
#define UNDEFINED_DEC() do { throw Exception(ExceptionType::UNDEFINED); } while (0)
#define TODO_DEC()      do { printf("warning: todo insn on line %u\n", __LINE__); UNDEFINED_DEC(); } while (0)
  // For CONSTRAINED UNPREDICTABLE which we choose to implement as UNDEFINED
#define CUNPREDICTABLE_UNDEFINED() UNDEFINED_DEC()
#define CUNPREDICTABLE_UNALIGNED() do { _ThrowUnaligned(); } while (0)

  // Custom function not corresponding to the ISA manual. Throws unaligned
  // usage fault. For use implementing UNPREDICTABLE where a permitted
  // implementation is to raise an UNALIGNED UsageFault.
  void _ThrowUnaligned() {
    InternalOr32(REG_UFSR, REG_UFSR__UNALIGNED);
    auto excInfo = _CreateException(UsageFault, false, false/*UNKNOWN*/);
    _HandleException(excInfo);
  }

  // Exposition only
  static uint32_t _ZeroExtend(uint32_t v, uint32_t w) {
    return v;
  }

  static uint32_t _Align(uint32_t x, uint32_t align) {
    return x & ~(align-1);
  }

  void _BranchWritePC(uint32_t address) {
    _BranchTo(address & ~BIT(0));
  }

  void _ALUWritePC(uint32_t address) {
    _BranchWritePC(address);
  }

  bool _InITBlock() {
    return !!(_GetITSTATE() & BITS(0,3));
  }

  bool _LastInITBlock() {
    return GETBITS(_GetITSTATE(),0,3) == 0b1000;
  }

  static std::tuple<uint32_t, bool> _LSL_C(uint32_t x, int shift) {
    ASSERT(shift > 0);

    if (shift == 32)
      return {0, !!(x & BIT(31))};

    if (shift > 32)
      return {0, false};

    uint32_t result   = x<<shift;
    bool     carryOut = !!(x & BIT(32-shift));

    return {result, carryOut};
  }

  static std::tuple<uint32_t, bool> _LSR_C(uint32_t x, int shift) {
    ASSERT(shift > 0);

    uint32_t result   = x>>shift;
    bool     carryOut = !!(x & BIT(shift-1));

    return {result, carryOut};
  }

  static std::tuple<uint32_t, bool> _ASR_C(uint32_t x, int shift) {
    ASSERT(shift > 0);

    int32_t xs = (int32_t)x;

    uint32_t result   = (uint32_t)(xs>>shift);
    bool     carryOut = !!(x & BIT(shift-1));

    return {result, carryOut};
  }

  static uint32_t _LSL(uint32_t x, int shift) {
    ASSERT(shift >= 0);
    if (!shift)
      return x;

    auto [result, _] = _LSL_C(x, shift);
    return result;
  }

  static uint32_t _LSR(uint32_t x, int shift) {
    ASSERT(shift >= 0);
    if (!shift)
      return x;

    auto [result, _] = _LSR_C(x, shift);
    return result;
  }

  static std::tuple<uint32_t, bool> _ROR_C(uint32_t x, int shift) {
    ASSERT(shift);

    uint32_t m        = shift % 32;
    uint32_t result   = _LSR(x, m) | _LSL(x, 32-m);
    bool     carryOut = !!(result & BIT(31));

    return {result, carryOut};
  }

  static std::tuple<uint32_t, bool> _RRX_C(uint32_t x, bool carryIn) {
    uint32_t result   = (carryIn ? BIT(31) : 0) | (x>>1);
    bool    carryOut  = !!(x & BIT(0));

    return {result, carryOut};
  }

  static std::tuple<uint32_t, bool> _Shift_C(uint32_t value, SRType srType, int amount, bool carryIn) {
    ASSERT(!(srType == SRType_RRX && amount != 1));

    if (!amount)
      return {value, carryIn};

    switch (srType) {
      case SRType_LSL:
        return _LSL_C(value, amount);
      case SRType_LSR:
        return _LSR_C(value, amount);
      case SRType_ASR:
        return _ASR_C(value, amount);
      case SRType_ROR:
        return _ROR_C(value, amount);
      case SRType_RRX:
        return _RRX_C(value, carryIn);
      default:
        abort();
    }
  }

  static bool _IsZero(uint32_t x) {
    return !x;
  }

  static bool _IsZeroBit(uint32_t x) {
    return _IsZero(x);
  }

  RName _LookUpRName(int n) {
    ASSERT(n >= 0 && n <= 15);
    switch (n) {
      case  0: return RName0;
      case  1: return RName1;
      case  2: return RName2;
      case  3: return RName3;
      case  4: return RName4;
      case  5: return RName5;
      case  6: return RName6;
      case  7: return RName7;
      case  8: return RName8;
      case  9: return RName9;
      case 10: return RName10;
      case 11: return RName11;
      case 12: return RName12;
      case 13: return _LookUpSP();
      case 14: return RName_LR;
      case 15: return RName_PC;
      default: abort();
    }
  }

  void _BranchToNS(uint32_t addr) {
    ASSERT(_HaveSecurityExt() && _IsSecure());

    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__T, 1);
    if (!(addr & BIT(0))) {
      _s.curState = SecurityState_NonSecure;
      if (_HaveFPExt())
        _s.controlS = CHGBITSM(_s.controlS, CONTROL__SFPA, 0);
    }

    _BranchTo(addr & ~BIT(0));
  }

  ExcInfo _FunctionReturn() {
    auto exc = _DefaultExcInfo();

    // Pull the return address and IPSR off the Secure stack
    PEMode    mode      = _CurrentMode();
    RName     spName    = _LookUpSP_with_security_mode(true, mode);
    uint32_t  framePtr  = _GetSP(spName);

    if (!_IsAligned(framePtr, 8))
      throw Exception(ExceptionType::UNPREDICTABLE);

    // Only stack locations, not the load order are architected
    uint32_t newPSR, newPC;
    if (exc.fault == NoFault) std::tie(exc, newPSR) = _Stack(framePtr, 4, spName, mode);
    if (exc.fault == NoFault) std::tie(exc, newPC)  = _Stack(framePtr, 0, spName, mode);

    // Check the IPSR value that has been unstacked is consistent with the current mode,
    // and being originally called from the Secure state.
    //
    // NOTE: It is IMPLEMENTATION DEFINED whether this check is performed before or after
    // the load of the return address above.
    if (   exc.fault == NoFault
        && !(   ( GETBITSM(_s.xpsr, XPSR__EXCEPTION) == 0 && GETBITSM(newPSR, RETPSR__EXCEPTION) == 0)
             || ( GETBITSM(_s.xpsr, XPSR__EXCEPTION) == 1 && GETBITSM(newPSR, RETPSR__EXCEPTION) != 0))) {
      if (_HaveMainExt())
        InternalOr32(REG_UFSR, REG_UFSR__INVPC);

      // Create the exception. NOTE: If Main Extension not implemented then the fault
      // always escalates to a HardFault.
      exc = _CreateException(UsageFault, true, true);
    }

    // The IPSR value is set as UNKNOWN if the IPSR value is not supported by the PE.
    uint32_t excNum = GETBITSM(newPSR, XPSR__EXCEPTION);

    bool validIPSR = (excNum == 0
                   || excNum == 1
                   || excNum == NMI
                   || excNum == HardFault
                   || excNum == SVCall
                   || excNum == PendSV
                   || excNum == SysTick);
    if (!validIPSR && _HaveMainExt())
      validIPSR = (excNum == MemManage
                || excNum == BusFault
                || excNum == UsageFault
                || excNum == SecureFault
                || excNum == DebugMonitor);

    if (!validIPSR && !_IsIrqValid(excNum))
      newPSR = CHGBITSM(newPSR, RETPSR__EXCEPTION, UNKNOWN_VAL(0));

    // Only consume the function return stack frame and update the XPSR/PC if no
    // faults occurred.
    if (exc.fault == NoFault) {
      // Transition to the Secure state
      _s.curState = SecurityState_Secure;
      // Update stack pointer. NOTE: Stack pointer limit not checked on function
      // return as stack pointer guaranteed to be ascending not descending.
      _s.r[spName]  = framePtr + 8;
      _s.xpsr       = CHGBITSM(_s.xpsr, XPSR__EXCEPTION, GETBITSM(newPSR, RETPSR__EXCEPTION));
      _s.controlS   = CHGBITSM(_s.controlS, CONTROL__SFPA, GETBITSM(newPSR, RETPSR__SFPA));
      // IT/ICI bits cleared to prevent non-secure code interfering with secure
      // execution.
      if (_HaveMainExt())
        _SetITSTATE(0);

      // If EPSR.T == 0, a UsageFault('Invalid State') or a HardFault is taken
      // on the next instruction depending on whether the Main Extension is
      // implemented or not.
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__T, newPC & BIT(0));
      _BranchTo(newPC & ~BIT(0));
    }

    return exc;
  }

  ExcInfo _BXWritePC(uint32_t addr, bool allowNonSecure) {
    auto exc = _DefaultExcInfo();

    if (_HaveSecurityExt() && (addr & ~BIT(0)) == 0b1111'1110'1111'1111'1111'1111'1111'1110) {
      // Unlike exception return, any faults raised during a FNC_RETURN
      // unstacking are raised synchronously with the instruction that
      // triggered the unstacking.
      exc = _FunctionReturn();
    } else if (_CurrentMode() == PEMode_Handler && GETBITS(addr,24,31) == 0xFF) {
      // The actual exception return is performed when the current instruction
      // completes. This is because faults that occur during the exception
      // return are handled differently from faults raised during the
      // instruction execution.
      _PendReturnOperation(addr);
    } else if (_HaveSecurityExt() && _IsSecure() && allowNonSecure) {
      // If in the Secure state and transitions to the non-secure state are allowed
      // then the target state is specified by the LSB of the target address.
      _BranchToNS(addr);
    } else {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__T, addr & 1);
      // If EPSR.T == 0 then an exception is taken on the next instruction:
      // UsageFault('Invalid State') if the Main Extension is implemented;
      // HardFault otherwise
      _BranchTo(addr & ~BIT(0));
    }

    return exc;
  }

  void _LoadWritePC(uint32_t addr, int baseReg, uint32_t baseRegVal, bool baseRegUpdate, bool spLimCheck) {
    RName    regName;
    uint32_t oldBaseVal;

    if (baseRegUpdate) {
      regName     = _LookUpRName(baseReg);
      oldBaseVal  = _GetR(baseReg);
      if (spLimCheck)
        _SetRSPCheck(baseReg, baseRegVal);
      else
        _SetR(baseReg, baseRegVal);
    }

    // Attempt to update the PC, which may result in a fault
    auto excInfo = _BXWritePC(addr, false);

    if (baseRegUpdate && excInfo.fault != NoFault)
      // Restore the previous base reg value, SP limit checking is not performed
      _s.r[regName] = oldBaseVal; // _R

    _HandleException(excInfo);
  }

  uint32_t _GetPRIMASK() {
    return _IsSecure() ? _s.primaskS : _s.primaskNS;
  }
  void _SetPRIMASK(uint32_t v) {
    if (_IsSecure())
      _s.primaskS = v;
    else
      _s.primaskNS = v;
  }

  uint32_t _GetFAULTMASK() {
    return _IsSecure() ? _s.faultmaskS : _s.faultmaskNS;
  }
  void _SetFAULTMASK(uint32_t v) {
    if (_IsSecure())
      _s.faultmaskS = v;
    else
      _s.faultmaskNS = v;
  }

  std::tuple<uint32_t, bool, bool> _AddWithCarry(uint32_t x, uint32_t y, bool carryIn) {
    uint32_t unsignedSum;
    int32_t  signedSum;
    bool     carryOut, overflow;

    carryOut = __builtin_add_overflow(x, y, &unsignedSum);
    carryOut = carryOut || __builtin_add_overflow(unsignedSum, (uint32_t)carryIn, &unsignedSum);

    overflow = __builtin_add_overflow((int32_t)x, (int32_t)y, &signedSum);
    overflow = overflow || __builtin_add_overflow(signedSum, (int32_t)carryIn, &signedSum);

    return {unsignedSum, carryOut, overflow};
  }

  // Corresponds to SignExtend(), but has an additional parameter to represent input width.
  uint32_t _SignExtend(uint32_t x, uint32_t inWidth, uint32_t outWidth) {
    if (x & BIT(inWidth-1)) {
      return x | BITS(inWidth,outWidth-1);
    } else
      return x;
  }

  void _DecodeExecute(uint32_t instr, uint32_t pc, bool is16bit) {
    if (is16bit)
      _DecodeExecute16(instr, pc);
    else
      _DecodeExecute32(instr, pc);
  }

  void _DecodeExecute16_1010xx_0(uint32_t instr, uint32_t pc) {
    // ADR § C2.4.8 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t d    = Rd, imm32 = _ZeroExtend(imm8<<2, 32);
    bool     add  = true;

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = add ? _Align(_GetPC(),4) + imm32 : _Align(_GetPC(),4) - imm32;
    _SetR(d, result);
  }

  void _DecodeExecute16_1010xx_1(uint32_t instr, uint32_t pc) {
    TODO_DEC();
  }

  void _DecodeExecute16_1010xx(uint32_t instr, uint32_t pc) {
    if (!(instr & BIT(16+11)))
      // ADR
      _DecodeExecute16_1010xx_0(instr, pc);
    else
      // ADD (SP plus immediate)
      _DecodeExecute16_1010xx_1(instr, pc);
  }

  void _DecodeExecute16_010001_10(uint32_t instr, uint32_t pc) {
    // MOV (register) § C2.4.90 T1
    // ---- DECODE --------------------------------------------------
    uint32_t D  = GETBITS(instr, 7, 7);
    uint32_t Rm = GETBITS(instr, 3, 6);
    uint32_t Rd = GETBITS(instr, 0, 2);

    uint32_t  d         = (D<<3) | Rd;
    uint32_t  m         = Rm;
    bool      setflags  = false;
    SRType    shiftT    = SRType_LSL;
    uint32_t  shiftN    = 0;

    if (_HaveMainExt())
      if (d == 15 && _InITBlock() && !_LastInITBlock())
        throw Exception(ExceptionType::UNPREDICTABLE);

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 15)
      _ALUWritePC(result); // setflags is always FALSE here
    else {
      _SetRSPCheck(d, result);
      if (setflags) {
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, !!(result & BIT(31)));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      }
    }
  }

  void _DecodeExecute16_010001_xx(uint32_t instr, uint32_t pc) {
    uint32_t op   = GETBITS(instr, 8, 9);
    uint32_t D    = GETBITS(instr, 7, 7);
    uint32_t Rs   = GETBITS(instr, 3, 6);
    uint32_t Rd   = GETBITS(instr, 0, 2);
    uint32_t DRd  = (D<<3) | Rd;

    switch (op) {
      case 0b00:
        if (Rs == 0b1101) {
          // ADD (SP plus register) - T1
          TODO_DEC();
        } else if (DRd == 0b1101) {
          // ADD (SP plus register) — T2
          TODO_DEC();
        } else {
          // ADD (register)
          TODO_DEC();
        }
        break;

      case 0b01:
        // CMP (register)
        TODO_DEC();
        break;

      case 0b10:
        // MOV (register)
        _DecodeExecute16_010001_10(instr, pc);
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute16_010001(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr, 8, 9);
    switch (op0) {
      case 0b11:
        // Branch and exchange
        TODO_DEC();
        break;

      default:
        // Add, subtract, compare, move (two high registers)
        _DecodeExecute16_010001_xx(instr, pc);
        break;
    }
  }

  void _DecodeExecute16_01001x(uint32_t instr, uint32_t pc) {
    // LDR (literal) § C2.4.53 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t t     = Rt;
    uint32_t imm32 = _ZeroExtend(imm8<<2, 32);
    bool     add   = true;

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t base     = _Align(_GetPC(), 4);
    uint32_t address  = add ? base + imm32 : base - imm32;
    uint32_t data     = _MemU(address, 4);
    if (t == 15) {
      if (GETBITS(address, 0, 1) == 0b00)
        _LoadWritePC(data, 0, 0, false, false);
      else
        throw Exception(ExceptionType::UNPREDICTABLE);
    } else
      _SetR(t, data);
  }

  void _DecodeExecute16_01100x(uint32_t instr, uint32_t pc) {
    // STR (immediate) § C2.4.183 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm5 = GETBITS(instr, 6,10);
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rt   = GETBITS(instr, 0, 2);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm5<<2, 32);
    bool index = true, add = true, wback = false;

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit should be checked
    uint32_t  limit;
    bool      applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated
    if (!applyLimit || offsetAddr >= limit)
      _MemU(addr, 4, _GetR(t));

    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  void _DecodeExecute16_01101x(uint32_t instr, uint32_t pc) {
    // LDR (immediate) § C2.4.52 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm5 = GETBITS(instr, 6,10);
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rt   = GETBITS(instr, 0, 2);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm5<<2, 32);
    bool index = true, add = true, wback = false;

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated
    uint32_t data = 0;
    if (!applyLimit || offsetAddr >= limit)
      data = _MemU(addr, 4);

    // If the stack pointer is being updated a fault will be
    // raised if the limit is violated
    if (t == 15) {
      if (GETBITS(addr, 0, 1) == 0b00)
        _LoadWritePC(data, n, offsetAddr, wback, true);
      else
        CUNPREDICTABLE_UNALIGNED();
    } else {
      if (wback)
        _SetRSPCheck(n, offsetAddr);

      _SetR(t, data);
    }
  }

  void _DecodeExecute16_011xxx(uint32_t instr, uint32_t pc) {
    uint32_t B = GETBITS(instr,12,12);
    uint32_t L = GETBITS(instr,11,11);

    switch ((B<<1)|L) {
      case 0b00:
        // STR (immediate)
        _DecodeExecute16_01100x(instr, pc);
        break;

      case 0b01:
        // LDR (immediate)
        _DecodeExecute16_01101x(instr, pc);
        break;

      case 0b10:
        // STRB (immediate)
        TODO_DEC();
        break;

      case 0b11:
        // LDRB (immediate)
        TODO_DEC();
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute16_101101_10_01_1(uint32_t instr, uint32_t pc) {
    // CPS § C2.4.32 T1
    // ---- DECODE --------------------------------------------------
    uint32_t im = GETBITS(instr, 4, 4);
    uint32_t I  = GETBITS(instr, 1, 1);
    uint32_t F  = GETBITS(instr, 0, 0);

    bool enable   = !im;
    bool disable  = !!im;
    if (_InITBlock())
      throw Exception(ExceptionType::UNPREDICTABLE);

    if (!I && !F)
      CUNPREDICTABLE_UNDEFINED();

    bool affectPRI    = !!I;
    bool affectFAULT  = !!F;
    if (!_HaveMainExt()) {
      if (!I)
        CUNPREDICTABLE_UNDEFINED();
      if (F)
        CUNPREDICTABLE_UNDEFINED();
    }

    // ---- EXECUTE -------------------------------------------------
    //EncodingSpecificOperations
    if (_CurrentModeIsPrivileged()) {
      if (enable) {
        if (affectPRI)
          _SetPRIMASK(CHGBITSM(_GetPRIMASK(), PRIMASK__PM, 0));
        if (affectFAULT)
          _SetFAULTMASK(CHGBITSM(_GetFAULTMASK(), FAULTMASK__FM, 0));
      }
      if (disable) {
        if (affectPRI)
          _SetPRIMASK(CHGBITSM(_GetPRIMASK(), PRIMASK__PM, 1));
        if (affectFAULT)
          _SetFAULTMASK(CHGBITSM(_GetFAULTMASK(), FAULTMASK__FM, 1));
      }
    }
  }

  void _DecodeExecute16_1011xx(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr, 8,11);
    uint32_t op1 = GETBITS(instr, 6, 7);
    uint32_t op2 = GETBITS(instr, 5, 5);
    uint32_t op3 = GETBITS(instr, 0, 3);

    switch (op0) {
      case 0b0000:
        // Adjust SP (immediate)
        TODO_DEC();
        break;

      case 0b0010:
        // Extend
        TODO_DEC();
        break;

      case 0b0110:
        if (op1 == 0b01 && op2) {
          // CPS
          _DecodeExecute16_101101_10_01_1(instr, pc);
        } else {
          // Unallocated
          UNDEFINED_DEC();
        }
        break;

      case 0b0111:
      case 0b1000:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b1010:
        if (op1 == 0b10) {
          // Unallocated
          UNDEFINED_DEC();
        } else {
          // Reverse bytes
          TODO_DEC();
        }
        break;

      case 0b1110:
        // BKPT
        TODO_DEC();
        break;

      case 0b1111:
        // Hints, IT
        TODO_DEC();
        break;

      case 0b0001:
      case 0b0011:
      case 0b1001:
      case 0b1011:
        // CBNZ, CBZ
        TODO_DEC();
        break;

      case 0b0100:
      case 0b0101:
      case 0b1100:
      case 0b1101:
        // Push and Pop
        TODO_DEC();
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute16_00101x(uint32_t instr, uint32_t pc) {
    // CMP (immediate) § C2.4.30 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), ~imm32, true);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
  }

  void _DecodeExecute16_001xxx(uint32_t instr, uint32_t pc) {
    uint32_t op   = GETBITS(instr,11,12);
    uint32_t Rd   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    switch (op) {
      case 0b00:
        // MOV (immediate)
        TODO_DEC();
        break;

      case 0b01:
        // CMP (immediate)
        _DecodeExecute16_00101x(instr, pc);
        break;

      case 0b10:
        // ADD (immediate)
        TODO_DEC();
        break;

      case 0b11:
        // SUB (immediate)
        TODO_DEC();
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute16_00xxxx(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr,13,13);
    uint32_t op1 = GETBITS(instr,11,12);
    uint32_t op2 = GETBITS(instr,10,10);

    switch ((op0<<2)|op1) {
      case 0b0'11:
        if (!op2) {
          // Add, subtract (three low registers)
          TODO_DEC();
        } else {
          // Add, ubstract (two low registers and immediate)
          TODO_DEC();
        }
        break;

      case 0b0'00:
      case 0b0'01:
      case 0b0'10:
        // MOV (register) — T2 variant
        TODO_DEC();
        break;

      case 0b1'00:
      case 0b1'01:
      case 0b1'10:
      case 0b1'11:
        // Add, subtract, compare, move (one low register and immediate)
        _DecodeExecute16_001xxx(instr, pc);
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute16_1101xx_xx(uint32_t instr, uint32_t pc) {
    // B § C2.4.15 T1
    // ---- DECODE --------------------------------------------------
    uint32_t cond = GETBITS(instr, 8,11);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    ASSERT (cond != 0b1110 && cond != 0b1111);

    uint32_t imm32 = _SignExtend(imm8<<1, 9, 32);

    if (_InITBlock())
      throw Exception(ExceptionType::UNPREDICTABLE);

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _BranchWritePC(_GetPC() + imm32);
  }

  void _DecodeExecute16_1101xx(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr, 8,11);

    switch (op0) {
      case 0b1110:
      case 0b1111:
        // Exception generation
        TODO_DEC();
        break;

      default:
        // B - T1 variant
        _DecodeExecute16_1101xx_xx(instr, pc);
        break;
    }
  }

  void _DecodeExecute16(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr,10,15);
    switch (op0) {
      case 0b00'0000:
      case 0b00'0001:
      case 0b00'0010:
      case 0b00'0011:
      case 0b00'0100:
      case 0b00'0101:
      case 0b00'0110:
      case 0b00'0111:
      case 0b00'1000:
      case 0b00'1001:
      case 0b00'1010:
      case 0b00'1011:
      case 0b00'1100:
      case 0b00'1101:
      case 0b00'1110:
      case 0b00'1111:
        // Shift (immediate), add, subtract, move and compare
        _DecodeExecute16_00xxxx(instr, pc);
        break;

      case 0b01'0000:
        // Data processing (two low registers)
        TODO_DEC();
        break;

      case 0b01'0001:
        // Special data instructions and branch and exchange
        _DecodeExecute16_010001(instr, pc);
        break;

      case 0b01'0010:
      case 0b01'0011:
        // LDR (literal) - T1 variant
        _DecodeExecute16_01001x(instr, pc);
        break;

      case 0b01'0100:
      case 0b01'0101:
      case 0b01'0110:
      case 0b01'0111:
        // Load/store (register offset)
        TODO_DEC();
        break;

      case 0b01'1000:
      case 0b01'1001:
      case 0b01'1010:
      case 0b01'1011:
      case 0b01'1100:
      case 0b01'1101:
      case 0b01'1110:
      case 0b01'1111:
        // Load/store word/byte (immediate offset)
        _DecodeExecute16_011xxx(instr, pc);
        break;

      case 0b10'0000:
      case 0b10'0001:
      case 0b10'0010:
      case 0b10'0011:
        // Load/store halfword (immediate offset)
        TODO_DEC();
        break;

      case 0b10'0100:
      case 0b10'0101:
      case 0b10'0110:
      case 0b10'0111:
        // Load/store (SP-relative)
        TODO_DEC();
        break;

      case 0b10'1000:
      case 0b10'1001:
      case 0b10'1010:
      case 0b10'1011:
        // Add PC/SP (immediate)
        _DecodeExecute16_1010xx(instr, pc);
        break;

      case 0b10'1100:
      case 0b10'1101:
      case 0b10'1110:
      case 0b10'1111:
        // Miscellaneous 16-bit instructions
        _DecodeExecute16_1011xx(instr, pc);
        break;

      case 0b11'0000:
      case 0b11'0001:
      case 0b11'0010:
      case 0b11'0011:
        // Load/store multiple
        TODO_DEC();
        break;

      case 0b11'0100:
      case 0b11'0101:
      case 0b11'0110:
      case 0b11'0111:
        // Conditional branch, and supervisor call
        _DecodeExecute16_1101xx(instr, pc);
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute32_0100_011_LS_STRD(uint32_t instr, uint32_t pc) {
    TODO_DEC();
  }

  void _DecodeExecute32_0100_011_LS_LDRD(uint32_t instr, uint32_t pc) {
    // LDRD (immediate) § C2.4.59 T1
    // ---- DECODE --------------------------------------------------
    uint32_t P    = GETBITS(instr>>16, 8, 8);
    uint32_t U    = GETBITS(instr>>16, 7, 7);
    uint32_t W    = GETBITS(instr>>16, 5, 5);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t Rt2  = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!P && !W) {
      // RELATED ENCODINGS
    }

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      throw Exception(ExceptionType::UNDEFINED);

    uint32_t t = Rt, t2 = Rt2, n = Rn, imm32 = _ZeroExtend(imm8<<2, 32);
    bool index = !!P, add = !!U, wback = !!W;
    if (wback && (n == t || n == t2))
      CUNPREDICTABLE_UNDEFINED();

    if ((t == 13 || t == 15) || (t2 == 13 || t2 == 15) || t == t2)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t address    = index ? offsetAddr : _GetR(n);

    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    if (!applyLimit || offsetAddr >= limit) {
      _SetR(t,  _MemA(address,   4));
      _SetR(t2, _MemA(address+4, 4));
    }

    if (wback)
      _SetRSPCheck(n, offsetAddr);

    // TODO untested
  }

  void _DecodeExecute32_0100_011_LS(uint32_t instr, uint32_t pc) {
    if (instr & BIT(16+4))
      // STRD (immediate)
      return _DecodeExecute32_0100_011_LS_STRD(instr, pc);
    else
      // LDRD (immediate)
      return _DecodeExecute32_0100_011_LS_LDRD(instr, pc);
  }

  void _DecodeExecute32_0100_011(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr>>16, 0, 3);
    switch (op0) {
      case 0b1111:
        throw Exception(ExceptionType::UNPREDICTABLE);

      default:
        // Load/store dual (immediate, post-indexed)
        return _DecodeExecute32_0100_011_LS(instr, pc);
    }
  }

  static uint32_t _BitCount(uint32_t x) {
    return __builtin_popcount(x);
  }

  static std::tuple<uint32_t, bool> _T32ExpandImm_C(uint32_t imm12, bool carryIn) {
    if (GETBITS(imm12,10,11) == 0b00) {
      uint32_t imm32;
      switch (GETBITS(imm12, 8, 9)) {
        case 0b00:
          imm32 = _ZeroExtend(GETBITS(imm12, 0, 7), 32);
          break;
        case 0b01:
          if (GETBITS(imm12, 0, 7) == 0)
            throw Exception(ExceptionType::UNPREDICTABLE);
          imm32 = (GETBITS(imm12, 0, 7)<<16) | GETBITS(imm12, 0, 7);
          break;
        case 0b10:
          if (GETBITS(imm12, 0, 7) == 0)
            throw Exception(ExceptionType::UNPREDICTABLE);
          imm32 = (GETBITS(imm12, 0, 7)<<24) | (GETBITS(imm12, 0, 7)<<8);
          break;
        case 0b11:
          if (GETBITS(imm12, 0, 7) == 0)
            throw Exception(ExceptionType::UNPREDICTABLE);
          imm32 = (GETBITS(imm12, 0, 7)<<24) | (GETBITS(imm12, 0, 7)<<16)
            | (GETBITS(imm12, 0, 7)<<8) | GETBITS(imm12, 0, 7);
          break;
      }
      return {imm32, carryIn};
    } else {
      uint32_t unrotatedValue = _ZeroExtend(BIT(7) | GETBITS(imm12, 0, 6), 32);
      return _ROR_C(unrotatedValue, GETBITS(imm12, 7, 11));
    }
  }

  void _DecodeExecute32_0100_10x_00(uint32_t instr, uint32_t pc) {
    // STMDB, STMFD § C2.4.182 T1
    // ---- DECODE --------------------------------------------------
    uint32_t W        = GETBITS(instr>>16, 5, 5);
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t M        = GETBITS(instr    ,14,14);
    uint32_t regList  = GETBITS(instr    , 0,12);

    if (!_HaveMainExt())
      throw Exception(ExceptionType::UNDEFINED);

    uint32_t  n         = Rn;
    uint32_t  registers = regList | (M<<14);
    bool      wback     = !!W;

    if (n == 15 || _BitCount(registers) < 2)
      CUNPREDICTABLE_UNDEFINED();

    if (wback && !!(registers & BIT(n)))
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t address = _GetR(n) - 4*_BitCount(registers);
    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    for (int i=0; i<15; ++i) {
      if (!!(registers & BIT(i)) && (!applyLimit || address >= limit)) {
        _MemA(address, 4, _GetR(i));
        address += 4;
      }
    }

    if (wback)
      _SetRSPCheck(n, _GetR(n) - 4*_BitCount(registers));
  }

  void _DecodeExecute32_0100_x0x(uint32_t instr, uint32_t pc) {
    uint32_t opc = GETBITS(instr>>16, 7, 8);
    uint32_t L   = GETBITS(instr>>16, 4, 4);
    switch ((opc<<1)|L) {
      case 0b00'0:
      case 0b00'1:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b01'0:
        // STM, STMIA, STMEA
        TODO_DEC();
        break;

      case 0b01'1:
        // LDM, LDMIA, LDMFD
        TODO_DEC();
        break;

      case 0b10'0:
        // STMDB, STMFD
        _DecodeExecute32_0100_10x_00(instr, pc);
        break;

      case 0b10'1:
        // LDMDB, LDMEA
        TODO_DEC();
        break;

      case 0b11'0:
      case 0b11'1:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute32_0100(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr>>16, 8, 8);
    uint32_t op1 = GETBITS(instr>>16, 5, 6);
    switch ((op0<<2)|op1) {
      case 0b0'00:
      case 0b0'01:
      case 0b1'00:
      case 0b1'01:
        // Load/store multiple
        _DecodeExecute32_0100_x0x(instr, pc);
        break;

      case 0b0'10:
        // Load/store exclusive, load-acquire/store-release, table branch
        TODO_DEC();
        break;

      case 0b0'11:
        // Load/store dual (post-indexed)
        _DecodeExecute32_0100_011(instr, pc);
        break;

      case 0b1'10:
        // Load/store dual (literal and immediate)
        TODO_DEC();
        break;

      case 0b1'11:
        // Load/store dual (pre-indexed), secure gateway
        TODO_DEC();
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute32_1001_0_1_011_1111(uint32_t instr, uint32_t pc) {
    // BFC § C2.4.16 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm3 = GETBITS(instr,12,14);
    uint32_t Rd   = GETBITS(instr, 8,11);
    uint32_t imm2 = GETBITS(instr, 6, 7);
    uint32_t msb  = GETBITS(instr, 0, 4);

    if (!_HaveMainExt())
      throw Exception(ExceptionType::UNDEFINED);

    uint32_t d      = Rd;
    uint32_t msbit  = msb;
    uint32_t lsbit  = (imm3<<2)|imm2;

    if (msbit < lsbit)
      CUNPREDICTABLE_UNDEFINED();

    if (d == 13 || d == 15)
      throw Exception(ExceptionType::UNPREDICTABLE);

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    if (msbit >= lsbit) {
      _SetR(d, CHGBITS(_GetR(d), lsbit, msbit, 0));
    } else
      _SetR(d, UNKNOWN_VAL(0));
  }

  void _DecodeExecute32_10x1_0_1(uint32_t instr, uint32_t pc) {
    uint32_t op1  = GETBITS(instr>>16, 5, 7);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t imm3_imm2 = (imm3<<2) | imm2;

    switch (op1) {
      case 0b000:
        // SSAT - Logical shift left variant
        TODO_DEC();
        break;

      case 0b001:
        if (imm3_imm2) {
          // SSAT - Arithmetic shift right variant
          TODO_DEC();
        } else {
          // SSAT16
          TODO_DEC();
        }
        break;

      case 0b010:
        // SBFX
        TODO_DEC();
        break;

      case 0b011:
        if (Rn != 0b1111) {
          // BFI
          TODO_DEC();
        } else {
          // BFC
          _DecodeExecute32_1001_0_1_011_1111(instr, pc);
        }
        break;

      case 0b100:
        // USAT - Logical shift left variant
        TODO_DEC();
        break;

      case 0b101:
        if (imm3_imm2) {
          // USAT - Arithmetic shift right variant
          TODO_DEC();
        } else {
          // USAT16
          TODO_DEC();
        }
        break;

      case 0b110:
        // UBFX
        TODO_DEC();
        break;

      case 0b111:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute32_10x1_0_010_0(uint32_t instr, uint32_t pc) {
    // MOV (immediate) § C2.4.89 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!S) { // MOV variant

    } else { // MOVS variant

    }

    if (!_HaveMainExt())
      throw Exception(ExceptionType::UNDEFINED);

    uint32_t d        = Rd;
    bool     setflags = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 13 || d == 15)
      throw Exception(ExceptionType::UNPREDICTABLE);

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = imm32;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  void _DecodeExecute32_10x1_0_010(uint32_t instr, uint32_t pc) {
    uint32_t o1 = GETBITS(instr>>16, 7, 7);
    if (!o1) {
      // MOV (immediate)
      _DecodeExecute32_10x1_0_010_0(instr, pc);
    } else {
      // MOVT
      TODO_DEC();
    }
  }

  void _DecodeExecute32_10x1_0(uint32_t instr, uint32_t pc) {
    uint32_t op0 = GETBITS(instr>>16, 8, 8);
    uint32_t op1 = GETBITS(instr>>16, 5, 6);

    switch ((op0<<2)|op1) {
      case 0b0'00:
      case 0b0'01:
        // Data processing (simple immediate)
        TODO_DEC();
        break;

      case 0b0'10:
        // Move Wide (16-bit immediate)
        _DecodeExecute32_10x1_0_010(instr, pc);
        break;

      case 0b0'11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b1'00:
      case 0b1'01:
      case 0b1'10:
      case 0b1'11:
        // Saturate, Bitfield
        _DecodeExecute32_10x1_0_1(instr, pc);
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute32_10x0_0_0000_0(uint32_t instr, uint32_t pc) {
    // AND (immediate) § C2.4.9 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr    , 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rd == 0b1111 && S));
    if (!_HaveMainExt())
      throw Exception(ExceptionType::UNDEFINED);

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    bool      setflags  = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 13 || (d == 15 && !S) || (n == 13 || n == 15))
      throw Exception(ExceptionType::UNPREDICTABLE);

    // ---- EXECUTE -------------------------------------------------
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _GetR(n) & imm32;
    _SetR(d, result);

    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  void _DecodeExecute32_10x0_0(uint32_t instr, uint32_t pc) {
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t op1  = GETBITS(instr>>16, 5, 8);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    switch (op1) {
      case 0b0000:
        if (!S) {
          // AND (immediate) — AND variant
          _DecodeExecute32_10x0_0_0000_0(instr, pc);
        } else if (Rd != 15) {
          // AND (immediate) — ANDS variant
          TODO_DEC();
        } else {
          // TST (immediate)
          TODO_DEC();
        }
        break;

      case 0b0001:
        // BIC (immediate)
        TODO_DEC();
        break;

      case 0b0010:
        if (!S) {
          if (Rd != 15) {
            // ORR (immediate) — ORR variant
            TODO_DEC();
          } else {
            // MOV (immediate) — MOV variant
            TODO_DEC();
          }
        } else {
          if (Rd != 15) {
            // ORR (immediate) — ORRS variant
            TODO_DEC();
          } else {
            // MOV (immediate) — MOVS variant
            TODO_DEC();
          }
        }
        break;

      case 0b0011:
        if (!S) {
          if (Rd != 15) {
            // ORN (immediate) — Non flag setting variant
            TODO_DEC();
          } else {
            // MVN (immediate) — MVN variant
            TODO_DEC();
          }
        } else {
          if (Rd != 15) {
            // ORN (immediate) — Flag setting variant
            TODO_DEC();
          } else {
            // MVN (immediate) — MVNS variant
            TODO_DEC();
          }
        }
        break;

      case 0b0100:
        if (!S) {
          // EOR (immediate) — EOR variant
          TODO_DEC();
        } else {
          if (Rd != 15) {
            // EOR (immediate) — EORS variant
            TODO_DEC();
          } else {
            // TEQ (immediate)
            TODO_DEC();
          }
        }
        break;

      case 0b0101:
      case 0b0110:
      case 0b0111:
      case 0b1001:
      case 0b1100:
      case 0b1111:
        UNDEFINED_DEC();
        break;

      case 0b1000:
        if (!S) {
          if (Rn != 0b1101) {
            // ADD (immediate) — ADD variant
            TODO_DEC();
          } else {
            // ADD (SP plus immediate) — ADD variant
            TODO_DEC();
          }
        } else if (Rd == 15) {
          // CMN (immediate)
          TODO_DEC();
        } else {
          if (Rn != 0b1101) {
            // ADD (immediate) — ADDS variant
            TODO_DEC();
          } else {
            // ADD (SP plus immediate) — ADDS variant
            TODO_DEC();
          }
        }
        break;

      case 0b1010:
        // ADC (immediate)
        TODO_DEC();
        break;

      case 0b1011:
        // SBC (immediate)
        TODO_DEC();
        break;

      case 0b1101:
        if (!S) {
          if (Rn != 0b1101) {
            // SUB (immediate) — SUB variant
            TODO_DEC();
          } else {
            // SUB (SP minus immediate) — SUB variant
            TODO_DEC();
          }
        } else if (Rd == 15) {
          // CMP (immediate)
          TODO_DEC();
        } else {
          if (Rn != 0b1101) {
            // SUB (immediate) — SUBS variant
            TODO_DEC();
          } else {
            // SUB (SP minus immediate) — SUBS variant
            TODO_DEC();
          }
        }
        break;

      case 0b1110:
        // RSB (immediate)
        TODO_DEC();
        break;

      default:
        abort();
    }
  }

  void _DecodeExecute32(uint32_t instr, uint32_t pc) {
    // For 32-bit instructions, the first 16 bit word is stored in (instr[16:31])
    uint32_t op0 = GETBITS(instr>>16, 9,12);
    uint32_t op1 = GETBITS(instr>>16, 4, 8);
    uint32_t op3 = GETBITS(instr    ,15,15);
    switch (op0) {
      case 0b0110:
      case 0b0111:
      case 0b1110:
      case 0b1111:
        // Coprocessor and floating-point instructions
        TODO_DEC();
        break;

      case 0b0100:
        // Load/store (multiple, dual, exclusive, acquire-release)
        _DecodeExecute32_0100(instr, pc);
        break;

      case 0b0101:
        // Data-processing (shifted register)
        TODO_DEC();
        break;

      case 0b1000:
      case 0b1001:
      case 0b1010:
      case 0b1011:
        if (op3) {
          // Branches and miscellaneous control
          TODO_DEC();
        } else {
          if (!(op0 & 1)) {
            // Data processing (modified immediate)
            _DecodeExecute32_10x0_0(instr, pc);
          } else {
            // Data processing (plain binary immediate)
            _DecodeExecute32_10x1_0(instr, pc);
          }
        }
        break;

      case 0b1100:
        if ((op1 & 0b10001) == 0b10000) {
          // Unallocated
          TODO_DEC();
        } else {
          // Load/store single
          TODO_DEC();
        }
        break;

      case 0b1101:
        if (!(op1 & BIT(4))) {
          // Data processing (register)
          TODO_DEC();
        } else if (!(op1 & BIT(3))) {
          // Multiply, multiply accumulate, and absolute difference
          TODO_DEC();
        } else {
          // Long multiply and divide
          TODO_DEC();
        }
        break;

      default:
        abort();
    }
  }

  void _SetITSTATEAndCommit(uint8_t it) {
    _s.nextInstrITState = it;
    _s.itStateChanged = true;
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__IT_ICI_LO, it>>2);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__IT_ICI_HI, it&3);
  }
  int _HaveSysTick() { return 2; }
  uint32_t _NextInstrAddr() {
    if (_s.pcChanged)
      return _s.nextInstrAddr;
    else
      return _ThisInstrAddr() + _ThisInstrLength();
  }
  int _ThisInstrLength() { return _s.thisInstrLength; }
  uint16_t _GetMemI(uint32_t addr);
  int _Load8(AddressDescriptor memAddrDesc, uint8_t &v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000)
      // Non-32 bit accesses to SCS are UNPREDICTABLE; generate BusFault.
      return 1;

    return _dev.Load8(memAddrDesc.physAddr, v);
  }
  int _Load16(AddressDescriptor memAddrDesc, uint16_t &v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000)
      // Non-32 bit accesses to SCS are UNPREDICTABLE; generate BusFault.
      return 1;

    return _dev.Load16(memAddrDesc.physAddr, v);
  }
  int _Load32(AddressDescriptor memAddrDesc, uint32_t &v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000)
      return _NestLoad32(memAddrDesc.physAddr, memAddrDesc.accAttrs.isPriv, !memAddrDesc.memAttrs.ns, v);

    return _dev.Load32(memAddrDesc.physAddr, v);
  }
  int _Store8(AddressDescriptor memAddrDesc, uint8_t v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000)
      // Non-32 bit accesses to SCS are UNPREDICTABLE; generate BusFault.
      return 1;

    return _dev.Store8(memAddrDesc.physAddr, v);
  }
  int _Store16(AddressDescriptor memAddrDesc, uint16_t v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000)
      // Non-32 bit accesses to SCS are UNPREDICTABLE; generate BusFault.
      return 1;

    return _dev.Store16(memAddrDesc.physAddr, v);
  }
  int _Store32(AddressDescriptor memAddrDesc, uint32_t v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000)
      return _NestStore32(memAddrDesc.physAddr, memAddrDesc.accAttrs.isPriv, !memAddrDesc.memAttrs.ns, v);

    return _dev.Store32(memAddrDesc.physAddr, v);
  }
  std::tuple<bool,uint32_t> _GetMem(AddressDescriptor memAddrDesc, int size) {
    switch (size) {
      case 1: {
        uint8_t v;
        if (_Load8(memAddrDesc, v))
          return {true,0};
        else
          return {false,v};
      }
      case 2: {
        uint16_t v;
        if (_Load16(memAddrDesc, v))
          return {true,0};
        else
          return {false,v};
      }
      case 4: {
        uint32_t v;
        if (_Load32(memAddrDesc, v))
          return {true,0};
        else
          return {false,v};
      }
      default:
        assert(false);
        break;
    }
  }
  bool _SetMem(AddressDescriptor memAddrDesc, int size, uint32_t v) {
    switch (size) {
      case 1:
        return !!_Store8(memAddrDesc, (uint8_t)v);
      case 2:
        return !!_Store16(memAddrDesc, (uint16_t)v);
      case 4:
        return !!_Store32(memAddrDesc, (uint32_t)v);
      default:
        assert(false);
    }
  }

  int _ExecutionPriority();
  bool _HaveHaltingDebug() { return true; }
  bool _CanHaltOnEvent(bool isSecure) {
    if (!_HaveSecurityExt())
      assert(!isSecure);
    return _HaveHaltingDebug() && _HaltingDebugAllowed() && !!(InternalLoad32(REG_DHCSR) & REG_DHCSR__C_DEBUGEN)
      && !(InternalLoad32(REG_DHCSR) & REG_DHCSR__S_HALT) && (!isSecure || !!(InternalLoad32(REG_DHCSR) & REG_DHCSR__S_SDE));
  }
  bool _CanPendMonitorOnEvent(bool isSecure, bool checkPri) {
    if (!_HaveSecurityExt())
      assert(!isSecure);
    return _HaveDebugMonitor() && !_CanHaltOnEvent(isSecure)
      && !!(InternalLoad32(REG_DEMCR) & REG_DEMCR__MON_EN)
      && !(InternalLoad32(REG_DHCSR) & REG_DHCSR__S_HALT)
      && (!isSecure || !!(InternalLoad32(REG_DEMCR) & REG_DEMCR__SDME))
      && (!checkPri || _ExceptionPriority(DebugMonitor, isSecure, true) < _ExecutionPriority());
  }
  uint8_t _ThisInstrITState() {
    if (_HaveMainExt())
      return (GETBITSM(_s.xpsr, XPSR__IT_ICI_LO)<<2) | GETBITSM(_s.xpsr, XPSR__IT_ICI_HI);
    else
      return 0;
  }

  uint8_t _GetITSTATE() { return _ThisInstrITState(); }
  void _SetITSTATE(uint8_t value) {
    _s.nextInstrITState = value;
    _s.itStateChanged = true;
  }

  uint32_t _GetSP(RName spreg) {
    assert(    spreg == RNameSP_Main_NonSecure
           || (spreg == RNameSP_Main_Secure       && _HaveSecurityExt())
           ||  spreg == RNameSP_Process_NonSecure
           || (spreg == RNameSP_Process_Secure    && _HaveSecurityExt()));
    return _s.r[spreg] & ~3;
  }

  ExcInfo _SetSP(RName spreg, bool excEntry, uint32_t value) {
    ExcInfo excInfo = _DefaultExcInfo();
    auto [limit, applyLimit] = _LookUpSPLim(spreg);
    if (applyLimit && value < limit) {
      if (excEntry)
        _s.r[spreg] = limit;

      if (_HaveMainExt())
        InternalOr32(REG_UFSR, REG_UFSR__STKOF);

      excInfo = _CreateException(UsageFault, false, false/*UNKNOWN*/);
      if (!excEntry)
        _HandleException(excInfo);
    } else
      _s.r[spreg] = value & ~3;
    return excInfo;
  }

  ExcInfo _Stack(uint32_t framePtr, int offset, RName spreg, PEMode mode, uint32_t value) {
    auto [limit, applyLimit] = _LookUpSPLim(spreg);
    bool doAccess;
    if (!applyLimit || framePtr >= limit)
      doAccess = true;
    else
      doAccess = IMPL_DEF_PUSH_NON_VIOL_LOCATIONS;

    uint32_t addr = framePtr + offset;
    ExcInfo excInfo;
    if (doAccess && (!applyLimit || ((addr >= limit)))) {
      bool secure = (spreg == RNameSP_Main_Secure || spreg == RNameSP_Process_Secure);
      bool isPriv = secure ? !GETBITSM(_s.controlS, CONTROL__NPRIV) : !GETBITSM(_s.controlNS, CONTROL__NPRIV);
      isPriv = isPriv || mode == PEMode_Handler;
      excInfo = _MemA_with_priv_security(addr, 4, AccType_STACK, isPriv, secure, true, value);
    } else
      excInfo = _DefaultExcInfo();

    return excInfo;
  }

  std::tuple<ExcInfo, uint32_t> _Stack(uint32_t framePtr, int offset, RName spreg, PEMode mode) {
    bool secure = (spreg == RNameSP_Main_Secure || spreg == RNameSP_Process_Secure);
    bool isPriv = secure ? !(_s.controlS & CONTROL__NPRIV) : !(_s.controlNS & CONTROL__NPRIV);
    isPriv = isPriv || mode == PEMode_Handler;
    uint32_t addr = framePtr + offset;
    auto [excInfo, value] = _MemA_with_priv_security(addr, 4, AccType_STACK, isPriv, secure, true);
    return {excInfo, value};
  }

  uint32_t _GetLR() { return _GetR(14); }
  void _SetLR(uint32_t v) { _SetR(14, v); }

  bool _HaveDSPExt() { return false; }

  uint32_t _GetR(int n) {
    assert(n >= 0 && n <= 15);
    switch (n) {
      case  0: return _s.r[RName0];
      case  1: return _s.r[RName1];
      case  2: return _s.r[RName2];
      case  3: return _s.r[RName3];
      case  4: return _s.r[RName4];
      case  5: return _s.r[RName5];
      case  6: return _s.r[RName6];
      case  7: return _s.r[RName7];
      case  8: return _s.r[RName8];
      case  9: return _s.r[RName9];
      case 10: return _s.r[RName10];
      case 11: return _s.r[RName11];
      case 12: return _s.r[RName12];
      case 13: return _s.r[_LookUpSP()] & ~3;
      case 14: return _s.r[RName_LR];
      case 15: return _s.r[RName_PC] + 4;
      default: assert(false);
    }
  }
  void _SetR(int n, uint32_t v) {
    assert(n >= 0 && n <= 14);
    switch (n) {
      case  0: _s.r[RName0 ] = v; break;
      case  1: _s.r[RName1 ] = v; break;
      case  2: _s.r[RName2 ] = v; break;
      case  3: _s.r[RName3 ] = v; break;
      case  4: _s.r[RName4 ] = v; break;
      case  5: _s.r[RName5 ] = v; break;
      case  6: _s.r[RName6 ] = v; break;
      case  7: _s.r[RName7 ] = v; break;
      case  8: _s.r[RName8 ] = v; break;
      case  9: _s.r[RName9 ] = v; break;
      case 10: _s.r[RName10] = v; break;
      case 11: _s.r[RName11] = v; break;
      case 12: _s.r[RName12] = v; break;
      case 13:
        if (IMPL_DEF_SPLIM_CHECK_UNPRED_INSTR)
          _SetSP(_LookUpSP(), false, v);
        else
          _s.r[_LookUpSP()] = v & ~3;
        break;
      case 14:
        _s.r[RName_LR] = v;
        break;
    }
  }

  RName _LookUpSP_with_security_mode(bool isSecure, PEMode mode) {
    bool spSel;

    if (isSecure)
      spSel = !!(_s.controlS & CONTROL__SPSEL);
    else
      spSel = !!(_s.controlNS & CONTROL__SPSEL);

    if (spSel && mode == PEMode_Thread)
      return isSecure ? RNameSP_Process_Secure : RNameSP_Process_NonSecure;
    else
      return isSecure ? RNameSP_Main_Secure : RNameSP_Main_NonSecure;
  }

  RName _LookUpSP() {
    return _LookUpSP_with_security_mode(_IsSecure(), _CurrentMode());
  }

  std::tuple<uint32_t, bool> _LookUpSPLim(RName spreg) {
    uint32_t limit;
    switch (spreg) {
      case RNameSP_Main_Secure:
        limit = _s.msplimS & ~7;
        break;
      case RNameSP_Process_Secure:
        limit = _s.psplimS & ~7;
        break;
      case RNameSP_Main_NonSecure:
        limit = _HaveMainExt() ? (_s.msplimNS & ~7) : 0;
        break;
      case RNameSP_Process_NonSecure:
        limit = _HaveMainExt()? (_s.psplimNS & ~7) : 0;
        break;
      default:
        assert(false);
    }

    bool applyLimit;
    bool secure = (spreg == RNameSP_Main_Secure || spreg == RNameSP_Process_Secure);
    assert(!secure || _HaveSecurityExt());
    if (_HaveMainExt() && _IsReqExcPriNeg(secure)) {
      bool ignLimit = secure ? (InternalLoad32(REG_CCR_S) & REG_CCR__STKOFHFNMIGN) : (InternalLoad32(REG_CCR_NS) & REG_CCR__STKOFHFNMIGN);
      applyLimit = !ignLimit;
    } else
      applyLimit = true;

    return {limit, applyLimit};
  }

  bool _IsReqExcPriNeg(bool secure) {
    bool neg = _IsActiveForState(NMI, secure) || _IsActiveForState(HardFault, secure);
    if (_HaveMainExt()) {
      uint32_t faultMask = secure ? _s.faultmaskS : _s.faultmaskNS;
      if (faultMask & 1)
        neg = true;
    }
    return neg;
  }

  uint32_t _GetSP() { return _GetR(13); }
  void _SetSP(uint32_t value) { _SetRSPCheck(13, value); }
  uint32_t _GetSP_Main() { return _IsSecure() ? _GetSP_Main_Secure() : _GetSP_Main_NonSecure(); }
  void _SetSP_Main(uint32_t value) {
    if (_IsSecure())
      _SetSP_Main_Secure(value);
    else
      _SetSP_Main_NonSecure(value);
  }
  uint32_t _GetSP_Main_NonSecure() {
    return _GetSP(RNameSP_Main_NonSecure);
  }
  void _SetSP_Main_NonSecure(uint32_t value) {
    _SetSP(RNameSP_Main_NonSecure, false, value);
  }
  void _SetSP_Main_Secure(uint32_t value) {
    _SetSP(RNameSP_Main_Secure, false, value);
  }
  uint32_t _GetSP_Main_Secure() {
    return _GetSP(RNameSP_Main_Secure);
  }
  uint32_t _GetSP_Process() {
    return _IsSecure() ? _GetSP_Process_Secure() : _GetSP_Process_NonSecure();
  }
  void _SetSP_Process(uint32_t value) {
    if (_IsSecure())
      _SetSP_Process_Secure(value);
    else
      _SetSP_Process_NonSecure(value);
  }
  uint32_t _GetSP_Process_NonSecure() {
    return _GetSP(RNameSP_Process_NonSecure);
  }
  void _SetSP_Process_NonSecure(uint32_t value) {
    _SetSP(RNameSP_Process_NonSecure, false, value);
  }
  uint32_t _GetSP_Process_Secure() {
    return _GetSP(RNameSP_Process_Secure);
  }
  void _SetSP_Process_Secure(uint32_t value) {
    _SetSP(RNameSP_Process_Secure, false, value);
  }
  void _SetRSPCheck(int n, uint32_t v) {
    if (n == 13)
      _SetSP(_LookUpSP(), false, v);
    else
      _SetR(n, v);
  }
  SAttributes _SecurityCheck(uint32_t addr, bool isInstrFetch, bool isSecure);
  void _Lockup(bool termInst) {
    InternalOr32(REG_DHCSR, REG_DHCSR__S_LOCKUP);
    _BranchToAndCommit(0xEFFF'FFFE);
    if (termInst)
      _EndOfInstruction();
  }

  void _BranchToAndCommit(uint32_t addr) {
    _s.r[RName_PC]    = addr & ~1;
    _s.pcChanged      = true;
    _s.nextInstrAddr  = addr & ~1;
    _s.pendingReturnOperation = false;
  }

  void _BranchTo(uint32_t addr) {
    _s.nextInstrAddr = addr;
    _s.pcChanged     = true;
    _s.pendingReturnOperation = false;
  }

  void _PendReturnOperation(uint32_t retValue) {
    _s.nextInstrAddr          = retValue;
    _s.pcChanged              = true;
    _s.pendingReturnOperation = true;
  }

  bool _IsActiveForState(int exc, bool isSecure) {
    if (!_HaveSecurityExt())
      isSecure = false;

    bool active;
    if (_IsExceptionTargetConfigurable(exc))
      active = (_s.excActive[exc] && _ExceptionTargetsSecure(exc, isSecure) == isSecure);
    else {
      int idx = isSecure ? 0 : 1;
      active = !!(_s.excActive[exc] & BIT(idx));
    }

    return active;
  }

  bool _IsExceptionTargetConfigurable(int e) {
    if (!_HaveSecurityExt())
      return false;

    switch (e) {
      case NMI: return true;
      case BusFault: return true;
      case DebugMonitor: return true;
      case SysTick: return _HaveSysTick() == 1;
      default: return e >= 16;
    }
  }

  std::tuple<ExcInfo, uint32_t> _GetVector(int excNo, bool isSecure) {
    uint32_t vtor = isSecure ? InternalLoad32(REG_VTOR_S) : InternalLoad32(REG_VTOR_NS);
    uint32_t addr = (vtor & ~BITS(0,6)) + 4*excNo;
    auto [exc, vector] = _MemA_with_priv_security(addr, 4, AccType_VECTABLE, true, isSecure, true);
    if (exc.fault != NoFault) {
      exc.isTerminal = true;
      exc.fault = HardFault;
      exc.isSecure = exc.isSecure || !(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS);
      InternalOr32(REG_HFSR, REG_HFSR__VECTTBL);
    }
    return {exc, vector};
  }

  std::tuple<ExcInfo, AddressDescriptor> _ValidateAddress(uint32_t addr, AccType accType, bool isPriv, bool secure, bool isWrite, bool aligned) {
    AddressDescriptor result;
    Permissions       perms;

    bool    ns            = false; // UNKNOWN
    ExcInfo excInfo       = _DefaultExcInfo();
    bool    isInstrFetch  = (accType == AccType_IFETCH);

    bool secureMpu;
    SAttributes sAttrib;
    if (_HaveSecurityExt()) {
      sAttrib = _SecurityCheck(addr, isInstrFetch, secure);
      if (isInstrFetch) {
        ns        = sAttrib.ns;
        secureMpu = !sAttrib.ns;
        isPriv    = _CurrentModeIsPrivileged(secureMpu);
      } else {
        ns        = (!secure || sAttrib.ns);
        secureMpu = secure;
      }
    } else {
      ns        = true;
      secureMpu = false;
    }

    std::tie(result.memAttrs, perms) = _MPUCheck(addr, accType, isPriv, secureMpu);
    result.memAttrs.ns = ns;

    if (!aligned && result.memAttrs.memType == MemType_Device && perms.apValid) {
      InternalOr32(REG_UFSR, REG_UFSR__UNALIGNED);
      excInfo = _CreateException(UsageFault, false, false/*UNKNOWN*/);
    }

    if (excInfo.fault == NoFault && _HaveSecurityExt()) {
      bool raiseSecFault = false;
      if (isInstrFetch) {
        if (secure) {
          if (sAttrib.ns) {
            InternalOr32(REG_SFSR, REG_SFSR__INVTRAN);
            raiseSecFault = true;
          }
        } else {
          if (!sAttrib.ns && !sAttrib.nsc) {
            InternalOr32(REG_SFSR, REG_SFSR__INVEP);
            raiseSecFault = true;
          }
        }
      } else {
        if (!secure && !sAttrib.ns) {
          if (_HaveMainExt() && accType != AccType_VECTABLE) {
            if (accType == AccType_LAZYFP)
              InternalOr32(REG_SFSR, REG_SFSR__LSPERR);
            else
              InternalOr32(REG_SFSR, REG_SFSR__AUVIOL);
            InternalOr32(REG_SFSR, REG_SFSR__SFARVALID);
            InternalStore32(REG_SFAR, addr);
          }
          raiseSecFault = true;
        }
      }
      if (raiseSecFault)
        excInfo = _CreateException(SecureFault, true, true);
    }

    result.physAddr = addr;
    result.accAttrs.isWrite = isWrite;
    result.accAttrs.isPriv  = isPriv;
    result.accAttrs.accType = accType;

    if (excInfo.fault == NoFault)
      excInfo = _CheckPermission(perms, addr, accType, isWrite, isPriv, secureMpu);

    return {excInfo, result};
  }

  uint32_t _MemU(uint32_t addr, int size) {
    if (_HaveMainExt())
      return _MemU_with_priv(addr, size, _FindPriv());
    else
      return _MemA(addr, size);
  }

  void _MemU(uint32_t addr, int size, uint32_t value) {
    if (_HaveMainExt())
      _MemU_with_priv(addr, size, _FindPriv(), value);
    else
      _MemA(addr, size, value);
  }

  uint32_t _MemU_with_priv(uint32_t addr, int size, bool priv) {
    uint32_t value;

    // Do aligned access, take alignment fault, or do sequence of bytes
    if (addr == _Align(addr, size)) {
      value = _MemA_with_priv(addr, size, priv, true);
    } else if (InternalLoad32(REG_CCR) & REG_CCR__UNALIGN_TRP) {
      InternalOr32(REG_UFSR, REG_UFSR__UNALIGNED);
      auto excInfo = _CreateException(UsageFault, false, UNKNOWN_VAL(false));
      _HandleException(excInfo);
    } else { // if unaligned access
      for (int i=0; i<size; ++i)
        value = CHGBITS(value, 8*i, 8*i+7, _MemA_with_priv(addr+i, 1, priv, false));
      // PPB (0xE0000000 to 0xE0100000) is always little endian
      if ((InternalLoad32(REG_AIRCR) & REG_AIRCR__ENDIANNESS) && GETBITS(addr,20,31) != 0xE00)
        value = _BigEndianReverse(value, size);
    }

    return value;
  }

  void _MemU_with_priv(uint32_t addr, int size, bool priv, uint32_t value) {

  }

  uint32_t _MemA(uint32_t addr, int size) {
    return _MemA_with_priv(addr, size, _FindPriv(), true);
  }

  void _MemA(uint32_t addr, int size, uint32_t value) {
    _MemA_with_priv(addr, size, _FindPriv(), true, value);
  }

  uint32_t _MemA_with_priv(uint32_t addr, int size, bool priv, bool aligned) {
    auto [excInfo, value] = _MemA_with_priv_security(addr, size, AccType_NORMAL, priv, _IsSecure(), aligned);
    _HandleException(excInfo);
    return value;
  }

  void _MemA_with_priv(uint32_t addr, int size, bool priv, bool aligned, uint32_t value) {
    auto excInfo = _MemA_with_priv_security(addr, size, AccType_NORMAL, priv, _IsSecure(), aligned, value);
    _HandleException(excInfo);
  }

  std::tuple<ExcInfo, uint32_t> _MemA_with_priv_security(uint32_t addr, int size, AccType accType, bool priv, bool secure, bool aligned) {
    ExcInfo excInfo = _DefaultExcInfo();
    if (!_IsAligned(addr, size)) {
      if (_HaveMainExt())
        InternalOr32(REG_UFSR, REG_UFSR__UNALIGNED);
      excInfo = _CreateException(UsageFault, true, secure);
    }

    uint32_t value;
    AddressDescriptor memAddrDesc;
    if (excInfo.fault == NoFault)
      std::tie(excInfo, memAddrDesc) = _ValidateAddress(addr, accType, priv, secure, false, aligned);

    if (excInfo.fault == NoFault) {
      bool error;
      std::tie(error, value) = _GetMem(memAddrDesc, size);

      if (error) {
        value = 0; // UNKNOWN
        if (_HaveMainExt()) {
          if (accType == AccType_STACK)
            InternalOr32(REG_BFSR, REG_BFSR__UNSTKERR);
          else if (accType == AccType_NORMAL || accType == AccType_ORDERED) {
            uint32_t bfar = InternalLoad32(REG_BFAR);
            bfar = CHGBITSM(bfar, REG_BFAR__ADDRESS, addr);
            InternalStore32(REG_BFAR, bfar);
            InternalOr32(REG_BFSR, REG_BFSR__BFARVALID | REG_BFSR__PRECISERR);
          }
        }

        if (!_IsReqExcPriNeg(secure) || !(InternalLoad32(REG_CCR) & REG_CCR__BFHFNMIGN))
          excInfo = _CreateException(BusFault, false, false/*UNKNOWN*/);
      } else if ((InternalLoad32(REG_AIRCR) & REG_AIRCR__ENDIANNESS) && GETBITS(addr,20,31) != 0xE00)
        value = _BigEndianReverse(value, size);

      if (_IsDWTEnabled()) {
        uint32_t dvalue = value;
        _DWT_DataMatch(addr, size, dvalue, true, secure);
      }
    }

    return {excInfo, value};
  }

  ExcInfo _MemA_with_priv_security(uint32_t addr, int size, AccType accType, bool priv, bool secure, bool aligned, uint32_t value) {
    ExcInfo excInfo = _DefaultExcInfo();

    if (!_IsAligned(addr, size)) {
      if (_HaveMainExt())
        InternalOr32(REG_UFSR, REG_UFSR__UNALIGNED);
      excInfo = _CreateException(UsageFault, true, secure);
    }

    AddressDescriptor memAddrDesc;
    if (excInfo.fault == NoFault)
      std::tie(excInfo, memAddrDesc) = _ValidateAddress(addr, accType, priv, secure, true, aligned);

    if (excInfo.fault == NoFault) {
      if (memAddrDesc.memAttrs.shareable)
        _ClearExclusiveByAddress(memAddrDesc.physAddr, _ProcessorID(), size);

      if (_IsDWTEnabled()) {
        uint32_t dvalue = value;
        _DWT_DataMatch(addr, size, dvalue, false, secure);
      }

      if ((InternalLoad32(REG_AIRCR) & REG_AIRCR__ENDIANNESS) && GETBITS(addr,20,31) != 0xE00)
        value = _BigEndianReverse(value, size);

      if (_SetMem(memAddrDesc, size, value)) {
        bool negativePri;
        if (accType == AccType_LAZYFP)
          negativePri = !(InternalLoad32(REG_FPCCR_S) & REG_FPCCR__HFRDY);
        else
          negativePri = _IsReqExcPriNeg(secure);

        if (_HaveMainExt()) {
          if (accType == AccType_STACK)
            InternalOr32(REG_BFSR, REG_BFSR__STKERR);
          else if (accType == AccType_LAZYFP)
            InternalOr32(REG_BFSR, REG_BFSR__LSPERR);
          else if (accType == AccType_NORMAL || accType == AccType_ORDERED) {
            InternalStore32(REG_BFAR, addr);
            InternalOr32(REG_BFSR, REG_BFSR__BFARVALID | REG_BFSR__PRECISERR);
          }
        }

        if (!negativePri || !(InternalLoad32(REG_CCR) & REG_CCR__BFHFNMIGN))
          excInfo = _CreateException(BusFault, false, false/*UNKNOWN*/);
      }
    }

    return excInfo;
  }

  void _ClearExclusiveByAddress(uint32_t addr, int exclProcID, int size) {
    // TODO
  }

  bool _IsAligned(uint32_t addr, int size) {
    assert(size == 1 || size == 2 || size == 4 || size == 8);
    uint32_t mask = (size-1);
    return !(addr & mask);
  }

  std::tuple<MemoryAttributes, Permissions> _MPUCheck(uint32_t addr, AccType accType, bool isPriv, bool secure) {
    assert(_HaveSecurityExt() || !secure);

    MemoryAttributes  attrs       = _DefaultMemoryAttributes(addr);
    Permissions       perms       = _DefaultPermissions(addr);
    bool              hit         = false;
    bool              isPPBAccess = (GETBITS(addr,20,31) == 0b111000000000);

    uint32_t mpuCtrl, mpuType;
    uint64_t mair;
    if (secure) {
      mpuCtrl = InternalLoad32(REG_MPU_CTRL_S);
      mpuType = InternalLoad32(REG_MPU_TYPE_S);
      mair    = (uint64_t(InternalLoad32(REG_MPU_MAIR1_S))<<32) | (uint64_t)InternalLoad32(REG_MPU_MAIR0_S);
    } else {
      mpuCtrl = InternalLoad32(REG_MPU_CTRL_NS);
      mpuType = InternalLoad32(REG_MPU_TYPE_NS);
      mair    = (uint64_t(InternalLoad32(REG_MPU_MAIR1_NS))<<32) | (uint64_t)InternalLoad32(REG_MPU_MAIR0_NS);
    }

    bool negativePri;
    if (accType == AccType_LAZYFP)
      negativePri = !(InternalLoad32(REG_FPCCR_S) & REG_FPCCR__HFRDY);
    else
      negativePri = _IsReqExcPriNeg(secure);

    if (accType == AccType_VECTABLE || isPPBAccess)
      hit = true;
    else if (!(mpuCtrl & REG_MPU_CTRL__ENABLE)) {
      if (mpuCtrl & REG_MPU_CTRL__HFNMIENA)
        throw Exception(ExceptionType::UNPREDICTABLE);
      else
        hit = true;
    } else if (!(mpuCtrl & REG_MPU_CTRL__HFNMIENA) && negativePri)
      hit = true;
    else {
      if ((mpuCtrl & REG_MPU_CTRL__PRIVDEFENA) && isPriv)
        hit = true;

      bool regionMatched = false;
      int  numRegions = GETBITSM(mpuType, REG_MPU_TYPE__DREGION);
      for (int r=0; r<numRegions; ++r) {
        uint32_t rbar, rlar;
        if (secure)
          std::tie(rbar,rlar) = _dev.InternalLoadMpuSecureRegion(r);
        else
          std::tie(rbar,rlar) = _dev.InternalLoadMpuNonSecureRegion(r);

        if (rlar & REG_MPU_RLAR__EN) {
          if (   addr >= ( GETBITSM(rbar, REG_MPU_RBAR__BASE )<<5)
              && addr <= ((GETBITSM(rlar, REG_MPU_RLAR__LIMIT)<<5) | 0b11111)) {
            uint32_t sh = 0;
            if (regionMatched) {
              perms.regionValid = false;
              perms.region      = 0;
              hit               = false;
            } else {
              regionMatched = true;
              perms.ap      = GETBITSM(rbar, REG_MPU_RBAR__AP);
              perms.xn      = GETBITSM(rbar, REG_MPU_RBAR__XN);
              perms.region  = r & 0xFF;
              perms.regionValid = true;
              hit = true;
              sh = GETBITSM(rbar, REG_MPU_RBAR__SH);
            }

            uint32_t idx        = GETBITSM(rlar, REG_MPU_RLAR__ATTR_IDX);
            uint32_t attrField  = GETBITS(mair, 8*idx, 8*idx + 7);
            attrs               = _MAIRDecode(attrField, sh);
          }
        }
      }
    }

    if (GETBITS(addr,29,31) == 0b111)
      perms.xn = true;

    if (!hit)
      perms.apValid = false;

    return {attrs, perms};
  }

  MemoryAttributes _MAIRDecode(uint8_t attrField, uint8_t sh) {
    MemoryAttributes memAttrs;
    bool unpackInner;

    if (!GETBITS(attrField, 4, 7)) {
      unpackInner = false;
      memAttrs.memType        = MemType_Device;
      memAttrs.shareable      = true;
      memAttrs.outerShareable = true;
      memAttrs.innerAttrs     = 0; // UNKNOWN
      memAttrs.outerAttrs     = 0; // UNKNOWN
      memAttrs.innerHints     = 0; // UNKNOWN
      memAttrs.outerHints     = 0; // UNKNOWN
      memAttrs.innerTransient = false; // UNKNOWN
      memAttrs.outerTransient = false; // UNKNOWN
      switch (GETBITS(attrField, 0, 3)) {
        case 0b0000: memAttrs.device = DeviceType_nGnRnE; break;
        case 0b0100: memAttrs.device = DeviceType_nGnRE; break;
        case 0b1000: memAttrs.device = DeviceType_nGRE; break;
        case 0b1100: memAttrs.device = DeviceType_GRE; break;
      }
      if (GETBITS(attrField, 0, 1))
        throw Exception(ExceptionType::UNPREDICTABLE);
    } else {
      unpackInner = true;
      memAttrs.memType        = MemType_Normal;
      memAttrs.device         = DeviceType_GRE; // UNKNOWN
      memAttrs.outerHints     = GETBITS(attrField, 4, 5);
      memAttrs.shareable      = (sh & BIT(1));
      memAttrs.outerShareable = (sh == 0b10);
      if (sh == 0b01)
        throw Exception(ExceptionType::UNPREDICTABLE);

      if (GETBITS(attrField, 6, 7) == 0b00) {
        memAttrs.outerAttrs       = 0b10;
        memAttrs.outerTransient   = true;
      } else if (GETBITS(attrField, 6, 7) == 0b01) {
        if (GETBITS(attrField, 4, 5) == 0b00) {
          memAttrs.outerAttrs     = 0b00;
          memAttrs.outerTransient = false;
        } else {
          memAttrs.outerAttrs     = 0b11;
          memAttrs.outerTransient = true;
        }
      } else {
        memAttrs.outerAttrs     = GETBITS(attrField, 6, 7);
        memAttrs.outerTransient = false;
      }
    }

    if (unpackInner) {
      if (GETBITS(attrField, 0, 3) == 0b0000)
        throw Exception(ExceptionType::UNPREDICTABLE);
      else {
        if (GETBITS(attrField, 2, 3) == 0b00) {
          memAttrs.innerAttrs     = 0b10;
          memAttrs.innerHints     = GETBITS(attrField, 0, 1);
          memAttrs.innerTransient = true;
        } else if (GETBITS(attrField, 2, 3) == 0b01) {
          memAttrs.innerHints     = GETBITS(attrField, 0, 1);
          if (GETBITS(attrField, 0, 1) == 0b00) {
            memAttrs.innerAttrs     = 0b00;
            memAttrs.innerTransient = false;
          } else {
            memAttrs.innerAttrs     = 0b11;
            memAttrs.innerTransient = true;
          }
        } else if (GETBITS(attrField, 2, 3) == 0b10) {
          memAttrs.innerHints     = GETBITS(attrField, 0, 1);
          memAttrs.innerAttrs     = 0b10;
          memAttrs.innerTransient = false;
        } else if (GETBITS(attrField, 2, 3) == 0b11) {
          memAttrs.innerHints     = GETBITS(attrField, 0, 1);
          memAttrs.innerAttrs     = 0b11;
          memAttrs.innerTransient = false;
        } else
          throw Exception(ExceptionType::UNPREDICTABLE);
      }
    }

    return memAttrs;
  }

  ExcInfo _CheckPermission(const Permissions &perms, uint32_t addr, AccType accType, bool isWrite, bool isPriv, bool isSecure) {
    bool fault = true;
    if (!perms.apValid)
      fault = true;
    else if (perms.xn && accType == AccType_IFETCH)
      fault = true;
    else
      switch (perms.ap) {
        case 0b00: fault = !isPriv; break;
        case 0b01: fault = false; break;
        case 0b10: fault = !isPriv || isWrite; break;
        case 0b11: fault = isWrite; break;
        default: throw Exception(ExceptionType::UNPREDICTABLE);
      }

    if (!fault)
      return _DefaultExcInfo();

    if (_HaveMainExt()) {
      uint8_t fsr = 0;
      switch (accType) {
        case AccType_IFETCH:
          fsr |= REG_MMFSR__IACCVIOL;
          break;
        case AccType_STACK:
          if (isWrite)
            fsr |= REG_MMFSR__MSTKERR;
          else
            fsr |= REG_MMFSR__MUNSTKERR;
          break;
        case AccType_LAZYFP:
          fsr |= REG_MMFSR__MLSPERR;
          break;
        case AccType_NORMAL:
        case AccType_ORDERED:
          fsr |= REG_MMFSR__MMARVALID;
          fsr |= REG_MMFSR__DACCVIOL;
          break;
        default:
          assert(false);
          break;
      }

      if (isSecure) {
        InternalOr32(REG_MMFSR_S, fsr);
        if (fsr & REG_MMFSR__MMARVALID)
          InternalStore32(REG_MMFAR_S, addr);
      } else {
        InternalOr32(REG_MMFSR_NS, fsr);
        if (fsr & REG_MMFSR__MMARVALID)
          InternalStore32(REG_MMFAR_NS, addr);
      }
    }

    return _CreateException(MemManage, true, isSecure);
  }

  uint32_t _BigEndianReverse(uint32_t value, int N) {
    assert(N == 1 || N == 2 || N == 4);
    uint32_t result;
    switch (N) {
      case 1:
        return value & UINT8_MAX;
      case 2:
        return (uint16_t(value)>>8) | (uint16_t(value)<<8);
      default:
      case 4:
        return
          (GETBITS(value,24,31)<< 0)
        | (GETBITS(value,16,23)<< 8)
        | (GETBITS(value, 8,15)<<16)
        | (GETBITS(value, 0, 7)<<24)
        ;
    }
  }

  void _DWT_DataMatch(uint32_t daddr, int dsize, uint32_t dvalue, bool read, bool nsReq) {
    bool triggerDebugEvent  = false;
    bool debugEvent         = false;

    uint32_t numComp = GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP);
    if (!_HaveDWT() || !numComp)
      return;

    for (uint32_t i=0; i<numComp; ++i) {
      if (_IsDWTConfigUnpredictable(i))
        throw Exception(ExceptionType::UNPREDICTABLE);

      bool daddrMatch   = _DWT_DataAddressMatch(i, daddr, dsize, read, nsReq);
      bool dvalueMatch  = _DWT_DataValueMatch(i, daddr, dvalue, dsize, read, nsReq);

      if (daddrMatch && (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__MATCH) & 0b1100) == 0b0100) {
        if (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__MATCH) != 0b0111) {
          InternalOr32(REG_DWT_FUNCTION(i), REG_DWT_FUNCTION__MATCHED);
          debugEvent = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__ACTION) == 0b01);
        } else {
          InternalMask32(REG_DWT_FUNCTION(i), REG_DWT_FUNCTION__MATCHED); // UNKNOWN
          InternalOr32(REG_DWT_FUNCTION(i-1), REG_DWT_FUNCTION__MATCHED);
          debugEvent = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__ACTION) == 0b01);
        }
      }

      if (dvalueMatch && (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__MATCH) & 0b1100) == 0b1000) {
        if (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__MATCH) != 0b1011) {
          InternalOr32(REG_DWT_FUNCTION(i), REG_DWT_FUNCTION__MATCHED);
          debugEvent = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__ACTION) == 0b01);
        } else {
          InternalOr32(REG_DWT_FUNCTION(i), REG_DWT_FUNCTION__MATCHED);
          debugEvent = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__ACTION) == 0b01);
        }
      }

      if (daddrMatch && (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__MATCH) & 0b1100) == 0b1100)
        InternalOr32(REG_DWT_FUNCTION(i), REG_DWT_FUNCTION__MATCHED);

      triggerDebugEvent = triggerDebugEvent || debugEvent;
    }

    if (triggerDebugEvent)
      debugEvent = _SetDWTDebugEvent(!nsReq);
  }

  bool _DWT_DataAddressMatch(int N, uint32_t daddr, int dsize, bool read, bool nsReq) {
    // TODO
    return false;
  }

  bool _DWT_DataValueMatch(int N, uint32_t daddr, uint32_t dvalue, int dsize, bool read, bool nsReq) {
    // TODO
    return false;
  }

  bool _IsDWTConfigUnpredictable(int N) {
    // TODO
    return false;
  }

  bool _SetDWTDebugEvent(bool secureMatch) {
    if (_CanHaltOnEvent(secureMatch)) {
      InternalOr32(REG_DHCSR, REG_DHCSR__C_HALT);
      InternalOr32(REG_DFSR, REG_DFSR__DWTTRAP);
      return true;
    }

    if (_HaveMainExt() && _CanPendMonitorOnEvent(secureMatch, true)) {
      InternalOr32(REG_DEMCR, REG_DEMCR__MON_PEND);
      InternalOr32(REG_DFSR, REG_DFSR__DWTTRAP);
      return true;
    }

    return false;
  }

  MemoryAttributes _DefaultMemoryAttributes(uint32_t addr) {
    MemoryAttributes attrs;

    switch (GETBITS(addr,29,31)) {
      case 0b000:
        attrs.memType     = MemType_Normal;
        attrs.device      = DeviceType_GRE; // UNKNOWN
        attrs.innerAttrs  = 0b10;
        attrs.shareable   = false;
        break;
      case 0b001:
        attrs.memType     = MemType_Normal;
        attrs.device      = DeviceType_GRE; // UNKNOWN
        attrs.innerAttrs  = 0b01;
        attrs.shareable   = false;
        break;
      case 0b010:
        attrs.memType     = MemType_Device;
        attrs.device      = DeviceType_nGnRE;
        attrs.innerAttrs  = 0b00;
        attrs.shareable   = true;
        break;
      case 0b011:
        attrs.memType     = MemType_Normal;
        attrs.device      = DeviceType_GRE; // UNKNOWN
        attrs.innerAttrs  = 0b01;
        attrs.shareable   = false;
        break;
      case 0b100:
        attrs.memType     = MemType_Normal;
        attrs.device      = DeviceType_GRE; // UNKNOWN
        attrs.innerAttrs  = 0b10;
        attrs.shareable   = false;
        break;
      case 0b101:
        attrs.memType     = MemType_Device;
        attrs.device      = DeviceType_nGnRE;
        attrs.innerAttrs  = 0b00;
        attrs.shareable   = true;
        break;
      case 0b110:
        attrs.memType     = MemType_Device;
        attrs.device      = DeviceType_nGnRE;
        attrs.innerAttrs  = 0b00;
        attrs.shareable   = true;
        break;
      case 0b111:
        if (!GETBITS(addr,20,28)) {
          attrs.memType     = MemType_Device;
          attrs.device      = DeviceType_nGnRnE;
          attrs.innerAttrs  = 0b00;
          attrs.shareable   = true;
        } else {
          attrs.memType     = MemType_Device;
          attrs.device      = DeviceType_nGnRE;
          attrs.innerAttrs  = 0b00;
          attrs.shareable   = true;
        }
        break;
    }

    attrs.outerAttrs      = attrs.innerAttrs;
    attrs.outerShareable  = attrs.shareable;
    attrs.ns              = false; // UNKNOWN
    return attrs;
  }

  Permissions _DefaultPermissions(uint32_t addr) {
    Permissions perms = {};
    perms.ap          = 0b01;
    perms.apValid     = true;
    perms.region      = 0;
    perms.regionValid = false;

    switch (GETBITS(addr,29,31)) {
      case 0b000: perms.xn = false; break;
      case 0b001: perms.xn = false; break;
      case 0b010: perms.xn = true;  break;
      case 0b011: perms.xn = false; break;
      case 0b100: perms.xn = false; break;
      case 0b101: perms.xn = true;  break;
      case 0b110: perms.xn = true;  break;
      case 0b111: perms.xn = true;  break;
    }

    return perms;
  }

  void _SetPending(int exc, bool isSecure, bool setNotClear) {
    if (!_HaveSecurityExt())
      isSecure = false;

    if (_IsExceptionTargetConfigurable(exc))
      _s.excPending[exc] = setNotClear ? 0b11 : 0b00;
    else {
      uint32_t idx = isSecure ? 0 : 1;
      _s.excPending[exc] = CHGBITS(_s.excPending[exc], idx, idx, setNotClear);
    }
  }

  uint8_t _NextInstrITState() {
    uint8_t nextState;
    if (_HaveMainExt()) {
      if (_s.itStateChanged)
        nextState = _s.nextInstrITState;
      else {
        nextState = _ThisInstrITState();
        if (GETBITS(nextState, 0, 2) == 0b000)
          nextState = 0;
        else
          nextState = CHGBITS(nextState, 0, 4, GETBITS(nextState, 0, 4)<<1);
      }
    } else
      nextState = 0;
    return nextState;
  }

  // XXX: Unfortunately the ARM ISA manual exceptionally does not give a definition
  // for this function. Its definition has been estimated via reference to qemu's codebase.
  std::tuple<bool, int, bool> _PendingExceptionDetails() {
    // _NvicPendingPriority() has a value higher than the highest possible
    // priority value if there is no pending interrupt so there is an interrupt
    // to be handled iff this is true.
    auto [pendingPrio, pendingExcNo, excIsSecure] = _PendingExceptionDetailsActual();
    bool canTakePendingExc = (_ExecutionPriority() > pendingPrio);

    if (!canTakePendingExc)
      return {false, 0, false};

    return {true, pendingExcNo, excIsSecure};
  }

  // XXX: Custom function, not found in ISA definition.
  std::tuple<int,int,bool> _PendingExceptionDetailsActual() {
    int  maxPrio      = 0x100; // Higher than any possible execution priority
    int  maxPrioExc   = 0;
    bool excIsSecure  = false;

    for (int i=NMI; i<16; ++i) { // Reset is not handled here
      for (int j=0; j<2; ++j) { // j=0: secure exception, j=1: non-secure exception
        if (!(_s.excPending[i] & BIT(j)))
          continue;

        bool excIsSecure_ = _ExceptionTargetsSecure(i, j == 0);
        int  excPrio      = _ExceptionPriority(i, excIsSecure_, /*applyPrigroup=*/true);

        if (excPrio < maxPrio) {
          maxPrio     = excPrio;
          maxPrioExc  = i;
          excIsSecure = excIsSecure_;
        }
      }
    }

    for (int i=0; i<16; ++i) {
      uint32_t v = InternalLoad32(REG_NVIC_ISPRn_S(i));
      if (!v)
        continue;

      // ARMv8-M supports exceptions in range [1,511], leaving room for 495
      // external interrupts. We must not attempt to ask about exceptions with
      // numbers above 511, even if the _dev implementation is buggy and
      // returns ones for ISPRn(15) bits [16:31].
      if (i == 15)
        v &= 0x0000FFFF;

      while (v) {
        // Determine number of least significant bit which is set.
        uint32_t bitNo    = CTZL(v);
        uint32_t intrNo   = i*32 + bitNo;
        // Calculate effective exception priority for this external interrupt,
        // including PRIGROUP etc.
        bool intrIsSecure = _ExceptionTargetsSecure(16 + intrNo, false/*doesn't matter*/);
        int  intrPrio     = _ExceptionPriority(16 + intrNo, intrIsSecure, /*applyPrigroup=*/true);
        if (intrPrio < maxPrio) {
          maxPrio     = intrPrio;
          maxPrioExc  = 16 + intrNo;
          excIsSecure = intrIsSecure;
        }
        // Mask off this bit from our ISPR mask to see if any other interrupts
        // are pending in this ISPR register by repeating CTZL above.
        v &= ~BIT(bitNo);
      }
    }

    return {maxPrio, maxPrioExc, excIsSecure};
  }

  int _RawExecutionPriority() {
    int execPri = _HighestPri();
    for (int i=2; i<=_MaxExceptionNum(); ++i)
      for (int j=0; j<2; ++j) {
        bool secure = !j;
        if (_IsActiveForState(i, secure)) {
          int effectivePriority = _ExceptionPriority(i, secure, true);
          if (effectivePriority < execPri)
            execPri = effectivePriority;
        }
      }

    return execPri;
  }

  int _HighestPri() {
    return 256;
  }

  int _RestrictedNSPri() {
    return 0x80;
  }

  bool _FindPriv() {
    return _CurrentModeIsPrivileged();
  }

  ExcInfo _ExceptionEntry(int excType, bool toSecure, bool instExecOk) {
    ExcInfo exc = _PushStack(toSecure, instExecOk);
    if (exc.fault == NoFault)
      exc = _ExceptionTaken(excType, false, toSecure, false);
    return exc;
  }

  ExcInfo _PushStack(bool secureExc, bool instExecOk) {
    auto &control = _IsSecure() ? _s.controlS : _s.controlNS;

    uint32_t frameSize;
    if (_HaveFPExt() && GETBITSM(control, CONTROL__FPCA) && (_IsSecure() || GETBITSM(InternalLoad32(REG_NSACR), REG_NSACR__CP(10)))) {
      if (_IsSecure() && GETBITSM(InternalLoad32(REG_FPCCR_S), REG_FPCCR__TS))
        frameSize = 0xA8;
      else
        frameSize = 0x68;
    } else
      frameSize = 0x20;

    bool framePtrAlign = GETBIT(_GetSP(), 2);
    uint32_t framePtr = (_GetSP() - frameSize) & ~BIT(2);
    RName spName = _LookUpSP();

    auto [retAddr, itState] = _ReturnState(instExecOk);
    uint32_t retpsr = _s.xpsr;
    retpsr = CHGBITSM(retpsr, RETPSR__IT_ICI_LO, itState>>2);
    retpsr = CHGBITSM(retpsr, RETPSR__IT_ICI_HI, itState);
    retpsr = CHGBITSM(retpsr, RETPSR__SPREALIGN, framePtrAlign);
    retpsr = CHGBITSM(retpsr, RETPSR__SFPA, _IsSecure() ? GETBITSM(_s.controlS, CONTROL__SFPA) : 0);

    PEMode mode = _CurrentMode();
    ExcInfo exc;
    if (1)                    exc = _Stack(framePtr, 0x00, spName, mode, _GetR( 0));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x04, spName, mode, _GetR( 1));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x08, spName, mode, _GetR( 2));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x0C, spName, mode, _GetR( 3));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x10, spName, mode, _GetR(12));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x14, spName, mode, _GetLR());
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x18, spName, mode, retAddr);
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x1C, spName, mode, retpsr);

    if (_HaveFPExt() && GETBITSM(control, CONTROL__FPCA)) {

    }

    ExcInfo spExc = _SetSP(spName, true, framePtr);
    exc = _MergeExcInfo(exc, spExc);

    bool isSecure = _IsSecure();
    bool isThread = (mode == PEMode_Thread);
    if (_HaveFPExt())
      _SetLR(BITS(7,31) | (uint32_t(isSecure)<<6) | (uint32_t(isThread)<<3) | 0b100000 | ((GETBITSM(control, CONTROL__FPCA)^1)<<4));
    else
      _SetLR(BITS(7,31) | (uint32_t(isSecure)<<6) | (uint32_t(isThread)<<3) | 0b110000);

    return exc;
  }

  ExcInfo _MergeExcInfo(const ExcInfo &a, const ExcInfo &b) {
    ExcInfo exc, pend;

    if (b.fault == NoFault || (a.isTerminal && !b.isTerminal))
      exc = a;
    else if (a.fault == NoFault || (b.isTerminal && !a.isTerminal))
      exc = b;
    else if (a.fault == b.fault && a.isSecure == b.isSecure)
      exc = a;
    else {
      int aPri = _ExceptionPriority(a.fault, a.isSecure, false);
      int bPri = _ExceptionPriority(b.fault, b.isSecure, false);

      if (aPri < bPri) {
        exc  = a;
        pend = b;
      } else {
        exc  = b;
        pend = a;
      }

      if (IMPL_DEF_OVERRIDDEN_EXCEPTIONS_PENDED)
        _SetPending(pend.fault, pend.isSecure, true);
    }

    return exc;
  }

  std::tuple<uint32_t, uint8_t> _ReturnState(bool instExecOk) {
    if (instExecOk)
      return {_NextInstrAddr(), _NextInstrITState()};
    else
      return {_ThisInstrAddr(), _ThisInstrITState()};
  }

  void _DerivedLateArrival(int pePriority, int peNumber, bool peIsSecure, const ExcInfo &deInfo, int oeNumber, bool oeIsSecure) {
    int oePriority = _ExceptionPriority(oeNumber, oeIsSecure, false);

    bool deIsDbgMonFault = _HaveMainExt() ? deInfo.origFault == DebugMonitor : false;

    bool targetIsSecure;
    int targetFault;
    if (deInfo.isTerminal) {
      targetIsSecure  = deInfo.isSecure;
      targetFault     = deInfo.fault;
      if (!_ComparePriorities(deInfo, false, oePriority, oeNumber, oeIsSecure)) {
        _ActivateException(oeNumber, oeIsSecure);
        _Lockup(true);
      }
    } else if (deIsDbgMonFault && !_ComparePriorities(deInfo, true, pePriority, peNumber, peIsSecure)) {
      _SetPending(DebugMonitor, deInfo.isSecure, false);
      targetFault = oeNumber;
      targetIsSecure = oeIsSecure;
    } else if (_ComparePriorities(deInfo, false, oePriority, oeNumber, oeIsSecure)) {
      targetFault = deInfo.fault;
      targetIsSecure = deInfo.isSecure;
    } else {
      if (deInfo.lockup) {
        _ActivateException(oeNumber, oeIsSecure);
        _Lockup(true);
      } else {
        targetFault = oeNumber;
        targetIsSecure = oeIsSecure;
      }
    }

    if (_HaveMainExt() && deInfo.fault == HardFault && deInfo.origFault != HardFault)
      InternalOr32(REG_HFSR, REG_HFSR__FORCED);

    _SetPending(deInfo.fault, deInfo.isSecure, true);
    ExcInfo excInfo = _ExceptionTaken(targetFault, deInfo.inExcTaken, targetIsSecure, true);
    if (excInfo.fault != NoFault)
      _DerivedLateArrival(pePriority, peNumber, peIsSecure, excInfo, targetFault, targetIsSecure);
  }

  bool _ComparePriorities(int exc0Pri, int exc0Number, bool exc0IsSecure,
                          int exc1Pri, int exc1Number, bool exc1IsSecure) {
    bool takeE0;
    if (exc0Pri != exc1Pri)
      takeE0 = (exc0Pri < exc1Pri);
    else if (exc0Number != exc1Number)
      takeE0 = (exc0Number < exc1Number);
    else if (exc0IsSecure != exc1IsSecure)
      takeE0 = exc0IsSecure;
    else
      takeE0 = false;
    return takeE0;
  }

  bool _ComparePriorities(const ExcInfo &exc0Info, bool groupPri, int exc1Pri, int exc1Number, bool exc1IsSecure) {
    int exc0Pri = _ExceptionPriority(exc0Info.fault, exc0Info.isSecure, groupPri);
    return _ComparePriorities(exc0Pri, exc0Info.fault, exc0Info.isSecure, exc1Pri, exc1Number, exc1IsSecure);
  }

  void _ActivateException(int excNo, bool excIsSecure) {
    _s.curState = excIsSecure ? SecurityState_Secure : SecurityState_NonSecure;
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__EXCEPTION, excNo);
    if (_HaveMainExt())
      _SetITSTATE(0);
    uint32_t &control = _IsSecure() ? _s.controlS : _s.controlNS;
    if (_HaveFPExt()) {
      control = CHGBITSM(control, CONTROL__FPCA, 0);
      _s.controlS = CHGBITSM(_s.controlS, CONTROL__SFPA, 0);
    }
    control = CHGBITSM(control, CONTROL__SPSEL, 0);

    _SetPending(excNo, excIsSecure, false);
    _SetActive(excNo, excIsSecure, true);
  }

  void _SetActive(int exc, bool isSecure, bool setNotClear) {
    if (!_HaveSecurityExt())
      isSecure = false;

    if (_IsExceptionTargetConfigurable(exc)) {
      if (_ExceptionTargetsSecure(exc, false/*UNKNOWN*/) == isSecure)
        _s.excActive[exc] = setNotClear ? 0b11 : 0b00;
    } else {
      uint32_t idx = isSecure ? 0 : 1;
      _s.excActive[exc] = CHGBITS(_s.excActive[exc], idx, idx, setNotClear ? 1 : 0);
    }
  }

  ExcInfo _TailChain(int excNo, bool excIsSecure, uint32_t excReturn) {
    if (!_HaveFPExt())
      excReturn = CHGBITSM(excReturn, EXC_RETURN__FTYPE, 1);
    excReturn = CHGBITSM(excReturn, EXC_RETURN__PREFIX, 0xFF);
    _SetLR(excReturn);

    return _ExceptionTaken(excNo, true, excIsSecure, false);
  }

  void _ConsumeExcStackFrame(uint32_t excReturn, bool fourByteAlign) {
    bool toSecure = _HaveSecurityExt() && !!(excReturn & BIT(6));
    uint32_t frameSize;
    if (toSecure && (!GETBITSM(excReturn, EXC_RETURN__ES) || !GETBITSM(excReturn, EXC_RETURN__DCRS)))
      frameSize = 0x48;
    else
      frameSize = 0x20;

    if (_HaveFPExt() && !GETBITSM(excReturn, EXC_RETURN__FTYPE)) {
      if (toSecure && (InternalLoad32(REG_FPCCR_S) & REG_FPCCR__TS))
        frameSize = frameSize + 0x88;
      else
        frameSize = frameSize + 0x48;
    }

    PEMode mode = GETBITSM(excReturn, EXC_RETURN__MODE) == 1 ? PEMode_Thread : PEMode_Handler;
    RName spName = _LookUpSP_with_security_mode(toSecure, mode);
    _s.r[spName] = (_GetSP(spName) + frameSize) | (fourByteAlign ? 0b100 : 0);
  }

  std::tuple<ExcInfo, uint32_t> _ExceptionReturn(uint32_t excReturn) {
    int returningExcNo = GETBITSM(_s.xpsr, XPSR__EXCEPTION);

    ExcInfo exc;
    std::tie(exc, excReturn) = _ValidateExceptionReturn(excReturn, returningExcNo);
    if (exc.fault != NoFault)
      return {exc, excReturn};

    bool excSecure, retToSecure;
    if (_HaveSecurityExt()) {
      excSecure   = !!GETBITSM(excReturn, EXC_RETURN__ES);
      retToSecure = !!GETBITSM(excReturn, EXC_RETURN__S);
    } else {
      excSecure   = false;
      retToSecure = false;
    }

    if (excSecure)
      _s.controlS = CHGBITSM(_s.controlS, CONTROL__SPSEL, GETBITSM(excReturn, EXC_RETURN__SPSEL));
    else
      _s.controlNS = CHGBITSM(_s.controlNS, CONTROL__SPSEL, GETBITSM(excReturn, EXC_RETURN__SPSEL));

    bool targetDomainSecure = !!GETBITSM(excReturn, EXC_RETURN__ES);
    _DeActivate(returningExcNo, targetDomainSecure);

    auto &control = _IsSecure() ? _s.controlS : _s.controlNS;
    if (_HaveFPExt() && (InternalLoad32(REG_FPCCR) & REG_FPCCR__CLRONRET) && (control & CONTROL__FPCA)) {
      if (InternalLoad32(REG_FPCCR_S) & REG_FPCCR__LSPACT) {
        InternalOr32(REG_SFSR, REG_SFSR__LSERR);
        exc = _CreateException(SecureFault, true, true);
        return {exc, excReturn};
      } else {
        for (int i=0; i<16; ++i)
          _SetS(i,0);
        _s.fpscr = 0;
      }
    }

    if (IMPL_DEF_TAIL_CHAINING_SUPPORTED) {
      auto [takeException, exc2, excIsSecure] = _PendingExceptionDetails();
      if (takeException) {
        exc = _TailChain(exc2, excIsSecure, excReturn);
        return {exc, excReturn};
      }
    }

    if (_HaveSecurityExt())
      _s.curState = retToSecure ? SecurityState_Secure : SecurityState_NonSecure;

    if (GETBITSM(excReturn, EXC_RETURN__MODE) && (InternalLoad32(REG_SCR) & REG_SCR__SLEEPONEXIT) && !_ExceptionActiveBitCount())
      _SleepOnExit();

    exc = _PopStack(excReturn);
    if (exc.fault == NoFault) {
      _ClearExclusiveLocal(_ProcessorID());
      _SetEventRegister();
      _InstructionSynchronizationBarrier(0b1111);
    }

    return {exc, excReturn};
  }

  int _ExceptionActiveBitCount() {
    int count = 0;
    for (int i=0; i<=_MaxExceptionNum(); ++i)
      for (int j=0; j<2; ++j)
        if (_IsActiveForState(i, !j))
          ++count;
    return count;
  }

  void _DeActivate(int returningExcNo, bool targetDomainSecure) {
    int rawPri = _RawExecutionPriority();
    if (rawPri == -1)
      _SetActive(HardFault, !(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS), false);
    else if (rawPri == -2)
      _SetActive(NMI,       !(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS), false);
    else if (rawPri == -3)
      _SetActive(HardFault,  true, false);
    else {
      bool secure = _HaveSecurityExt() && targetDomainSecure;
      _SetActive(returningExcNo, secure, false);
    }

    if (_HaveMainExt() && rawPri >= 0) {
      if (_HaveSecurityExt() && targetDomainSecure)
        _s.faultmaskS &= ~1;
      else
        _s.faultmaskNS &= ~1;
    }
  }

  void _SleepOnExit() {
    // TODO
  }

  bool _IsIrqValid(int e) {
    return true; // TODO
  }

  ExcInfo _PopStack(uint32_t excReturn) {
    PEMode    mode      = (GETBITSM(excReturn, EXC_RETURN__MODE) ? PEMode_Thread : PEMode_Handler);
    bool      toSecure  = _HaveSecurityExt() && GETBITSM(excReturn, EXC_RETURN__S);
    RName     spName    = _LookUpSP_with_security_mode(toSecure, mode);
    uint32_t  framePtr  = _GetSP(spName);
    if (!_IsAligned(framePtr, 8))
      throw Exception(ExceptionType::UNPREDICTABLE);

    uint32_t tmp;
    ExcInfo exc = _DefaultExcInfo();
    if (toSecure && (!GETBITSM(excReturn, EXC_RETURN__ES) || !GETBITSM(excReturn, EXC_RETURN__DCRS))) {
      uint32_t expectedSig = 0xFEFA'125B;
      if (_HaveFPExt())
        expectedSig = CHGBITS(expectedSig, 0, 0, GETBITSM(excReturn, EXC_RETURN__FTYPE));
      uint32_t integritySig;
      std::tie(exc, integritySig) = _Stack(framePtr, 0, spName, mode);
      if (exc.fault == NoFault && integritySig != expectedSig) {
        if (_HaveMainExt())
          InternalOr32(REG_SFSR, REG_SFSR__INVIS);
        return _CreateException(SecureFault, true, true);
      }

      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x08, spName, mode); _SetR( 4, tmp); }
      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x0C, spName, mode); _SetR( 5, tmp); }
      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x10, spName, mode); _SetR( 6, tmp); }
      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x14, spName, mode); _SetR( 7, tmp); }
      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x18, spName, mode); _SetR( 8, tmp); }
      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x1C, spName, mode); _SetR( 9, tmp); }
      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x20, spName, mode); _SetR(10, tmp); }
      if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x24, spName, mode); _SetR(11, tmp); }
      framePtr += 0x28;
    }

    uint32_t pc, psr;
    if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x00, spName, mode); _SetR( 0, tmp); }
    if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x04, spName, mode); _SetR( 1, tmp); }
    if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x08, spName, mode); _SetR( 2, tmp); }
    if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x0C, spName, mode); _SetR( 3, tmp); }
    if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x10, spName, mode); _SetR(12, tmp); }
    if (exc.fault == NoFault) { std::tie(exc, tmp) = _Stack(framePtr, 0x14, spName, mode); _SetLR(tmp); }
    if (exc.fault == NoFault) { std::tie(exc, pc ) = _Stack(framePtr, 0x18, spName, mode); }
    if (exc.fault == NoFault) { std::tie(exc, psr) = _Stack(framePtr, 0x1C, spName, mode); }
    _BranchToAndCommit(pc);

    uint32_t excNo = GETBITSM(psr, XPSR__EXCEPTION);
    if (exc.fault == NoFault && (mode == PEMode_Handler) == !excNo) {
      if (_HaveMainExt())
        InternalOr32(REG_UFSR, REG_UFSR__INVPC);
      return _CreateException(UsageFault, false, false/*UNKNOWN*/);
    }

    bool validIPSR = (excNo == 0 || excNo == 1 || excNo == NMI || excNo == HardFault || excNo == SVCall || excNo == PendSV || excNo == SysTick);
    if (!validIPSR && _HaveMainExt())
      validIPSR = (excNo == MemManage || excNo == BusFault || excNo == UsageFault || excNo == SecureFault || excNo == DebugMonitor);

    if (!validIPSR && !_IsIrqValid(excNo))
      psr = CHGBITSM(psr, XPSR__EXCEPTION, 0); // UNKNOWN

    if (_HaveFPExt()) {
      if (!GETBITSM(excReturn, EXC_RETURN__FTYPE)) {
        if (!toSecure && (InternalLoad32(REG_FPCCR_S) & REG_FPCCR__LSPACT)) {
          InternalOr32(REG_SFSR, REG_SFSR__LSERR);
          ExcInfo newExc = _CreateException(SecureFault, true, true);
          if (IMPL_DEF_DROP_PREV_GEN_EXC)
            exc = newExc;
          else
            exc = _MergeExcInfo(exc, newExc);
        } else {
          bool lspAct = !!(toSecure ? InternalLoad32(REG_FPCCR_S) & REG_FPCCR__LSPACT : InternalLoad32(REG_FPCCR_NS) & REG_FPCCR__LSPACT);
          if (lspAct) {
            if (exc.fault == NoFault) {
              if (toSecure)
                InternalMask32(REG_FPCCR_S, REG_FPCCR__LSPACT);
              else
                InternalMask32(REG_FPCCR_NS, REG_FPCCR__LSPACT);
            }
          } else {
            if (exc.fault == NoFault) {
              bool nPriv  = toSecure ? GETBITSM(_s.controlS, CONTROL__NPRIV) : GETBITSM(_s.controlNS, CONTROL__NPRIV);
              bool isPriv = (mode == PEMode_Handler || !nPriv);
              exc = _CheckCPEnabled(10, isPriv, toSecure);
            }

            if (exc.fault == NoFault) {
              for (int i=0; i<16; ++i)
                if (exc.fault == NoFault) {
                  uint32_t offset = 0x20+(4*i);
                  uint32_t tmp;
                  std::tie(exc, tmp) = _Stack(framePtr, offset, spName, mode);
                  _SetS(i, tmp);
                }
              if (exc.fault == NoFault) {
                uint32_t tmp;
                std::tie(exc, tmp) = _Stack(framePtr, 0x60, spName, mode);
                _s.fpscr = tmp;
              }
              if (toSecure && (InternalLoad32(REG_FPCCR_S) & REG_FPCCR__TS)) {
                for (int i=0; i<16; ++i)
                  if (exc.fault == NoFault) {
                    uint32_t offset = 0x68+(4*i);
                    uint32_t tmp;
                    std::tie(exc, tmp) = _Stack(framePtr, offset, spName, mode);
                    _SetS(i+16, tmp);
                  }

                if (exc.fault != NoFault)
                  for (int i=16; i<32; ++i)
                    _SetS(i, _HaveSecurityExt() ? 0 : 0 /* UNKNOWN */);
              }
              if (exc.fault != NoFault) {
                for (int i=0; i<16; ++i)
                  _SetS(i, _HaveSecurityExt() ? 0 : 0 /* UNKNOWN */);
                _s.fpscr = (_HaveSecurityExt() ? 0 : 0 /* UNKNOWN */);
              }
            }
          }
        }
      }

      auto &control = _IsSecure() ? _s.controlS : _s.controlNS;
      control = CHGBITSM(control, CONTROL__FPCA, GETBITSM(excReturn, EXC_RETURN__FTYPE) ^ 1);
    }

    if (exc.fault == NoFault)
      _ConsumeExcStackFrame(excReturn, GETBITSM(psr, RETPSR__SPREALIGN));

    if (_HaveDSPExt())
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__GE, GETBITSM(psr, XPSR__GE));

    if (_IsSecure())
      _s.controlS = CHGBITSM(_s.controlS, CONTROL__SFPA, GETBITSM(psr, RETPSR__SFPA));

    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__EXCEPTION, GETBITSM(psr, XPSR__EXCEPTION));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__T,         GETBITSM(psr, XPSR__T));
    if (_HaveMainExt()) {
      _s.xpsr = CHGBITS(_s.xpsr,27,31,GETBITS(psr,27,31));

      uint32_t it = (GETBITSM(psr, XPSR__IT_ICI_LO)<<2) | GETBITSM(psr, XPSR__IT_ICI_HI);
      _SetITSTATEAndCommit(it);
    } else
      _s.xpsr = CHGBITS(_s.xpsr,28,31,GETBITS(psr,28,31));

    return exc;
  }

  ExcInfo _CheckCPEnabled(int cp) {
    return _CheckCPEnabled(cp, _CurrentModeIsPrivileged(), _IsSecure());
  }

  ExcInfo _CheckCPEnabled(int cp, bool priv, bool secure) {
    auto [enabled, toSecure] = _IsCPEnabled(cp, priv, secure);
    ExcInfo excInfo;
    if (!enabled) {
      if (toSecure)
        InternalOr32(REG_UFSR_S, REG_UFSR__NOCP);
      else
        InternalOr32(REG_UFSR_NS, REG_UFSR__NOCP);
      excInfo = _CreateException(UsageFault, true, toSecure);
    } else
      excInfo = _DefaultExcInfo();
    return excInfo;
  }

  std::tuple<ExcInfo, uint32_t> _ValidateExceptionReturn(uint32_t excReturn, int retExcNo) {
    bool error = false;
    assert(_CurrentMode() == PEMode_Handler);
    if (GETBITS(excReturn, 7,23) != BITS(0,16) || GETBITS(excReturn, 1, 1))
      throw Exception(ExceptionType::UNPREDICTABLE);
    if (!_HaveFPExt() && !GETBITSM(excReturn, EXC_RETURN__FTYPE))
      throw Exception(ExceptionType::UNPREDICTABLE);

    bool targetDomainSecure = !!GETBITSM(excReturn, EXC_RETURN__ES);
    bool excStateNonSecure;
    int excNo;
    if (_HaveSecurityExt()) {
      excStateNonSecure = (_s.curState == SecurityState_NonSecure || !targetDomainSecure);
      if (excStateNonSecure && (!GETBITSM(excReturn, EXC_RETURN__DCRS) || targetDomainSecure)) {
        if (_HaveMainExt())
          InternalOr32(REG_SFSR, REG_SFSR__INVER);

        if (excStateNonSecure && targetDomainSecure)
          excReturn = CHGBITSM(excReturn, EXC_RETURN__ES, 0);

        targetDomainSecure = false;
        error = true;
        excNo = SecureFault;
      }
    } else
      excStateNonSecure = true;

    if (!error)
      if (!_IsActiveForState(retExcNo, targetDomainSecure)) {
        error = true;
        if (_HaveMainExt()) {
          InternalOr32(REG_UFSR, REG_UFSR__INVPC);
          excNo = UsageFault;
        } else
          excNo = HardFault;
      }

    ExcInfo excInfo;
    if (error) {
      _DeActivate(retExcNo, targetDomainSecure);
      if (_HaveSecurityExt() && targetDomainSecure)
        _s.controlS = CHGBITSM(_s.controlS, CONTROL__SPSEL, GETBITSM(excReturn, EXC_RETURN__SPSEL));
      else
        _s.controlNS = CHGBITSM(_s.controlNS, CONTROL__SPSEL, GETBITSM(excReturn, EXC_RETURN__SPSEL));
      excInfo = _CreateException(excNo, false, false /*UNKNOWN*/);
    } else
      excInfo = _DefaultExcInfo();

    return {excInfo, excReturn};
  }

  ExcInfo _ExceptionTaken(int excNo, bool doTailChain, bool excIsSecure, bool ignStackFaults) {
    assert(_HaveSecurityExt() || !excIsSecure);

    ExcInfo exc = _DefaultExcInfo();
    if (_HaveSecurityExt() && GETBIT(_GetLR(), 6)) {
      if (excIsSecure) {
        if (doTailChain && !GETBIT(_GetLR(), 0))
          _SetLR(CHGBITS(_GetLR(), 5, 5, 0));
      } else {
        if (GETBIT(_GetLR(), 5) && !(doTailChain && !GETBIT(_GetLR(),0)))
          exc = _PushCalleeStack(doTailChain);
        _SetLR(CHGBITS(_GetLR(), 5, 5, 1));
      }
    }

    if (excIsSecure)
      _SetLR(CHGBITS(CHGBITS(_GetLR(), 2, 2, GETBITSM(_s.controlS, CONTROL__SPSEL)), 0, 0, 1));
    else
      _SetLR(CHGBITS(CHGBITS(_GetLR(), 2, 2, GETBITSM(_s.controlNS, CONTROL__SPSEL)), 0, 0, 0));

    uint32_t callerRegValue = (!_HaveSecurityExt() || excIsSecure) ? 0 /*UNKNOWN*/ : 0;
    for (int n=0; n<4; ++n)
      _SetR(n, callerRegValue);
    _SetR(12, callerRegValue);
    _s.xpsr = (callerRegValue & ~XPSR__EXCEPTION) | (_s.xpsr & XPSR__EXCEPTION);

    if (_HaveSecurityExt() && GETBIT(_GetLR(), 6)) {
      if (excIsSecure) {
        if (!GETBIT(_GetLR(),5))
          for (int n=4; n<12; ++n)
            _SetR(n, 0/*UNKNOWN*/);
      } else
        for (int n=4; n<12; ++n)
          _SetR(n, 0);
    }

    uint32_t start;
    if (exc.fault == NoFault || ignStackFaults)
      std::tie(exc, start) = _GetVector(excNo, excIsSecure);

    if (exc.fault == NoFault) {
      _ActivateException(excNo, excIsSecure);
      _SCS_UpdateStatusRegs();
      _ClearExclusiveLocal(_ProcessorID());
      _SetEventRegister();
      _InstructionSynchronizationBarrier(0b1111);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__T, start & 1);
      _BranchTo(start & ~1);
    } else
      exc.inExcTaken = true;

    return exc;
  }

  ExcInfo _PushCalleeStack(bool doTailChain) {
    PEMode mode;
    RName spName;
    if (doTailChain) {
      if (!GETBIT(_GetLR(), 3)) {
        mode = PEMode_Handler;
        spName = RNameSP_Main_Secure;
      } else {
        mode = PEMode_Thread;
        spName = (GETBIT(_GetLR(), 2) ? RNameSP_Process_Secure : RNameSP_Main_Secure);
      }
    } else {
      spName = _LookUpSP();
      mode = _CurrentMode();
    }

    uint32_t framePtr = _GetSP(spName) - 0x28;

    uint32_t integritySig = _HaveFPExt() ? CHGBITS(0xFEFA125A,0,0,GETBIT(_GetLR(),4)) : 0xFEFA125B;
    ExcInfo exc = _Stack(framePtr, 0x00, spName, mode, integritySig);

    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x08, spName, mode, _GetR( 4));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x0C, spName, mode, _GetR( 5));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x10, spName, mode, _GetR( 6));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x14, spName, mode, _GetR( 7));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x18, spName, mode, _GetR( 8));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x1C, spName, mode, _GetR( 9));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x20, spName, mode, _GetR(10));
    if (exc.fault == NoFault) exc = _Stack(framePtr, 0x24, spName, mode, _GetR(11));

    ExcInfo spExc = _SetSP(spName, true, framePtr);
    return _MergeExcInfo(exc, spExc);
  }

  void _SCS_UpdateStatusRegs() {
    // TODO
  }

  bool _ConstrainUnpredictableBool(bool x) { return x; }

  int _ExceptionPriority(int n, bool isSecure, bool groupPri) {
    if (_HaveMainExt())
      assert(n >= 1 && n <= 511);
    else
      assert(n >= 1 && n <= 48);

    int result;
    if (n == Reset)
      result = -4;
    else if (n == NMI)
      result = -2;
    else if (n == HardFault) {
      if (isSecure && (InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS))
        result = -3;
      else
        result = -1;
    } else if (_HaveMainExt() && n == MemManage)
      result = isSecure ? GETBITSM(InternalLoad32(REG_SHPR1_S), REG_SHPR1__PRI_4) : GETBITSM(InternalLoad32(REG_SHPR1_NS), REG_SHPR1__PRI_4);
    else if (_HaveMainExt() && n == BusFault)
      result = GETBITSM(InternalLoad32(REG_SHPR1_S), REG_SHPR1__PRI_5);
    else if (_HaveMainExt() && n == UsageFault)
      result = isSecure ? GETBITSM(InternalLoad32(REG_SHPR1_S), REG_SHPR1__PRI_6) : GETBITSM(InternalLoad32(REG_SHPR1_NS), REG_SHPR1__PRI_6);
    else if (_HaveMainExt() && n == SecureFault)
      result = GETBITSM(InternalLoad32(REG_SHPR1_S), REG_SHPR1__PRI_7);
    else if (n == SVCall)
      result = isSecure ? GETBITSM(InternalLoad32(REG_SHPR2_S), REG_SHPR2__PRI_11) : GETBITSM(InternalLoad32(REG_SHPR2_NS), REG_SHPR2__PRI_11);
    else if (_HaveMainExt() && n == DebugMonitor)
      result = GETBITSM(InternalLoad32(REG_SHPR3_S), REG_SHPR3__PRI_12);
    else if (n == PendSV)
      result = isSecure ? GETBITSM(InternalLoad32(REG_SHPR3_S), REG_SHPR3__PRI_14) : GETBITSM(InternalLoad32(REG_SHPR3_NS), REG_SHPR3__PRI_14);
    else if (   n == SysTick
             && (   (_HaveSysTick() == 2)
                 || (_HaveSysTick() == 1 && (!(InternalLoad32(REG_ICSR_S) & REG_ICSR__STTNS)) == isSecure)))
      result = isSecure ? GETBITSM(InternalLoad32(REG_SHPR3_S), REG_SHPR3__PRI_15) : GETBITSM(InternalLoad32(REG_SHPR3_NS), REG_SHPR3__PRI_15);
    else if (n >= 16) {
      int r = (n-16)/4;
      int v = n%4;
      result = GETBITS(InternalLoad32(_IsSecure() ? REG_NVIC_IPRn_S(r) : REG_NVIC_IPRn_NS(r)), v*8, v*8+7);
    } else
      result = 256;

    if (result >= 0) {
      if (_HaveMainExt() && groupPri) {
        int subGroupShift;
        if (isSecure)
          subGroupShift = GETBITSM(InternalLoad32(REG_AIRCR_S), REG_AIRCR__PRIGROUP);
        else
          subGroupShift = GETBITSM(InternalLoad32(REG_AIRCR_NS), REG_AIRCR__PRIGROUP);
        int groupValue    = 2<<subGroupShift;
        int subGroupValue = result % groupValue;
        result = result - subGroupValue;
      }

      int priSNsPri = _RestrictedNSPri();
      if ((InternalLoad32(REG_AIRCR_S) & REG_AIRCR__PRIS) && !isSecure)
        result = (result>>1) + priSNsPri;
    }

    return result;
  }

private:
  CpuState _s{};
  CpuNest  _n{};
  IDevice &_dev;
};

/* _TopLevel {{{2
 * ---------
 * Called once for each tick the PE is not in a sleep state. Handles all
 * instruction processing, including fetching the opcode, decode and execute.
 * It also handles pausing execution when in the lockup state.
 */
void Emulator::_TopLevel() {
  // If the PE has locked up then abort execution of this instruction. Set
  // the length of the current instruction to 0 so NextInstrAddr() reports
  // the correct lockup address.
  TRACE("top-level begin\n");
  TRACE_BLOCK();
  bool ok = !GETBITSM(InternalLoad32(REG_DHCSR), REG_DHCSR__S_LOCKUP);
  if (!ok) {
    TRACE("locked up\n");
    _SetThisInstrDetails(0, 0, 0b1111);
  } else {
    ASSERT(!_s.pcChanged);

    // Check for stepping debug for current insn fetch.
    bool monStepActive = _SteppingDebug();
    _UpdateSecureDebugEnable();
    uint32_t pc = _ThisInstrAddr();

    uint32_t instr;
    bool is16bit;
    try {
      // Not locked up, so attempt to fetch the instruction.
      std::tie(instr, is16bit) = _FetchInstr(pc);
      TRACE("fetched %d-bit insn: 0x%08x\n", is16bit ? 16 : 32, instr);

      // Setup the details of the instruction. NOTE: The default condition
      // is based on the ITSTATE, however this is overridden in the decode
      // stage by instructions that have explicit condition codes.
      uint32_t len = is16bit ? 2 : 4;

      uint32_t defaultCond = !GETBITS(_GetITSTATE(),0,3) ? 0b1110 : GETBITS(_GetITSTATE(),4,7);
      _SetThisInstrDetails(instr, len, defaultCond);

      // Checking for FPB Breakpoint on instructions
      if (_HaveFPB() && _FPB_CheckBreakPoint(pc, len, true, _IsSecure()))
        _FPB_BreakpointMatch();

      // Finally try and execute the instruction.
      _DecodeExecute(instr, pc, is16bit);

      // Check for Monitor Step
      if (_HaveDebugMonitor())
        _SetMonStep(monStepActive);

      // Check for DWT match
      if (_IsDWTEnabled())
        _DWT_InstructionMatch(pc);

    } catch (Exception e) {
      if (_IsSEE(e) || _IsUNDEFINED(e)) {
        TRACE("top-level SEE/UD exception\n");
        // Unallocated instructions in the NOP hint space and instructions that
        // fail their condition tests are treated like NOPs.
        bool nopHint =
             (instr & 0b11111111111111111111111100001111U) == 0b00000000000000001011111100000000U
          || (instr & 0b11111111111111111111111100000000U) == 0b11110011101011111000000000000000U;
        if (_ConditionHolds(_CurrentCond()) && !nopHint) {
          ok = false;
          bool toSecure = _IsSecure();
          // Unallocated instructions in the coprocessor space behave as NOCP if the
          // coprocessor is disabled.
          auto [isCp, cpNum] = _IsCPInstruction(instr);
          if (isCp) {
            auto [cpEnabled, cpFaultState] = _IsCPEnabled(cpNum);
            if (!cpEnabled) {
              // A PE is permitted to decode the coprocessor space and raise
              // UNDEFINSTR UsageFaults for unallocated encodings even if the
              // coprocessor is disabled.
              if (IMPL_DEF_DECODE_CP_SPACE)
                InternalOr32(REG_UFSR, REG_UFSR__UNDEFINSTR);
              else {
                InternalOr32(REG_UFSR, REG_UFSR__NOCP);
                toSecure = cpFaultState;
              }
            }
          } else
            InternalOr32(REG_UFSR, REG_UFSR__UNDEFINSTR);

          // If Main Extension is not implemented the fault will escalate to a HardFault.
          ExcInfo excInfo = _CreateException(UsageFault, true, toSecure);

          // Prevent EndOfInstruction() being called in HandleInstruction() as the
          // instruction has already been terminated so there is no need to throw
          // the exception again.
          excInfo.termInst = false;
          _HandleException(excInfo);
        }
      } else if (_IsExceptionTaken(e)) { // XXX guessing this is EndOfInstruction
        TRACE("top-level EOI exception\n");
        ok = false;
      } else
        // Do not catch UNPREDICTABLE or internal errors
        throw;
    }
  }

  // SEE
  // UNDEFINED
  // Taken
  // UNPREDICTABLE
  // internal

  // If there is a reset pending do that, otherwise process the normal
  // instruction advance.
  try {
    if (_s.excPending[Reset]) {
      TRACE("top-level handling pending reset\n");
      _s.excPending[Reset] = 0;
      _TakeReset();
      TRACE("top-level done handling pending reset\n");
    } else {
      // Call instruction advance for exception handling and PC/ITSTATE
      // advance.
      TRACE("top-level advancing\n");
      _InstructionAdvance(ok);
      TRACE("top-level advance done\n");
    }

  } catch (Exception e) {
    TRACE("top-level reset/advance exception\n");

    // Do not catch UNPREDICTABLE or internal errors
    if (!_IsExceptionTaken(e))
      throw;

    // The correct architectural behaviour for any exceptions is performed
    // inside TakeReset() and InstructionAdvance(). So no additional actions
    // are required in this catch block.
  }
}

/* _EndOfInstruction
 * -----------------
 */
void Emulator::_EndOfInstruction() {
  throw Exception(ExceptionType::EndOfInstruction);
}

/* _CreateException
 * ----------------
 */
ExcInfo Emulator::_CreateException(int exc, bool forceSecurity, bool isSecure, bool isSync) {
  // Work out the effective target state of the exception.
  if (_HaveSecurityExt()) {
    if (!forceSecurity)
      isSecure = _ExceptionTargetsSecure(exc, _IsSecure());
    else
      isSecure = false;
  }

  // An implementation without Security Extensions cannot cause a fault targeting
  // Secure state.
  assert(_HaveSecurityExt() || !isSecure);

  // Get the remaining exception details.
  auto [escalateToHf, termInst] = _ExceptionDetails(exc, isSecure, isSync);

  // Fill in the default exception info.
  ExcInfo info            = _DefaultExcInfo();
  info.fault              = exc;
  info.termInst           = termInst;
  info.origFault          = exc;
  info.origFaultIsSecure  = isSecure;

  // Check for HardFault escalation.
  // NOTE: In some cases (for example faults during lazy floating-point state
  // preservation) the decision to escalate below is ignored and instead
  // based on the info.origFaults fields and other factors.
  if (escalateToHf && info.fault != HardFault) {
    // Update the exception info with the escalation details, including
    // whether there's a change in destination Security state.
    info.fault    = HardFault;
    isSecure      = _ExceptionTargetsSecure(HardFault, isSecure);
    bool dummy;
    std::tie(escalateToHf, dummy) = _ExceptionDetails(HardFault, isSecure, isSync);
  }

  // If the requested exception was already a HardFault then we can't
  // escalate to a HardFault, so lockup. NOTE: Async BusFaults never cause
  // lockups, if the BusFault is disabled it escalates to a HardFault that is
  // pended.
  if (escalateToHf && isSync && info.fault == HardFault)
    info.lockup = true;

  // Fill in the remaining exception info.
  info.isSecure = isSecure;
  return info;
}

void Emulator::_UpdateSecureDebugEnable() {
  /// DHCSR.S_SDE is frozen if the PE is in Debug state
  uint32_t dhcsr = InternalLoad32(REG_DHCSR);
  if (!GETBITSM(dhcsr, REG_DHCSR__S_HALT)) {
    dhcsr = CHGBITSM(dhcsr, REG_DHCSR__S_SDE, _SecureHaltingDebugAllowed());
    InternalStore32(REG_DHCSR, dhcsr);
  }

  uint32_t demcr = InternalLoad32(REG_DEMCR);
  if (_HaveDebugMonitor() && !_s.excActive[DebugMonitor] && !GETBITSM(demcr, REG_DEMCR__MON_PEND)) {
    demcr = CHGBITSM(demcr, REG_DEMCR__SDME, _SecureDebugMonitorAllowed());
    InternalStore32(REG_DEMCR, demcr);
  }
}

void Emulator::_TakeReset() {
  _s.curState = _HaveSecurityExt() ? SecurityState_Secure : SecurityState_NonSecure;

  _ResetSCSRegs(); // Catch-all function for System Control Space reset
  _s.xpsr = 0; // APSR is UNKNOWN UNPREDICTABLE, IPSR exception number is 0
  if (_HaveMainExt()) {
    _s.lr = 0xFFFF'FFFF;      // Preset to an illegal exception return value
    _SetITSTATEAndCommit(0);  // IT/ICI bits cleared
  } else
    _s.lr = 0xFFFF'FFFF;      // UNKNOWN

  // Reset priority boosting
  _s.primaskNS &= ~1;
  if (_HaveSecurityExt())
    _s.primaskS &= ~1;
  if (_HaveMainExt()) {
    _s.faultmaskNS &= ~1;
    _s.basepriNS = CHGBITS(_s.basepriNS,0,7,0);
    if (_HaveSecurityExt()) {
      _s.faultmaskS &= ~1;
      _s.basepriS = CHGBITS(_s.basepriS,0,7,0);
    }
  }

  // Initialize the Floating Point Extn
  if (_HaveFPExt()) {
    _s.controlS = CHGBITSM(_s.controlS, CONTROL__FPCA, 0); // FP inactive
    uint32_t fpdscrNS = InternalLoad32(REG_FPDSCR_NS);
    fpdscrNS = CHGBITSM(fpdscrNS, REG_FPDSCR__AHP,    0);
    fpdscrNS = CHGBITSM(fpdscrNS, REG_FPDSCR__DN,     0);
    fpdscrNS = CHGBITSM(fpdscrNS, REG_FPDSCR__FZ,     0);
    fpdscrNS = CHGBITSM(fpdscrNS, REG_FPDSCR__RMODE,  0);
    InternalStore32(REG_FPDSCR_NS, fpdscrNS);
    uint32_t fpccr = InternalLoad32(REG_FPCCR_S);
    fpccr = CHGBITSM(fpccr, REG_FPCCR__LSPEN, 1);
    InternalStore32(REG_FPCCR_S, fpccr);
    uint32_t fpccrNS = InternalLoad32(REG_FPCCR_NS);
    fpccrNS = CHGBITSM(fpccrNS, REG_FPCCR__ASPEN, 1);
    fpccrNS = CHGBITSM(fpccrNS, REG_FPCCR__LSPACT, 0);
    InternalStore32(REG_FPCCR_NS, fpccrNS);
    InternalStore32(REG_FPCAR_NS, 0); // UNKNOWN
    if (_HaveSecurityExt()) {
      _s.controlS = CHGBITSM(_s.controlS, CONTROL__SFPA, 0);
      uint32_t fpdscrS = InternalLoad32(REG_FPDSCR_S);
      fpdscrS = CHGBITSM(fpdscrS, REG_FPDSCR__AHP, 0);
      fpdscrS = CHGBITSM(fpdscrS, REG_FPDSCR__DN, 0);
      fpdscrS = CHGBITSM(fpdscrS, REG_FPDSCR__FZ, 0);
      fpdscrS = CHGBITSM(fpdscrS, REG_FPDSCR__RMODE, 0);
      InternalStore32(REG_FPDSCR_S, fpdscrS);
      uint32_t fpccr = InternalLoad32(REG_FPCCR_S);
      fpccr = CHGBITSM(fpccr, REG_FPCCR__LSPENS, 0);
      InternalStore32(REG_FPCCR_S, fpccr);
      uint32_t fpccrS = InternalLoad32(REG_FPCCR_S);
      fpccrS = CHGBITSM(fpccrS, REG_FPCCR__ASPEN, 1);
      fpccrS = CHGBITSM(fpccrS, REG_FPCCR__LSPACT, 0);
      InternalStore32(REG_FPCCR_S, fpccrS);
      InternalStore32(REG_FPCAR_S, 0); // UNKNOWN
    }
    for (int i=0; i<32; ++i)
      _SetS(i, 0); // UNKNOWN
  }

  for (int i=0; i<_MaxExceptionNum(); ++i) // All exceptions Inactive
    _s.excActive[i] = 0;
  _ClearExclusiveLocal(_ProcessorID());
  _ClearEventRegister();
  for (int i=0; i<13; ++i)
    _s.r[i] = 0; // UNKNOWN

  // Stack limit registers. It is IMPLEMENTATIOND EFINED how many bits
  // of these registers are writable. The following writes only affect the bits
  // that an implementation defines as writable.
  if (_HaveMainExt()) {
    _s.msplimNS = 0;
    _s.psplimNS = 0;
  }
  if (_HaveSecurityExt()) {
    _s.msplimS = 0;
    _s.psplimS = 0;
  }

  // Load the initial value of the stack pointer and the reset value from the
  // vector table. The order of the loads is IMPLEMENTATION DEFINED.
  auto [excSp, sp]      = _GetVector(0,     _HaveSecurityExt());
  auto [excRst, start]  = _GetVector(Reset, _HaveSecurityExt());
  if (excSp.fault != NoFault || excRst.fault != NoFault)
    _Lockup(true);

  // Initialize the stack pointers and start execution at the reset vector
  if (_HaveSecurityExt()) {
    _SetSP_Main_Secure(sp);
    _SetSP_Main_NonSecure(0); // UNKNOWN
    _SetSP_Process_Secure(0); // UNKNOWN
  } else
    _SetSP_Main_NonSecure(sp);

  // Begin Implementation-Specific Resets
  _NestReset(); // Implementation-Specific
  _s.curCondOverride = -1;
  // End Implementation-Specific Resets

  _SetSP_Process_NonSecure(0); // UNKNOWN
  _s.xpsr = CHGBITSM(_s.xpsr, XPSR__T, start & 1);
  _BranchToAndCommit(start & ~1);

  // This is not included in the psuedocode but is required else the first
  // instruction will be executed twice.
  _s.pcChanged       = false;
}

bool Emulator::_SteppingDebug() {
  // If halting debug is allowed and C_STEP is set, set C_HALT for the next instruction.
  uint32_t dhcsr = InternalLoad32(REG_DHCSR);
  if (_CanHaltOnEvent(_IsSecure()) && !!(dhcsr & REG_DHCSR__C_STEP)) {
    dhcsr |= REG_DHCSR__C_HALT;
    InternalStore32(REG_DHCSR, dhcsr);

    uint32_t dfsr = InternalLoad32(REG_DFSR);
    dfsr |= REG_DFSR__HALTED;
    InternalStore32(REG_DFSR, dfsr);
  }

  uint32_t demcr = InternalLoad32(REG_DEMCR);
  bool monStepEnabled = _HaveDebugMonitor() && _CanPendMonitorOnEvent(_IsSecure(), false);
  return monStepEnabled && !!(demcr & REG_DEMCR__MON_STEP);
}

std::tuple<uint32_t,bool> Emulator::_FetchInstr(uint32_t addr) {
  uint32_t sgOpcode = 0xE97F'E97F;

  SAttributes hw1Attr = _SecurityCheck(addr, true, _IsSecure());
  // Fetch the T16 instruction, or the first half of a T32.
  uint16_t hw1Instr = _GetMemI(addr);

  // If the T bit is clear then the instruction can't be decoded
  if (!GETBITSM(_s.xpsr, XPSR__T)) {
    // Attempted NS->S domain crossing with the T bit clear raise an INVEP
    // SecureFault
    ExcInfo excInfo;
    if (!_IsSecure() && !hw1Attr.ns) {
      uint32_t sfsr = InternalLoad32(REG_SFSR);
      sfsr |= REG_SFSR__INVEP;
      InternalStore32(REG_SFSR, sfsr);
      excInfo = _CreateException(SecureFault, true, true);
    } else {
      uint32_t ufsr = InternalLoad32(REG_UFSR);
      ufsr |= REG_UFSR__INVSTATE;
      InternalStore32(REG_UFSR, ufsr);
      excInfo = _CreateException(UsageFault, false, false/*unknown*/);
    }
    _HandleException(excInfo);
  }

  // Implementations are permitted to terminate the fetch process early if a
  // domain crossing is being attempted and the first 16bits of the opcode
  // isn't the first part of the SG instruction.
  if (IMPL_DEF_EARLY_SG_CHECK) {
    if (!_IsSecure() && !hw1Attr.ns && (hw1Instr != sgOpcode>>16)) {
      uint32_t sfsr = InternalLoad32(REG_SFSR);
      sfsr |= REG_SFSR__INVEP;
      InternalStore32(REG_SFSR, sfsr);
      ExcInfo excInfo = _CreateException(SecureFault, true, true);
      _HandleException(excInfo);
    }
  }

  // NOTE: Implementations are also permitted to terminate the fetch process
  // at this point with an UNDEFINSTR UsageFault if the first 16 bits are an
  // undefined T32 prefix.

  // If the data fetched is the top half of a T32 instruction, fetch the bottom
  // 16 bits.
  uint32_t instr;
  SAttributes hw2Attr;
  bool isT16 = GETBITS(hw1Instr,11,15) < 0b11101;
  if (isT16)
    instr = hw1Instr;
  else {
    hw2Attr = _SecurityCheck(addr+2, true, _IsSecure());
    // The following test covers 2 possible fault conditions:
    // 1) NS code branching to a T32 instruction where the first half is in
    //    NS memory, and the second half is in S memory
    // 2) NS code branching to a T32 instruction in S & NSC memory, but
    //    where the second half of the instruction is in NS memory
    if (!_IsSecure() && hw1Attr.ns != hw2Attr.ns) {
      uint32_t sfsr = InternalLoad32(REG_SFSR);
      sfsr |= REG_SFSR__INVEP;
      InternalStore32(REG_SFSR, sfsr);
      ExcInfo excInfo = _CreateException(SecureFault, true, true);
      _HandleException(excInfo);
    }

    // Fetch the second half of the TE2 instruction.
    instr = (uint32_t(hw1Instr)<<16) | _GetMemI(addr+2);
  }

  // Raise a fault if an otherwise valid NS->S transition that doesn't land on
  // an SG instruction.
  if (!_IsSecure() && !hw1Attr.ns && instr != sgOpcode) {
    uint32_t sfsr = InternalLoad32(REG_SFSR);
    sfsr |= REG_SFSR__INVEP;
    InternalStore32(REG_SFSR, sfsr);
    ExcInfo excInfo = _CreateException(SecureFault, true, true);
    _HandleException(excInfo);
  }

  return {instr, isT16};
}

bool Emulator::_GenerateDebugEventResponse() {
  if (_CanHaltOnEvent(_IsSecure())) {
    InternalOr32(REG_DFSR, REG_DFSR__BKPT);
    InternalOr32(REG_DHCSR, REG_DHCSR__C_HALT);
    return true;
  } else if (_HaveMainExt() && _CanPendMonitorOnEvent(_IsSecure(), true)) {
    InternalOr32(REG_DFSR, REG_DFSR__BKPT);
    InternalOr32(REG_DEMCR, REG_DEMCR__MON_PEND);
    ExcInfo excInfo = _CreateException(DebugMonitor, false, false/*UNKNOWN*/);
    _HandleException(excInfo);
    return true;
  } else
    return false;
}

bool Emulator::_FPB_CheckBreakPoint(uint32_t iaddr, int size, bool isIFetch, bool isSecure) {
  bool match = _FPB_CheckMatchAddress(iaddr);
  if (!match && size == 4 && _FPB_CheckMatchAddress(iaddr+2))
    match = _ConstrainUnpredictableBool(false/*Unpredictable_FPBreakpoint*/);
  return match;
}

bool Emulator::_FPB_CheckMatchAddress(uint32_t iaddr) {
  if (!(InternalLoad32(REG_FP_CTRL) & REG_FP_CTRL__ENABLE))
    return false; // FPB not enabled

  // Instruction Comparator.
  uint32_t fpCtrl = InternalLoad32(REG_FP_CTRL);
  uint32_t numAddrCmp = GETBITSM(fpCtrl, REG_FP_CTRL__NUM_CODE_LO) | (GETBITSM(fpCtrl, REG_FP_CTRL__NUM_CODE_HI)<<4);
  if (!numAddrCmp)
    return false; // No comparator support

  for (int N=0; N<numAddrCmp; ++N) {
    uint32_t x = InternalLoad32(REG_FP_COMP(N));
    if (x & REG_FP_COMPn__BE) // Breakpoint enabled
      if ((iaddr>>1) == GETBITSM(x, REG_FP_COMPn__BPADDR))
        return true;
  }

  return false;
}

std::tuple<bool,bool> Emulator::_ExceptionDetails(int exc, bool isSecure, bool isSync) {
  bool escalateToHf, termInst, enabled, canEscalate;

  switch (exc) {
    case HardFault:
      termInst = true;
      enabled = true;
      canEscalate = true;
      break;
    case MemManage:
      termInst = true;
      if (_HaveMainExt()) {
        uint32_t val = isSecure ? InternalLoad32(REG_SHCSR_S) : InternalLoad32(REG_SHCSR_NS);
        enabled = !!(val & REG_SHCSR__MEMFAULTENA);
      } else
        enabled = false;
      canEscalate = true;
      break;
    case BusFault:
      termInst = isSync;
      enabled = _HaveMainExt() ? !!(InternalLoad32(REG_SHCSR_S) & REG_SHCSR__BUSFAULTENA) : false;
      canEscalate = termInst || !enabled; // Async BusFaults only escalate if they are disabled
      break;
    case UsageFault:
      termInst = true;
      if (_HaveMainExt()) {
        uint32_t val = isSecure ? InternalLoad32(REG_SHCSR_S) : InternalLoad32(REG_SHCSR_NS);
        enabled = !!(val & REG_SHCSR__USGFAULTENA);
      } else
        enabled = false;
      canEscalate = true;
      break;
    case SecureFault:
      termInst = true;
      enabled = _HaveMainExt() ? !!(InternalLoad32(REG_SHCSR_S) & REG_SHCSR__SECUREFAULTENA) : false;
      canEscalate = true;
      break;
    case SVCall:
      termInst = false;
      enabled  = true;
      canEscalate = true;
      break;
    case DebugMonitor:
      termInst = true;
      enabled = _HaveMainExt() ? !!(InternalLoad32(REG_DEMCR) & REG_DEMCR__MON_EN) : false;
      canEscalate = false; // true if fault caused by BKPT instruction
      break;
    default:
      termInst = false;
      canEscalate = false;
      break;
  }

  // If the fault can escalate then check if exception can be taken
  // immediately, or whether it should escalate.
  escalateToHf = false;
  if (canEscalate) {
    int execPri = _ExecutionPriority();
    int excePri = _ExceptionPriority(exc, isSecure, true);
    if (excePri >= execPri || !enabled)
      escalateToHf = true;
  }

  return {escalateToHf, termInst};
}

void Emulator::_HandleException(const ExcInfo &excInfo) {
  if (excInfo.fault == NoFault)
    return;

  TRACE("handling exception %d\n", excInfo.fault);

  if (excInfo.lockup) {
    TRACE("commencing lockup\n");
    _Lockup(excInfo.termInst);
    return;
  }

  // If the fault escalated to a HardFault update the syndrome info
  if (_HaveMainExt() && excInfo.fault == HardFault && excInfo.origFault != HardFault)
    InternalOr32(REG_HFSR, REG_HFSR__FORCED);

  // If the exception does not cause a lockup set the exception pending and
  // potentially terminate execution of the current instruction
  _SetPending(excInfo.fault, excInfo.isSecure, true);
  if (excInfo.termInst)
    _EndOfInstruction();
}

void Emulator::_InstructionAdvance(bool instExecOk) {
  // Check for, and process any exception returns that were requested. This
  // must be done after the instruction has completed so any exceptions raised
  // during the exception return do not interfere with the execution of the
  // instruction that causes the exception return (e.g. a POP causing an
  // excReturn value to be written to the PC must adjust SP even if the
  // exception return caused by the POP raises a fault).
  bool excRetFault = false;
  uint32_t excReturn = _NextInstrAddr();
  if (_s.pendingReturnOperation) {
    _s.pendingReturnOperation = false;
    ExcInfo excInfo;
    std::tie(excInfo, excReturn) = _ExceptionReturn(excReturn);
    // Handle any faults raised during exception return
    if (excInfo.fault != NoFault) {
      excRetFault = true;
      // Either lockup, or pend the fault if it can be taken
      if (excInfo.lockup) {
        // Check if the fault occured on an exception return, or whether it
        // occured during a tail chained exception entry. This is because
        // Lockups on exception return have to be handled differently.
        if (!excInfo.inExcTaken) {
          // If the fault occured during exception return, then the register
          // state is UNKNOWN. This is due to the fact that an unknown amount
          // of the exception stack frame might have been restored.
          for (int n=0; n<13; ++n)
            _s.r[n] = 0; // UNKNOWN
          _s.lr = 0; // UNKNOWN
          _s.xpsr = 0; // UNKNOWN
          if (_HaveFPExt())
            for (int n=0; n<32; ++n)
              _SetS(n, 0); // UNKNOWN
          _s.fpscr = 0; // UNKNOWN
          // If lockup is entered as a result of an exception return fault the
          // original exception is deactivated. Therefore the stack pointer
          // must be updated to consume the exception stack frame to keep the
          // stack depth consistent with the number of active exceptions. NOTE:
          // The XPSR SP alignment flag is UNKNOWN, assume is was zero.
          _ConsumeExcStackFrame(excReturn, false);
          // IPSR from stack is UNKNOWN, set IPSR bsased on mode specified in
          // EXC_RETURN.
          _s.xpsr = CHGBITSM(_s.xpsr, XPSR__EXCEPTION, (excReturn & EXC_RETURN__MODE) ? NoFault : HardFault);
          if (_HaveFPExt()) {
            uint32_t &control = _IsSecure() ? _s.controlS : _s.controlNS;
            control = CHGBITSM(control,  CONTROL__FPCA, ~GETBITSM(excReturn, EXC_RETURN__FTYPE));
            _s.controlS = CHGBITSM(_s.controlS, CONTROL__SFPA, 0); // UNKNOWN
          }
        }
        _Lockup(false);
      } else {
        // Set syndrome if fault escalated to a hardfault
        if (_HaveMainExt() && excInfo.fault == HardFault && excInfo.origFault != HardFault)
          InternalOr32(REG_HFSR, REG_HFSR__FORCED);
        _SetPending(excInfo.fault, excInfo.isSecure, true);
      }
    }
  }

    TRACE("NIA Y ch=%d thisAddr=0x%x thisLen=%d ovr=0x%x\n", _s.pcChanged, _ThisInstrAddr(), _ThisInstrLength(), _s.nextInstrAddr);
  // If there is a pending exception with sufficient priority take it now. This
  // is done before committing PC and ITSTATE changes caused by the previous
  // instruction so that calls to ThisInstrAddr(), NextInstrAddr(),
  // ThisInstrITState(), NextInstrITState() represent the context the
  // instruction was executed in. IE so the correct context is pushed to the
  // stack.
  auto [takeException, exception, excIsSecure] = _PendingExceptionDetails();
  if (takeException) {
    TRACE("TAKE EXC %d\n", exception);
    // If a fault occurred during an exception return then the exception
    // stack frame will already be on the stack, as a resut entry to the
    // next exception is treated as if it were a tail chain.
    int pePriority = _ExecutionPriority();
    uint32_t peException = GETBITSM(_s.xpsr, XPSR__EXCEPTION);
    bool peIsSecure = _IsSecure();
    ExcInfo excInfo;
    if (excRetFault) {
      // If the fault occurred during ExceptionTaken() then LR will have been
      // updated with the new exception return value. To excReturn consistent
      // with the state of the exception stack frame we need to use the updated
      // version in this case. If no updates have occurred then the excReturn
      // value from the previous exception return is used.
      uint32_t nextExcReturn = excInfo.inExcTaken ? _s.lr : excReturn;
      excInfo = _TailChain(exception, excIsSecure, nextExcReturn);
    } else
      excInfo = _ExceptionEntry(exception, excIsSecure, instExecOk);
    // Handle any derived faults that have occurred
    if (excInfo.fault != NoFault)
      _DerivedLateArrival(pePriority, peException, peIsSecure, excInfo, exception, excIsSecure);
  }

  // If the PC has moved away from the lockup address (eg because an NMI has
  // been taken) leave the lockup state.
  if (InternalLoad32(REG_DHCSR) & REG_DHCSR__S_LOCKUP && _NextInstrAddr() != 0xEFFF'FFFE)
    InternalMask32(REG_DHCSR, REG_DHCSR__S_LOCKUP);

  // Only advance the PC and ISTATE if not locked up.
  if (!(InternalLoad32(REG_DHCSR) & REG_DHCSR__S_LOCKUP)) {
    // Commit PC and ITSTATE changes ready for the next instruction.
    TRACE("NIA Z ch=%d thisAddr=0x%x thisLen=%d ovr=0x%x\n", _s.pcChanged, _ThisInstrAddr(), _ThisInstrLength(), _s.nextInstrAddr);
    _s.pc = _NextInstrAddr();
    _s.pcChanged = false;
    if (_HaveMainExt()) {
      uint32_t next = _NextInstrITState();
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__IT_ICI_LO, next>>2);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__IT_ICI_HI, next);
      _s.itStateChanged = false;
    }
  }
}

bool Emulator::_ConditionHolds(uint32_t cond) {
  bool result;
  switch ((cond>>1) & 0b111) {
    case 0b000: result = GETBITSM(_s.xpsr, XPSR__Z); break;
    case 0b001: result = GETBITSM(_s.xpsr, XPSR__C); break;
    case 0b010: result = GETBITSM(_s.xpsr, XPSR__N); break;
    case 0b011: result = GETBITSM(_s.xpsr, XPSR__V); break;
    case 0b100: result = GETBITSM(_s.xpsr, XPSR__C) && !GETBITSM(_s.xpsr, XPSR__Z); break;
    case 0b101: result = GETBITSM(_s.xpsr, XPSR__Z) == GETBITSM(_s.xpsr, XPSR__V); break;
    case 0b110: result = GETBITSM(_s.xpsr, XPSR__Z) == GETBITSM(_s.xpsr, XPSR__V) && !GETBITSM(_s.xpsr, XPSR__Z); break;
    case 0b111: result = true; break;
  }

  if ((cond & 1) && cond != 0b111)
    result = !result;

  return result;
}

void Emulator::_SetMonStep(bool monStepActive) {
  if (!monStepActive)
    return;

  if (!(InternalLoad32(REG_DEMCR) & REG_DEMCR__MON_STEP))
    throw Exception(ExceptionType::UNPREDICTABLE);

  if (_ExceptionPriority(DebugMonitor, _IsSecure(), true) < _ExecutionPriority()) {
    InternalOr32(REG_DEMCR, REG_DEMCR__MON_PEND);
    InternalOr32(REG_DFSR, REG_DFSR__HALTED);
  }
}

bool Emulator::_ExceptionTargetsSecure(int excNo, bool isSecure) {
  if (!_HaveSecurityExt())
    return false;

  bool targetSecure = false;
  switch (excNo) {
    case NMI:
      targetSecure = !(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS);
      break;
    case HardFault:
      targetSecure = !(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS) || isSecure;
      break;
    case MemManage:
      targetSecure = isSecure;
      break;
    case BusFault:
      targetSecure = !(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS);
      break;
    case UsageFault:
      targetSecure = isSecure;
      break;
    case SecureFault:
      targetSecure = true;
      break;
    case SVCall:
      targetSecure = isSecure;
      break;
    case DebugMonitor:
      targetSecure = !!(InternalLoad32(REG_DEMCR) & REG_DEMCR__SDME);
      break;
    case PendSV:
      targetSecure = isSecure;
      break;
    case SysTick:
      if (_HaveSysTick() == 2) {
      } else if (_HaveSysTick() == 1)
        // SysTick target state is configurable
        targetSecure = !(InternalLoad32(REG_ICSR_S) & REG_ICSR__STTNS);
      break;

    default:
      if (excNo >= 16)
        targetSecure = !(InternalLoad32(REG_NVIC_ITNSn((excNo-16)/32)) & BIT((excNo-16)%32));
      break;
  }

  return targetSecure;
}

std::tuple<bool, int> Emulator::_IsCPInstruction(uint32_t instr) {
  bool isCp = false;
  if ((instr & 0b11101111000000000000000000000000) == 0b11101110000000000000000000000000)
    isCp = true;
  if ((instr & 0b11101110000000000000000000000000) == 0b11101100000000000000000000000000)
    isCp = true;
  int cpNum = isCp ? GETBITS(instr, 8,11) : 0 /* UNKNOWN */;
  if (cpNum == 11)
    cpNum = 10;
  return {isCp, cpNum};
}

void Emulator::_DWT_InstructionMatch(uint32_t iaddr) {
  bool triggerDebugEvent = false;
  bool debugEvent = false;

  if (!_HaveDWT() || !(InternalLoad32(REG_DWT_CTRL) & REG_DWT_CTRL__NUMCOMP))
    return;

  assert(false); // TODO
}

std::tuple<bool, bool> Emulator::_IsCPEnabled(int cp, bool priv, bool secure) {
  bool enabled, forceToSecure = false;

  uint32_t cpacr = secure ? InternalLoad32(REG_CPACR_S) : InternalLoad32(REG_CPACR_NS);
  switch (GETBITS(cpacr,cp*2,cp*2+1)) {
    case 0b00:
      enabled = false;
      break;
    case 0b01:
      enabled = priv;
      break;
    case 0b10:
      throw Exception(ExceptionType::UNPREDICTABLE);
      break;
    case 0b11:
      enabled = true;
      break;
  }

  if (enabled && _HaveSecurityExt()) {
    if (!secure && !(InternalLoad32(REG_NSACR) & BIT(cp))) {
      enabled = false;
      forceToSecure = true;
    }
  }

  if (enabled && !!(InternalLoad32(REG_CPPWR) & BIT(cp*2))) {
    enabled = false;
    forceToSecure = !!(InternalLoad32(REG_CPPWR) & BIT(cp*2+1));
  }

  return {enabled, secure || forceToSecure};
}

uint16_t Emulator::_GetMemI(uint32_t addr) {
  uint16_t value;
  auto [excInfo, memAddrDesc] = _ValidateAddress(addr, AccType_IFETCH, _FindPriv(), _IsSecure(), false, true);
  if (excInfo.fault == NoFault) {
    bool error;
    std::tie(error, value) = _GetMem(memAddrDesc, 2);
    if (error) {
      value = UINT16_MAX; // UNKNOWN
      InternalOr32(REG_BFSR, REG_BFSR__IBUSERR);
      excInfo = _CreateException(BusFault, false, false/*UNKNOWN*/);
      TRACE("fetch failed\n");
    }
  } else TRACE("fetch addr validate failed\n");

  _HandleException(excInfo);
  if (_IsDWTEnabled())
    _DWT_InstructionMatch(addr);
  return value;
}

int Emulator::_ExecutionPriority() {
  int boostedPri = _HighestPri();

  int priSNsPri = _RestrictedNSPri();
  if (_HaveMainExt()) {
    if (GETBITS(_s.basepriNS, 0, 7)) {
      uint32_t basepri        = GETBITS(_s.basepriNS, 0, 7);
      uint32_t subGroupShift  = GETBITSM(InternalLoad32(REG_AIRCR_NS), REG_AIRCR__PRIGROUP);
      uint32_t groupValue     = 2U<<subGroupShift;
      uint32_t subGroupValue  = basepri % groupValue;
      boostedPri              = basepri - subGroupValue;
      if (InternalLoad32(REG_AIRCR_S) & REG_AIRCR__PRIS)
        boostedPri = (boostedPri>>1) + priSNsPri;
    }

    if (GETBITS(_s.basepriS, 0, 7)) {
      uint32_t basepri        = GETBITS(_s.basepriS, 0, 7);
      uint32_t subGroupShift  = GETBITSM(InternalLoad32(REG_AIRCR_S), REG_AIRCR__PRIGROUP);
      uint32_t groupValue     = 2U<<subGroupShift;
      uint32_t subGroupValue  = basepri % groupValue;
      basepri                 = basepri - subGroupValue;
      if (boostedPri > basepri)
        boostedPri = basepri;
    }
  }

  if (_s.primaskNS & 1) {
    if (!(InternalLoad32(REG_AIRCR_S) & REG_AIRCR__PRIS))
      boostedPri = 0;
    else if (boostedPri > priSNsPri)
      boostedPri = priSNsPri;
  }

  if (_s.primaskS & 1)
    boostedPri = 0;

  if (_HaveMainExt()) {
    if (_s.faultmaskNS & 1) {
      if (!(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS)) {
        if (!(InternalLoad32(REG_AIRCR_S) & REG_AIRCR__PRIS))
          boostedPri = 0;
        else if (boostedPri > priSNsPri)
          boostedPri = priSNsPri;
      } else
        boostedPri = -1;
    }

    if (_s.faultmaskS & 1)
      boostedPri = !(InternalLoad32(REG_AIRCR) & REG_AIRCR__BFHFNMINS) ? -1 : -3;
  }

  int rawExecPri = _RawExecutionPriority();
  return boostedPri < rawExecPri ? boostedPri : rawExecPri;
}

SAttributes Emulator::_SecurityCheck(uint32_t addr, bool isInstrFetch, bool isSecure) {
  SAttributes result = {};

  bool idauExempt = false;
  bool idauNs = true;
  bool idauNsc = true;

  if (IMPL_DEF_IDAU_PRESENT) {
    //TODO
  }

  if (isInstrFetch && GETBITS(addr,28,31) == 0b1111) {
    // Use default attributes defined above.
  } else if (   idauExempt
             || (isInstrFetch && GETBITS(addr,28,31) == 0b1110)
             || (addr >= 0xE000'0000 && addr <= 0xE000'2FFF)
             || (addr >= 0xE000'E000 && addr <= 0xE000'EFFF)
             || (addr >= 0xE002'E000 && addr <= 0xE002'EFFF)
             || (addr >= 0xE004'0000 && addr <= 0xE004'1FFF)
             || (addr >= 0xE00F'F000 && addr <= 0xE00F'FFFF)) {
    result.ns       = !isSecure;
    result.irvalid  = false;
  } else {
    if (InternalLoad32(REG_SAU_CTRL) & REG_SAU_CTRL__ENABLE) {
      bool multiRegionHit = false;
      uint32_t numRegion = GETBITSM(InternalLoad32(REG_SAU_TYPE), REG_SAU_TYPE__SREGION);
      for (int r=0; r<numRegion; ++r) {
        auto [rbar,rlar] = _dev.InternalLoadSauRegion(r);
        if (rlar & REG_SAU_RLAR__ENABLE) {
          uint32_t baseAddr   = (GETBITSM(rbar, REG_SAU_RBAR__BADDR) << 5) | 0b00000;
          uint32_t limitAddr  = (GETBITSM(rlar, REG_SAU_RLAR__LADDR) << 5) | 0b11111;
          if (baseAddr <= addr && limitAddr >= addr) {
            if (result.srvalid)
              multiRegionHit = true;
            else {
              result.ns       =  !(rlar & REG_SAU_RLAR__NSC);
              result.nsc      = !!(rlar & REG_SAU_RLAR__NSC);
              result.srvalid  = true;
              result.sregion  = r & 0xFF;
            }
          }
        }
      }
      if (multiRegionHit) {
        result.ns       = false;
        result.nsc      = false;
        result.sregion  = 0;
        result.srvalid  = false;
      }
    } else if (InternalLoad32(REG_SAU_CTRL) & REG_SAU_CTRL__ALLNS)
      result.ns = true;

    if (!idauNs) {
      if (result.ns || (!idauNsc && result.nsc)) {
        result.ns   = false;
        result.nsc  = idauNsc;
      }
    }
  }

  return result;
}

#if 1
struct CorePeripheralState {
  uint32_t fpdscrS{}, fpdscrNS{}, fpccrS{BIT(2)|BIT(30)|BIT(31)}, fpccrNS{BIT(2)|BIT(30)|BIT(31)}, fpcarS{}, fpcarNS{}, vtorS{0x2000'4000}, vtorNS{0x2000'4000}, sauCtrl{}, mpuTypeS{}, mpuTypeNS{}, mpuCtrlS{}, mpuCtrlNS{}, mpuMair0S{}, mpuMair0NS{}, mpuMair1S{}, mpuMair1NS{}, aircrS{}, aircrNS{}, demcrS{}, demcrNS{}, dhcsrS{}, dhcsrNS{}, dauthCtrl{}, mmfsrS{}, mmfsrNS{}, shcsrS{}, shcsrNS{}, shpr1S{}, shpr1NS{}, hfsrS{}, hfsrNS{}, ufsrS{}, ufsrNS{}, fpCtrl{}, nvicPendingS[16]{}, nvicPendingNS[16]{}, nvicNonSecure[16]{}, nvicIntrPrio[124]{};
};

struct CorePeripheral final :IDevice {
  CorePeripheral() {}

  void InternalReset() override {
    printf("Bus internal reset\n");
  }

  uint32_t InternalLoad32(phys_t addr) override {
    printf("Bus internal load 32 0x%x\n", addr);
    switch (addr) {
      case REG_FPDSCR_S:  return _s.fpdscrS;
      case REG_FPDSCR_NS: return _s.fpdscrNS;
      case REG_FPCCR_S:   return _s.fpccrS;
      case REG_FPCCR_NS:  return _s.fpccrNS;
      case REG_FPCAR_S:   return _s.fpcarS;
      case REG_FPCAR_NS:  return _s.fpcarNS;
      case REG_VTOR_S:    return _s.vtorS;
      case REG_VTOR_NS:   return _s.vtorNS;
      case REG_SAU_CTRL:  return _s.sauCtrl;
      case REG_MPU_TYPE_S:    return _s.mpuTypeS;
      case REG_MPU_TYPE_NS:   return _s.mpuTypeNS;
      case REG_MPU_CTRL_S:    return _s.mpuCtrlS;
      case REG_MPU_CTRL_NS:   return _s.mpuCtrlNS;
      case REG_MPU_MAIR0_S:   return _s.mpuMair0S;
      case REG_MPU_MAIR0_NS:  return _s.mpuMair0NS;
      case REG_MPU_MAIR1_S:   return _s.mpuMair1S;
      case REG_MPU_MAIR1_NS:  return _s.mpuMair1NS;
      case REG_AIRCR_S:       return _s.aircrS;
      case REG_AIRCR_NS:      return _s.aircrNS;
      case REG_DEMCR:         return _s.demcrS;
      case REG_DEMCR_NS:      return _s.demcrNS;
      case REG_DHCSR:         return _s.dhcsrS;
      case REG_DHCSR_NS:      return _s.dhcsrNS;
      case REG_DAUTHCTRL:     return _s.dauthCtrl;
      case REG_MMFSR_S:       return _s.mmfsrS;
      case REG_MMFSR_NS:      return _s.mmfsrNS;
      case REG_SHCSR_S:       return _s.shcsrS;
      case REG_SHCSR_NS:      return _s.shcsrNS;
      case REG_SHPR1_S:       return _s.shpr1S;
      case REG_SHPR1_NS:      return _s.shpr1NS;
      case REG_HFSR_S:        return _s.hfsrS;
      case REG_HFSR_NS:       return _s.hfsrNS;
      case REG_UFSR_S:        return _s.ufsrS;
      case REG_UFSR_NS:       return _s.ufsrNS;
      case REG_FP_CTRL:       return _s.fpCtrl;
      default:
        if (addr >= 0xE000E200 && addr < 0xE000E240)
          return _s.nvicPendingS[(addr/4)&0xF];
        if (addr >= 0xE002E200 && addr < 0xE002E240)
          return _s.nvicPendingNS[(addr/4)&0xF];
        if (addr >= 0xE000E380 && addr < 0xE000E3C0)
          return _s.nvicNonSecure[(addr/4)&0xF];
        if (addr >= 0xE000E400 && addr < 0xE000E5F0)
          return _s.nvicIntrPrio[(addr - 0xE000E400)/4];
        abort();
    }
  }

  void InternalStore32(phys_t addr, uint32_t v) override {
    printf("Bus internal store 32 0x%x = 0x%x\n", addr, v);
    switch (addr) {
      case REG_FPDSCR_S:  _s.fpdscrS  = v; break;
      case REG_FPDSCR_NS: _s.fpdscrNS = v; break;
      case REG_FPCCR_S:   _s.fpccrS   = v; break;
      case REG_FPCCR_NS:  _s.fpccrNS  = v; break;
      case REG_FPCAR_S:   _s.fpcarS   = v; break;
      case REG_FPCAR_NS:  _s.fpcarNS  = v; break;
      case REG_VTOR_S:    _s.vtorS    = v; break;
      case REG_VTOR_NS:   _s.vtorNS   = v; break;
      case REG_DEMCR:     _s.demcrS   = v; break;
      case REG_DEMCR_NS:  _s.demcrNS  = v; break;
      case REG_DHCSR:     _s.dhcsrS   = v; break;
      case REG_DHCSR_NS:  _s.dhcsrNS  = v; break;
      case REG_MMFSR_S:   _s.mmfsrS   = v; break;
      case REG_MMFSR_NS:  _s.mmfsrNS  = v; break;
      case REG_HFSR_S:    _s.hfsrS    = v; break;
      case REG_HFSR_NS:   _s.hfsrNS   = v; break;
      case REG_UFSR_S:    _s.ufsrS    = v; break;
      case REG_UFSR_NS:   _s.ufsrNS   = v; break;
      default:
        abort();
    }
  }

  std::tuple<uint32_t,uint32_t> InternalLoadMpuSecureRegion(size_t idx) override {
    printf("Bus internal load MPU secure region %zu\n", idx);
    return {0,0}; // {RBAR,RLAR}
  }

  std::tuple<uint32_t,uint32_t> InternalLoadMpuNonSecureRegion(size_t idx) override {
    printf("Bus internal load MPU non-secure region %zu\n", idx);
    return {0,0}; // {RBAR,RLAR}
  }

  std::tuple<uint32_t,uint32_t> InternalLoadSauRegion(size_t idx) override {
    printf("Bus internal load SAU region %zu\n", idx);
    return {0,0}; // {RBAR,RLAR}
  }

  int Load32(phys_t addr, uint32_t &v) override {
    return 0;
  }

  int Load16(phys_t addr, uint16_t &v) override {
    return 0;
  }

  int Load8(phys_t addr, uint8_t &v) override {
    return 0;
  }

  int Store32(phys_t addr, uint32_t v) override {
    return 0;
  }

  int Store16(phys_t addr, uint16_t v) override {
    return 0;
  }

  int Store8(phys_t addr, uint8_t v) override {
    return 0;
  }

private:
  CorePeripheralState _s{};
};

struct RamDevice final :IDevice {
  RamDevice(phys_t base, size_t len) {
    _base = base;
    _len  = len;
    _buf  = (uint8_t*)new uint32_t[(len+3)/4];
    memset(_buf, 0, _len);
  }

  ~RamDevice() {
    delete[] _buf;
    _buf = nullptr;
  }

  void *GetBuffer() const { return _buf; }
  size_t GetSize() const { return _len; }

  void InternalReset() override {}
  uint32_t InternalLoad32(phys_t addr) override { return 0xFFFF'FFFF; }
  void InternalStore32(phys_t addr, uint32_t v) override {}
  std::tuple<uint32_t,uint32_t> InternalLoadMpuSecureRegion(size_t idx) override { return {0,0}; }
  std::tuple<uint32_t,uint32_t> InternalLoadMpuNonSecureRegion(size_t idx) override { return {0,0}; }
  std::tuple<uint32_t,uint32_t> InternalLoadSauRegion(size_t idx) override { return {0,0}; }

  bool Contains(phys_t addr) const {
    return addr >= _base && addr < _base + _len;
  }

  int Load32(phys_t addr, uint32_t &v) override {
    if (addr < _base || addr+3 >= _base + _len)
      return -1;

    addr -= _base;
    v = *(uint32_t*)(_buf + addr);
    return 0;
  }

  int Load16(phys_t addr, uint16_t &v) override {
    if (addr < _base || addr+1 >= _base + _len)
      return -1;

    addr -= _base;
    v = *(uint16_t*)(_buf + addr);
    return 0;
  }

  int Load8(phys_t addr, uint8_t &v) override {
    if (addr < _base || addr >= _base + _len)
      return -1;

    addr -= _base;
    v = *(uint8_t*)(_buf + addr);
    return 0;
  }

  int Store32(phys_t addr, uint32_t v) override {
    if (addr < _base || addr+3 >= _base + _len)
      return -1;

    addr -= _base;
    *(uint32_t*)(_buf + addr) = v;
    return 0;
  }

  int Store16(phys_t addr, uint16_t v) override {
    if (addr < _base || addr+1 >= _base + _len)
      return -1;

    addr -= _base;
    *(uint16_t*)(_buf + addr) = v;
    return 0;
  }

  int Store8(phys_t addr, uint8_t v) override {
    if (addr < _base || addr >= _base + _len)
      return -1;

    addr -= _base;
    *(uint8_t*)(_buf + addr) = v;
    return 0;
  }

private:
  phys_t   _base;
  size_t   _len;
  uint8_t *_buf;
};

struct RemapDevice final :IDevice {
  RemapDevice(phys_t addr, size_t len, phys_t dstAddr, IDevice &dev) :_addr(addr), _dstAddr(dstAddr), _len(len), _dev(dev) {}

  void InternalReset() override {}
  uint32_t InternalLoad32(phys_t addr) override { return 0xFFFF'FFFF; }
  void InternalStore32(phys_t addr, uint32_t v) override {}
  std::tuple<uint32_t,uint32_t> InternalLoadMpuSecureRegion(size_t idx) override { return {0,0}; }
  std::tuple<uint32_t,uint32_t> InternalLoadMpuNonSecureRegion(size_t idx) override { return {0,0}; }
  std::tuple<uint32_t,uint32_t> InternalLoadSauRegion(size_t idx) override { return {0,0}; }

  bool Contains(phys_t addr, size_t n=1) const {
    return addr >= _addr && addr + (n-1) < _addr + _len;
  }

  int Load32(phys_t addr, uint32_t &v) override {
    if (!Contains(addr, 4))
      return -1;

    return _dev.Load32(addr - _addr + _dstAddr, v);
  }

  int Load16(phys_t addr, uint16_t &v) override {
    if (!Contains(addr, 2))
      return -1;

    return _dev.Load16(addr - _addr + _dstAddr, v);
  }

  int Load8(phys_t addr, uint8_t &v) override {
    if (!Contains(addr, 1))
      return -1;

    return _dev.Load8(addr - _addr + _dstAddr, v);
  }

  int Store32(phys_t addr, uint32_t v) override {
    if (!Contains(addr, 4))
      return -1;

    return _dev.Store32(addr - _addr + _dstAddr, v);
  }

  int Store16(phys_t addr, uint16_t v) override {
    if (!Contains(addr, 2))
      return -1;

    return _dev.Store16(addr - _addr + _dstAddr, v);
  }

  int Store8(phys_t addr, uint8_t v) override {
    if (!Contains(addr, 1))
      return -1;

    return _dev.Store8(addr - _addr + _dstAddr, v);
  }

private:
  phys_t    _addr, _dstAddr;
  size_t    _len;
  IDevice  &_dev;
};

struct STM32Device final :IDevice {
  void InternalReset() override {
    _core.InternalReset();
  }

  uint32_t InternalLoad32(phys_t addr) override {
    return _core.InternalLoad32(addr);
  }

  void InternalStore32(phys_t addr, uint32_t v) override {
    _core.InternalStore32(addr, v);
  }

  std::tuple<uint32_t,uint32_t> InternalLoadMpuSecureRegion(size_t idx) override {
    return _core.InternalLoadMpuSecureRegion(0);
  }
  std::tuple<uint32_t,uint32_t> InternalLoadMpuNonSecureRegion(size_t idx) override {
    return _core.InternalLoadMpuNonSecureRegion(0);
  }
  std::tuple<uint32_t,uint32_t> InternalLoadSauRegion(size_t idx) override {
    return _core.InternalLoadSauRegion(0);
  }

  int Load32(phys_t addr, uint32_t &v) override {
    auto dev = (IDevice*)_Resolve(addr);
    if (!dev) {
      printf("Bus load 32 0x%x: nothing\n", addr);
      v = 0xFFFF'FFFF;
      return 0;
    }

    int rc = dev->Load32(addr, v);
    printf("Bus load 32 0x%x: 0x%x\n", addr, v);
    return rc;
  }

  int Load16(phys_t addr, uint16_t &v) override {
    auto dev = (IDevice*)_Resolve(addr);
    if (!dev) {
      printf("Bus load 16 0x%x: nothing\n", addr);
      v = 0xFFFF;
      return 0;
    }

    int rc = dev->Load16(addr, v);
    printf("Bus load 16 0x%x = 0x%x\n", addr, v);
    return rc;
  }

  int Load8(phys_t addr, uint8_t &v) override {
    printf("Bus load 8 0x%x\n", addr);
    auto dev = (IDevice*)_Resolve(addr);
    if (!dev) {
      v = 0xFF;
      return 0;
    }

    return dev->Load8(addr, v);
  }

  int Store32(phys_t addr, uint32_t v) override {
    printf("Bus store 32 0x%x = 0x%x\n", addr, v);
    auto dev = (IDevice*)_Resolve(addr);
    if (!dev)
      return 0;

    return dev->Store32(addr, v);
  }

  int Store16(phys_t addr, uint16_t v) override {
    printf("Bus store 16 0x%x = 0x%x\n", addr, v);
    auto dev = (IDevice*)_Resolve(addr);
    if (!dev)
      return 0;

    return dev->Store16(addr, v);
  }

  int Store8(phys_t addr, uint8_t v) override {
    printf("Bus store 8 0x%x = 0x%x\n", addr, v);
    auto dev = (IDevice*)_Resolve(addr);
    if (!dev)
      return 0;

    return dev->Store8(addr, v);
  }

  RamDevice &GetSRAM1() { return _sram1; }

private:
  IDevice *_Resolve(phys_t addr) {
    const phys_t sram1End   = 0x2000'0000 + _sram1.GetSize();
    const phys_t sram2AEnd  = 0x1000'0000 + _sram2.GetSize();
    const phys_t sram2BEnd  = 0x2000'C000 + _sram2.GetSize();

    if (addr >= 0 && addr < 0x4'0000) {
      // "Flash, system memory or SRAM depending on boot configuration"
      return &_bootr;
    } else if (addr >= 0x0800'0000 && addr < 0x0804'0000) {
      // Flash memory
      return nullptr;
    } else if (addr >= 0x1000'0000 && addr < sram2AEnd) {
      // SRAM2
      return &_sram2r;
    } else if (addr >= 0x1FFF'0000 && addr < 0x1FFF'7000) {
      // System memory
      return nullptr;
    } else if (addr >= 0x1FFF'7000 && addr < 0x1FFF'7400) {
      // OTP area
      return nullptr;
    } else if (addr >= 0x1FFF'7800 && addr < 0x1FFF'7810) {
      // Option bytes
      return nullptr;
    } else if (addr >= 0x2000'0000 && addr < sram1End) {
      // SRAM1
      return &_sram1;
    } else if (addr >= 0x2000'C000 && addr < sram2BEnd) {
      // SRAM2
      return &_sram2;
    } else if (addr >= 0x4000'0000 && addr < 0) {
      // Peripherals
      return nullptr;
    } else if (addr >= 0x9000'0000 && addr < 0xA000'0000) {
      // QUADSPI Flash bank
      return nullptr;
    } else if (addr >= 0xA000'1000 && addr < 0xC000'0000) {
      // QUADSPI Registers
      return nullptr;
    } else if (addr >= 0xE000'0000 && addr <= 0xFFFF'FFFF) {
      return nullptr; //&_core;
    } else
      return nullptr;
  }

  CorePeripheral      _core;
  RamDevice           _sram1{0x2000'0000, 48*1024};
  RamDevice           _sram2{0x2000'C000, 16*1024};
  RemapDevice         _sram2r{0x1000'0000, 16*1024, 0x2000'C000, _sram2};
  RemapDevice         _bootr{0x0000'0000, 256*1024, 0x2000'0000, _sram1};
};

int main(int argc, char **argv) {
  STM32Device dev;
  RamDevice  &sram1     = dev.GetSRAM1();
  void       *sram1Buf  = sram1.GetBuffer();
  size_t      sram1Len  = sram1.GetSize();

  FILE *f = fopen("../debang1.bin0", "rb");
  if (!f)
    return 1;

  fseek(f, 0, SEEK_END);
  size_t flen = ftell(f);
  fseek(f, 0, SEEK_SET);

  if (flen > sram1Len)
    return 2;

  if (fread((uint8_t*)sram1Buf + 0x4000, 1, flen, f) != flen)
    return 3;

  fclose(f);

  Emulator emu(dev);

  for (;;) {
    emu.TopLevel();
  }
  return 0;
}
#endif
