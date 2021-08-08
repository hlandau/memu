#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <exception>
#include <tuple>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <limits>
#include <unordered_map>

/* Preprocessor Utilities                                                  {{{1
 * ============================================================================
 */

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
#define CHGBIT(OldValue, X, NewValue)       CHGBITS((OldValue), (X), (X), (NewValue))
// Change bits in the field defined by the given mask to the new value.
#define CHGBITSM(OldValue, Mask, NewValue)  (((OldValue) & ~(Mask))|PUTBITSM((NewValue), (Mask)))

#define likely(Expr)      (__builtin_expect(!!(Expr), 1))
#define unlikely(Expr)    (__builtin_expect(!!(Expr), 0))
#define UNREACHABLE()     do { __builtin_unreachable(); } while(0)


/* ARMv8-M Simulator                                                       {{{1
 * ============================================================================
 * TODO LIST:
 *  _SendEvent
 *  _SCS_UpdateStatusRegs
 *
 *  Add missing TRACEIs
 *
 *  Test monitor support
 *  Proper register implementation -- ~done for essential registers
 *  Debug stuff
 *  S_HALT support
 *  Full manual review
 *  Coprocessor handling
 *  Floating point support
 *
 *  SCR.SEVONPEND: Exceptions entering the pending state should cause SetEventRegister
 *  Check NVIC usage of _excEnable/_excPending/_excActive; should we use _SetPending etc.?
 *  DHCSR.C_MASKINTS
 */

/* Simulator Compile-Time Configuration {{{2
 * ====================================
 */
#define NUM_MPU_REGION_S    16
#define NUM_MPU_REGION_NS   16
#define NUM_SAU_REGION      8
#define NUM_DWT_COMP        4
#define NUM_FPB_COMP        4
#define CP_IMPL_MASK        0b11111111
#define ENFORCE_SOFT_BITS   1

// Implementation Defined Flags
#define IMPL_DEF_DECODE_CP_SPACE                      1
#define IMPL_DEF_EARLY_SG_CHECK                       1
#define IMPL_DEF_SPLIM_CHECK_UNPRED_INSTR             1
#define IMPL_DEF_SPLIM_EXCEPTION_ON_INVAL_MEM_ACCESS  1
#define IMPL_DEF_IDAU_PRESENT                         0
#define IMPL_DEF_PUSH_NON_VIOL_LOCATIONS              0
#define IMPL_DEF_OVERRIDDEN_EXCEPTIONS_PENDED         1
#define IMPL_DEF_TAIL_CHAINING_SUPPORTED              1
#define IMPL_DEF_DROP_PREV_GEN_EXC                    1
#define IMPL_DEF_BASELINE_NO_SW_ACCESS_DWT            0
#define IMPL_DEF_BASELINE_NO_SW_ACCESS_FPB            0
#define IMPL_DEF_LOCAL_MON_CHECK_ADDR                 1

// Verify Configuration
#if NUM_FPB_COMP > 126
#  error Cannot have more than 126 FPB comparator registers
#endif

#ifndef EMU_TRACE
#  define EMU_TRACE 0
#endif


/* Simulator Debugging and Tracing Utilities {{{2
 * =========================================
 */
#if EMU_TRACE
int g_emuTraceIndent;
struct EmuTraceBlock {
  EmuTraceBlock() { ++g_emuTraceIndent; }
  ~EmuTraceBlock() { --g_emuTraceIndent; }
};

#  define TRACE_BLOCK() EmuTraceBlock PP_CAT(_traceblk_,__COUNTER__)
#  define TRACE(...) do { printf("T: "); for (size_t _traceI=0; _traceI<g_emuTraceIndent; ++_traceI) printf("  "); printf(__VA_ARGS__); } while(0)
#  define TRACEI(Name, Encoding, ...) do { printf("%s: %08x " #Name " " #Encoding " ", _ConditionPassed() ? "I" : "i", pc); printf(__VA_ARGS__); printf("\n"); } while (0)
#  define TRACEIU(Name, Encoding, ...) do { printf("I: %08x " #Name " " #Encoding " ", pc); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#  define TRACE_BLOCK()
#  define TRACE(...) do {} while (0)
#  define TRACEI(...) do {} while (0)
#endif

#define ASSERT(Cond) do { if unlikely (!(Cond)) { printf("assertion fail on line %u: %s\n", __LINE__, #Cond); abort(); } } while(0)

#if ENFORCE_SOFT_BITS
#  define CHECK01(BitsOff, BitsOn)                                          \
  do {                                                                      \
    if unlikely ((instr & (BitsOff)) || (~instr & (BitsOn))) {              \
      TRACE("unpredictable encoding 0x%x, line %u\n", instr, __LINE__);     \
      THROW_UNDEFINED();                            \
    }                                                                       \
  } while (0)                                                            /**/
#else
#  define CHECK01(BitsOff, BitsOn)  do {} while (0)
#endif

#define CHECKV(MinVer)                               \
  do {                                               \
    if unlikely (_cfg.IsaVersion() < (MinVer)) {     \
      TRACE("instruction not supported for this ISA version (encoding 0x%x, line %u)\n", instr, __LINE__); \
      THROW_UNDEFINED();     \
    }                                                \
  } while (0)                                     /**/


/* Simulator Definitions {{{2
 * =====================
 */
// Maximum number of exceptions an ARMv8-M implementation may support.
#define NUM_EXC 512

// Where a value is defined as UNDEFINED or UNKNOWN in the ISA psuedocode, we
// choose a specific value X and wrap it in one of these macros here as
// exposition.
#define UNDEFINED_VAL(X)  (X)
#define UNKNOWN_VAL(X)    (X)

// The standard exceptions. Exceptions 16 and higher are external interrupts.
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

using phys_t = uint32_t;

#define THROW_UNPREDICTABLE() do { TRACE("W: unpredictable on %u\n", __LINE__); throw Exception(ExceptionType::UNPREDICTABLE); } while (0)
#define THROW_UNDEFINED()     do { TRACE("W: undefined on %u\n", __LINE__); throw Exception(ExceptionType::UNDEFINED); } while (0)
#define UNDEFINED_DEC() do { printf("W:         UNDEFINED %u\n", __LINE__); THROW_UNDEFINED(); } while (0)
#define TODO_DEC()      do { printf("W: %08x  TODO insn on line %u\n", pc, __LINE__); UNDEFINED_DEC(); } while (0)
  // For CONSTRAINED UNPREDICTABLE which we choose to implement as UNDEFINED
#define CUNPREDICTABLE_UNDEFINED() UNDEFINED_DEC()
#define CUNPREDICTABLE_UNALIGNED() do { _ThrowUnaligned(); } while (0)

/* Exception {{{3
 * ---------
 */
// These constitute the various reasons for which normal control flow can be
// interrupted inside our simulator. 'Exception' here means C++ exception, not
// ARM exception; C++ exceptions may or may not be related to the handling of
// an ARM exception.
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

  const char *what() const noexcept override final {
    switch (GetType()) {
      case ExceptionType::SEE:              return "SEE";
      case ExceptionType::UNDEFINED:        return "UNDEFINED";
      case ExceptionType::UNPREDICTABLE:    return "UNPREDICTABLE";
      case ExceptionType::EndOfInstruction: return "EndOfInstruction";
      case ExceptionType::Internal:         return "Internal";
      default: return "?";
    }
  }

private:
  ExceptionType _type;
};

/* Register Definitions {{{3
 * ====================
 */
#define SECREG(Name) (_IsSecure() ? Name##_S : Name##_NS)

/* ITM: Instrumentation Macrocell {{{4
 * ------------------------------
 */

/* DWT: Data Watchpoint and Trace {{{4
 * ------------------------------
 */
#define REG_DWT_CTRL      0xE000'1000
#define   REG_DWT_CTRL__NUMCOMP   BITS(28,31)
#define   REG_DWT_CTRL__NOTRCPKT  BIT (27)
#define   REG_DWT_CTRL__NOCYCCNT  BIT (25)
#define   REG_DWT_CTRL__NOPRFCNT  BIT (24)
#define REG_DWT_COMP(N)     (0xE000'1020 + 16*(N))
#define REG_DWT_FUNCTION(N) (0xE000'1028 + 16*(N))
#define   REG_DWT_FUNCTION__MATCH     BITS( 0, 3)
#define   REG_DWT_FUNCTION__ACTION    BITS( 4, 5)
#define   REG_DWT_FUNCTION__DATAVSIZE BITS(10,11)
#define   REG_DWT_FUNCTION__MATCHED   BIT (24)
#define   REG_DWT_FUNCTION__ID        BITS(27,31)

/* FPB: Flash Patch and Breakpoint {{{4
 * -------------------------------
 */
#define REG_FP_CTRL       0xE000'2000
#define   REG_FP_CTRL__ENABLE       BIT ( 0)
#define   REG_FP_CTRL__KEY          BIT ( 1)
#define   REG_FP_CTRL__NUM_CODE_LO  BITS( 4, 7)
#define   REG_FP_CTRL__NUM_CODE_HI  BITS(12,14)
#define   REG_FP_CTRL__NUM_LIT      BITS( 8,11)
#define   REG_FP_CTRL__REV          BITS(28,31)
#define REG_FP_COMP(N)     (0xE000'2008+4*(N))
#define   REG_FP_COMPn__BE             BIT ( 0)
#define   REG_FP_COMPn__BPADDR         BITS( 1,31)

/* ICB: Implementation Control Block {{{4
 * ---------------------------------
 */
#define REG_CPPWR         SECREG(REG_CPPWR)
#define REG_CPPWR_S       0xE000'E00C
#define REG_CPPWR_NS      0xE002'E00C
#define   REG_CPPWR__SUn(N)   BIT (N*2)
#define   REG_CPPWR__SUSn(N)  BIT (N*2+1)

/* SysTick: SysTick Timer {{{4
 * ----------------------
 */
#define REG_SYST_CSR_S    0xE000'E010
#define REG_SYST_CSR_NS   0xE002'E010
#define   REG_SYST_CSR__ENABLE    BIT ( 0)
#define   REG_SYST_CSR__TICKINT   BIT ( 1)
#define   REG_SYST_CSR__CLKSOURCE BIT ( 2)
#define   REG_SYST_CSR__COUNTFLAG BIT (16)
#define REG_SYST_RVR_S    0xE000'E014
#define REG_SYST_RVR_NS   0xE002'E014
#define REG_SYST_CVR_S    0xE000'E018
#define REG_SYST_CVR_NS   0xE002'E018
#define REG_SYST_CALIB_S  0xE000'E01C
#define REG_SYST_CALIB_NS 0xE002'E01C
#define   REG_SYST_CALIB__TENMS   BITS( 0,23)
#define   REG_SYST_CALIB__SKEW    BIT (30)
#define   REG_SYST_CALIB__NOREF   BIT (31)

/* NVIC: Nested Vectored Interrupt Controller {{{4
 * ------------------------------------------
 */
#define REG_NVIC_IPRn_S(N)  (0xE000'E400 + 4*(N))
#define REG_NVIC_IPRn_NS(N) (0xE002'E400 + 4*(N))

#define REG_NVIC_ISPRn_S(N)  (0xE000'E200 + 4*(N))
#define REG_NVIC_ISPRn_NS(N) (0xE002'E200 + 4*(N))

#define REG_NVIC_ITNSn(N) (0xE000'E380+4*(N))

/* SCB: System Control Block {{{4
 * -------------------------
 */
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
#define REG_VTOR_S    0xE000'ED08
#define REG_VTOR_NS   0xE002'ED08
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
#define REG_SCR           SECREG(REG_SCR)
#define REG_SCR_S         0xE000'ED10
#define REG_SCR_NS        0xE002'ED10
#define   REG_SCR__SLEEPONEXIT    BIT ( 1)
#define REG_CCR       SECREG(REG_CCR)
#define REG_CCR_S     0xE000'ED14
#define REG_CCR_NS    0xE002'ED14
#define   REG_CCR__UNALIGN_TRP        BIT ( 3)
#define   REG_CCR__DIV_0_TRP          BIT ( 4)
#define   REG_CCR__BFHFNMIGN          BIT ( 8)
#define   REG_CCR__STKOFHFNMIGN       BIT (10)
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
#define REG_CFSR      SECREG(REG_CFSR)
#define REG_CFSR_S    0xE000'ED28
#define REG_CFSR_NS   0xE002'ED28
#define   REG_CFSR__MMFSR          BITS( 0, 7)
#define     REG_CFSR__MMFSR__IACCVIOL   BIT ( 0)
#define     REG_CFSR__MMFSR__DACCVIOL   BIT ( 1)
#define     REG_CFSR__MMFSR__MUNSTKERR  BIT ( 3)
#define     REG_CFSR__MMFSR__MSTKERR    BIT ( 4)
#define     REG_CFSR__MMFSR__MLSPERR    BIT ( 5)
#define     REG_CFSR__MMFSR__MMARVALID  BIT ( 7)
#define   REG_CFSR__BFSR           BITS( 8,15)
#define     REG_CFSR__BFSR__IBUSERR     BIT ( 8+ 0)
#define     REG_CFSR__BFSR__PRECISERR   BIT ( 8+ 1)
#define     REG_CFSR__BFSR__IMPRECISERR BIT ( 8+ 2)
#define     REG_CFSR__BFSR__UNSTKERR    BIT ( 8+ 3)
#define     REG_CFSR__BFSR__STKERR      BIT ( 8+ 4)
#define     REG_CFSR__BFSR__LSPERR      BIT ( 8+ 5)
#define     REG_CFSR__BFSR__BFARVALID   BIT ( 8+ 7)
#define   REG_CFSR__UFSR           BITS(16,31)
#define     REG_CFSR__UFSR__UNDEFINSTR  BIT (16+ 0)
#define     REG_CFSR__UFSR__INVSTATE    BIT (16+ 1)
#define     REG_CFSR__UFSR__INVPC       BIT (16+ 2)
#define     REG_CFSR__UFSR__NOCP        BIT (16+ 3)
#define     REG_CFSR__UFSR__STKOF       BIT (16+ 4)
#define     REG_CFSR__UFSR__UNALIGNED   BIT (16+ 8)
#define     REG_CFSR__UFSR__DIVBYZERO   BIT (16+ 9)
#define REG_HFSR      SECREG(REG_HFSR)
#define REG_HFSR_S    0xE000'ED2C
#define REG_HFSR_NS   0xE002'ED2C
#define   REG_HFSR__VECTTBL   BIT ( 1)
#define   REG_HFSR__FORCED    BIT (30)
#define REG_DFSR      SECREG(REG_DFSR)
#define REG_DFSR_S    0xE000'ED30
#define REG_DFSR_NS   0xE002'ED30
#define   REG_DFSR__HALTED          BIT ( 0)
#define   REG_DFSR__BKPT            BIT ( 1)
#define   REG_DFSR__DWTTRAP         BIT ( 2)
#define   REG_DFSR__VCATCH          BIT ( 3)
#define   REG_DFSR__EXTERNAL        BIT ( 4)
#define REG_MMFAR     SECREG(REG_MMFAR)
#define REG_MMFAR_S   0xE000'ED34
#define REG_MMFAR_NS  0xE002'ED34
#define   REG_MMFAR__ADDRESS    BITS ( 0,31)
#define REG_BFAR      SECREG(REG_BFAR)
#define REG_BFAR_S    0xE000ED38
#define REG_BFAR_NS   0xE002ED38
#define   REG_BFAR__ADDRESS BITS( 0,31)

#define REG_CPACR         SECREG(REG_CPACR)
#define REG_CPACR_S       0xE000'ED88
#define REG_CPACR_NS      0xE002'ED88
#define   REG_CPACR__CPn(N)   BITS(2*(N),2*(N)+1)
#define REG_NSACR     0xE000'ED8C
#define   REG_NSACR__CP10   BIT(10)
#define   REG_NSACR__CP(N)  BIT( N)

/* MPU: Memory Protection Unit {{{4
 * ---------------------------
 */
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

#define REG_MPU_RNR_S   0xE000'ED98
#define REG_MPU_RNR_NS  0xE002'ED98

#define REG_MPU_RBAR_S  0xE000'ED9C
#define REG_MPU_RBAR_NS 0xE002'ED9C

#define REG_MPU_RBAR__XN        BIT ( 0)
#define REG_MPU_RBAR__AP        BITS( 1, 2)
#define REG_MPU_RBAR__SH        BITS( 3, 4)
#define REG_MPU_RBAR__BASE      BITS( 5,31)

#define REG_MPU_RLAR_S  0xE000'EDA0
#define REG_MPU_RLAR_NS 0xE002'EDA0

#define REG_MPU_RLAR__EN        BIT ( 0)
#define REG_MPU_RLAR__ATTR_IDX  BITS( 1, 3)
#define REG_MPU_RLAR__LIMIT     BITS(5,31)

#define REG_MPU_RBAR_A1_S 0xE000'EDA4
#define REG_MPU_RBAR_A2_S 0xE000'EDAC
#define REG_MPU_RBAR_A3_S 0xE000'EDB4
#define REG_MPU_RBAR_A1_NS 0xE002'EDA4
#define REG_MPU_RBAR_A2_NS 0xE002'EDAC
#define REG_MPU_RBAR_A3_NS 0xE002'EDB4

#define REG_MPU_RLAR_A1_S 0xE000'EDA8
#define REG_MPU_RLAR_A2_S 0xE000'EDB0
#define REG_MPU_RLAR_A3_S 0xE000'EDB8
#define REG_MPU_RLAR_A1_NS 0xE002'EDA8
#define REG_MPU_RLAR_A2_NS 0xE002'EDB0
#define REG_MPU_RLAR_A3_NS 0xE002'EDB8

#define REG_MPU_MAIR0     SECREG(REG_MPU_MAIR0)
#define REG_MPU_MAIR0_S   0xE000EDC0
#define REG_MPU_MAIR0_NS  0xE002EDC0

#define REG_MPU_MAIR1     SECREG(REG_MPU_MAIR1)
#define REG_MPU_MAIR1_S   0xE000EDC4
#define REG_MPU_MAIR1_NS  0xE002EDC4

/* SAU: Security Attribution Unit {{{4
 * ------------------------------
 */
#define REG_SAU_CTRL      0xE000'EDD0
#define   REG_SAU_CTRL__ENABLE        BIT ( 0)
#define   REG_SAU_CTRL__ALLNS         BIT ( 1)
#define REG_SAU_TYPE      0xE000'EDD4
#define   REG_SAU_TYPE__SREGION       BITS( 0, 7)
#define REG_SAU_RNR       0xE000'EDD8
#define   REG_SAU_RNR__REGION         BITS( 0, 7)
#define REG_SAU_RBAR      0xE000'EDDC
#define   REG_SAU_RBAR__BADDR   BITS( 5,31)
#define REG_SAU_RLAR      0xE000'EDE0
#define   REG_SAU_RLAR__ENABLE  BIT ( 0)
#define   REG_SAU_RLAR__NSC     BIT ( 1)
#define   REG_SAU_RLAR__LADDR   BITS( 5,31)

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

/* DCB: Debug Control Block {{{4
 * ------------------------
 */
#define REG_DHCSR     SECREG(REG_DHCSR)
#define REG_DHCSR_S   0xE000'EDF0
#define REG_DHCSR_NS  0xE002'EDF0
#define   REG_DHCSR__S_HALT     BIT(17)
#define   REG_DHCSR__S_LOCKUP   BIT(19)
#define   REG_DHCSR__S_SDE      BIT(20)
#define   REG_DHCSR__C_DEBUGEN      BIT ( 0)
#define   REG_DHCSR__C_HALT         BIT ( 1)
#define   REG_DHCSR__C_STEP         BIT ( 2)
#define   REG_DHCSR__C_MASKINTS     BIT ( 3)
#define REG_DEMCR     SECREG(REG_DEMCR)
#define REG_DEMCR_S   0xE000'EDFC
#define REG_DEMCR_NS  0xE002'EDFC
#define   REG_DEMCR__VC_CORERESET BIT( 0)
#define   REG_DEMCR__VC_MMERR     BIT( 4)
#define   REG_DEMCR__VC_NOCPERR   BIT( 5)
#define   REG_DEMCR__VC_CHKERR    BIT( 6)
#define   REG_DEMCR__VC_STATERR   BIT( 7)
#define   REG_DEMCR__VC_BUSERR    BIT( 8)
#define   REG_DEMCR__VC_INTERR    BIT( 9)
#define   REG_DEMCR__VC_HARDERR   BIT(10)
#define   REG_DEMCR__VC_SFERR     BIT(11)
#define   REG_DEMCR__MON_EN       BIT(16)
#define   REG_DEMCR__MON_PEND     BIT(17)
#define   REG_DEMCR__MON_STEP     BIT(18)
#define   REG_DEMCR__MON_REQ      BIT(19)
#define   REG_DEMCR__SDME         BIT(20)
#define   REG_DEMCR__TRCENA       BIT(24)
#define REG_DAUTHCTRL 0xE000'EE04
#define   REG_DAUTHCTRL__SPIDENSEL  BIT ( 0)
#define   REG_DAUTHCTRL__INTSPIDEN  BIT ( 1)
#define   REG_DAUTHCTRL__SPNIDENSEL BIT ( 2)
#define   REG_DAUTHCTRL__INTSPNIDEN BIT ( 3)

/* STIR {{{4
 * ----
 */

/* FPE: Floating-Point Extension {{{4
 * -----------------------------
 */
#define REG_FPCCR     SECREG(REG_FPCCR)
#define REG_FPCCR_S   0xE000'EF34
#define REG_FPCCR_NS  0xE002'EF34
#define   REG_FPCCR__LSPACT         BIT ( 0)
#define   REG_FPCCR__USER           BIT ( 1)
#define   REG_FPCCR__S              BIT ( 2)
#define   REG_FPCCR__THREAD         BIT ( 3)
#define   REG_FPCCR__HFRDY          BIT ( 4)
#define   REG_FPCCR__MMRDY          BIT ( 5)
#define   REG_FPCCR__BFRDY          BIT ( 6)
#define   REG_FPCCR__SFRDY          BIT ( 7)
#define   REG_FPCCR__MONRDY         BIT ( 8)
#define   REG_FPCCR__SPLIMVIOL      BIT ( 9)
#define   REG_FPCCR__UFRDY          BIT (10)
#define   REG_FPCCR__TS             BIT (26)
#define   REG_FPCCR__CLRONRETS      BIT (27)
#define   REG_FPCCR__CLRONRET       BIT (28)
#define   REG_FPCCR__LSPENS         BIT (29)
#define   REG_FPCCR__LSPEN          BIT (30)
#define   REG_FPCCR__ASPEN          BIT (31)

#define REG_FPCAR_S   0xE000'EF38
#define REG_FPCAR_NS  0xE002'EF38

#define REG_FPDSCR_S  0xE000'EF3C
#define REG_FPDSCR_NS 0xE002'EF3C
#define   REG_FPDSCR__RMODE     BITS(22,23)
#define   REG_FPDSCR__FZ        BIT (24)
#define   REG_FPDSCR__DN        BIT (25)
#define   REG_FPDSCR__AHP       BIT (26)

/* Cache Maintenance Operations {{{4
 * ----------------------------
 */

/* DIB: Debug Identification Block {{{4
 * -------------------------------
 */

/* TPIU: Trace Port Interface Unit {{{4
 * -------------------------------
 */

/* Architectural Registers {{{4
 * -----------------------
 */
#define PRIMASK__PM     BIT(0)
#define FAULTMASK__FM   BIT(0)

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

#define MSPLIM__LIMIT     BITS( 3,31)
#define PSPLIM__LIMIT     BITS( 3,31)

/* Payloads {{{4
 * --------
 */
#define RETPSR__EXCEPTION BITS( 0, 8)
#define RETPSR__SPREALIGN BIT ( 9)
#define RETPSR__IT_ICI_LO BITS(10,15)
#define RETPSR__IT_ICI_HI BITS(25,26)
#define RETPSR__GE        BITS(16,19)
#define RETPSR__SFPA      BIT (20)
#define RETPSR__T         BIT (24)

#define TT_RESP__MREGION    BITS( 0, 7)
#define TT_RESP__SREGION    BITS( 8,15)
#define TT_RESP__MRVALID    BIT (16)
#define TT_RESP__SRVALID    BIT (17)
#define TT_RESP__R          BIT (18)
#define TT_RESP__RW         BIT (19)
#define TT_RESP__NSR        BIT (20)
#define TT_RESP__NSRW       BIT (21)
#define TT_RESP__S          BIT (22)
#define TT_RESP__IRVALID    BIT (23)
#define TT_RESP__IREGION    BITS(24,31)

/* Special Values {{{3
 * --------------
 */
#define EXC_RETURN__ES      BIT ( 0)
#define EXC_RETURN__SPSEL   BIT ( 2)
#define EXC_RETURN__MODE    BIT ( 3)
#define EXC_RETURN__FTYPE   BIT ( 4)
#define EXC_RETURN__DCRS    BIT ( 5)
#define EXC_RETURN__S       BIT ( 6)
#define EXC_RETURN__PREFIX  BITS(24,31)

/* CPU State {{{3
 * ---------
 */
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

enum :uint32_t {
  EXIT_CAUSE__NORMAL          = 0,
  EXIT_CAUSE__WFI             = BIT(0), // Last instruction was WFI
  EXIT_CAUSE__WFE             = BIT(1), // Last instruction was WFE
  EXIT_CAUSE__YIELD           = BIT(2), // Last instruction was YIELD
  EXIT_CAUSE__DBG             = BIT(3), // Last instruction was DBG. This is architecturally a NOP but is reported here for debugging use.
  EXIT_CAUSE__SLEEP_ON_EXIT   = BIT(4), // We just exited an exception and SLEEPONEXIT is enabled.
};

struct CpuState {
  // General-purpose registers, including the banked versions of SP.
  union {
    struct {
      uint32_t r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,spMainNS,spProcessNS,lr,pc,spMainS,spProcessS;
    };
    uint32_t r[_RName_Max];
  };

  // Special architectural registers, accessed via MRS/MSR or VMRS/VMSR.
  uint32_t xpsr, psplimNS, psplimS, msplimNS, msplimS, fpscr;
  uint32_t primaskNS, primaskS, faultmaskNS, faultmaskS, basepriNS, basepriS, controlNS, controlS;

  // Security state (Secure or Non-Secure).
  SecurityState curState;

  // Exception enable, active, and pending flags.
  // For exceptions which are banked between security states (e.g. MemManage,
  // UsageFault, SVCall, PendSV, and SysTick if dual SysTick timers are present):
  //   bit 0: Secure enabled/active/pending (unused if no security ext)
  //   bit 1: Non-Secure enabled/active/pending
  // For unbanked exceptions (e.g. NMI, BusFault, DebugMonitor, SysTick if only
  // a single SysTick timer is present, and external interrupts), these are
  // 0b11 if enabled/active/pending and 0b00 otherwise.
  uint8_t excEnable[NUM_EXC];
  uint8_t excActive[NUM_EXC];
  uint8_t excPending[NUM_EXC];

  // Floating point registers.
  uint64_t d[16];

  // Miscellaneous flags, including the event flag.
  bool event;                     // The event flag.
  bool pendingReturnOperation;
  bool itStateChanged;            // Used to override next ITSTATE.
  bool pcChanged;                 // Used to override next instruction address, e.g. when branching.
  uint8_t nextInstrITState;       // Used only if itStateChanged is set. New ITSTATE.
  uint32_t nextInstrAddr;         // Used only if pcChanged is set. Branch target address.

  // Information about the current instruction.
  uint32_t thisInstr;             // instruction encoding
  uint8_t  thisInstrLength;       // in bytes (2 or 4, or 0 if in lockup)
  uint32_t thisInstrDefaultCond;

  // Implementation-specific state. If set to a non-negative value, this is
  // used for _CurrentCond() instead of thisInstrDefaultCond. Can be used in
  // the decoder of a specific instruction to override the condition code (e.g.
  // for branch instructions which have a condition code field rather than
  // using ITSTATE). Defaults and is reset to -1 at the start of each cycle.
  char      curCondOverride;

  // Implementation-specific state. The simulator is advanced by calling
  // TopLevel in a loop. The caller may wish to control loop flow according to
  // a number of conditions, such as whether a WFI instruction has just been
  // executed. Therefore we arrange a way to report to the caller if the last
  // instruction executed was "interesting". If exitCause is 0, the last
  // instruction executed was not interesting, otherwise it is one of the
  // values in EXIT_CAUSE__*.
  uint32_t  exitCause;
};

/* Nest State {{{3
 * ----------
 */
struct CpuNest {
  uint32_t dwtCtrl{};               // REG_DWT_CTRL, res0 if !DWT, unbanked, may not be accessible to SW if !Main
  uint32_t dwtComp[15]{};           // REG_DWT_COMPn, res0 if !DWT, unbanked, may not be accessible to SW if !Main
  uint32_t dwtFunction[15]{};       // REG_DWT_FUNCTIONn, res0 if !DWT, unbanked, may not be accessible to SW if !Main
  uint32_t fpCtrl{};                // REG_FP_CTRL, res0 if !FPB, unbanked, may not be accessible to SW if !Main
  uint32_t fpComp[NUM_FPB_COMP]{};  // REG_FP_COMPn, res0 if !FPB, unbanked, may not be accessible to SW if !Main
  uint32_t cppwrS{}, cppwrNS{};     // REG_CPPWR, res0 if !Main, has NS alias, unbanked

  uint32_t nvicItns[16]{};          // REG_NVIC_ITNSn, unbanked, NS: RAZ/WI
  uint32_t nvicIpr[124]{};          // REG_NVIC_IPRn, unbanked

  uint32_t cfsrS{}, cfsrNS{};
  uint32_t hfsrS{}, hfsrNS{};
  uint32_t dfsrS{}, dfsrNS{};
  uint32_t mmfarS{}, mmfarNS{};
  uint32_t bfarS{}, bfarNS{};

  uint32_t shpr1S{}, shpr1NS{}, shpr2S{}, shpr2NS{}, shpr3S{}, shpr3NS{};
  uint32_t ccrS{}, ccrNS{};
  uint32_t scrS{}, scrNS{};
  uint32_t aircrS{}, aircrNS{};
  uint32_t cpacrS{}, cpacrNS{};
  uint32_t nsacr{};

  uint32_t mpuTypeS{}, mpuTypeNS{};
  uint32_t mpuCtrlS{}, mpuCtrlNS{};
  uint32_t mpuRnrS{}, mpuRnrNS{};
  uint32_t mpuMair0S{}, mpuMair0NS{}, mpuMair1S{}, mpuMair1NS{};
  uint32_t mpuRbarS[NUM_MPU_REGION_S]{}, mpuRbarNS[NUM_MPU_REGION_NS]{};
  uint32_t mpuRlarS[NUM_MPU_REGION_S]{}, mpuRlarNS[NUM_MPU_REGION_NS]{};

  uint32_t sauCtrl{};
  uint32_t sauRnr{};
  uint32_t sauRbar[NUM_SAU_REGION]{};
  uint32_t sauRlar[NUM_SAU_REGION]{};

  uint32_t sfsr{};
  uint32_t sfar{};

  uint32_t dauthCtrl{};

  uint32_t fpccrS{}, fpccrNS{}; // unbanked bits live in fpccrS
  uint32_t fpcarS{}, fpcarNS{};
  uint32_t fpdscrS{}, fpdscrNS{};

  uint32_t vtorS{}, vtorNS{};

  uint32_t icsr{};
  uint32_t dhcsr{};
  uint32_t demcr{};

  uint32_t systCsrS{REG_SYST_CSR__CLKSOURCE}, systCsrNS{REG_SYST_CSR__CLKSOURCE};
  uint32_t systRvrS{}, systRvrNS{};
  uint32_t systCalibS{}, systCalibNS{};
};

/* IDevice {{{3
 * -------
 */
enum :uint32_t {
  DEBUG_PIN__DBGEN      = BIT(0),
  DEBUG_PIN__NIDEN      = BIT(1),
  DEBUG_PIN__SPIDEN     = BIT(2),
  DEBUG_PIN__SPNIDEN    = BIT(3),
};

enum :uint32_t {
  LS_FLAG__ATYPE__MASK    = BITS(0,2),
    LS_FLAG__ATYPE__NORMAL   = AccType_NORMAL,
    LS_FLAG__ATYPE__ORDERED  = AccType_ORDERED,
    LS_FLAG__ATYPE__STACK    = AccType_STACK,
    LS_FLAG__ATYPE__LAZYFP   = AccType_LAZYFP,
    LS_FLAG__ATYPE__IFETCH   = AccType_IFETCH,
    LS_FLAG__ATYPE__VECTABLE = AccType_VECTABLE,
  LS_FLAG__PRIV           = BIT (3),
  LS_FLAG__NS             = BIT (4),
  LS_FLAG__DEVICE         = BIT (5),
  LS_FLAG__DEVTYPE__MASK  = BITS(6,7),
    LS_FLAG__DEVTYPE__GRE     = DeviceType_GRE,
    LS_FLAG__DEVTYPE__nGRE    = DeviceType_nGRE,
    LS_FLAG__DEVTYPE__nGnRE   = DeviceType_nGnRE,
    LS_FLAG__DEVTYPE__nGnRnE  = DeviceType_nGnRnE,
  LS_FLAG__IATTR__MASK    = BITS(8,9),
    LS_FLAG__IATTR__NC        = 0,
    LS_FLAG__IATTR__WB        = 1,
    LS_FLAG__IATTR__WT        = 2,
  LS_FLAG__OATTR__MASK    = BITS(10,11),
    LS_FLAG__OATTR__NC        = 0,
    LS_FLAG__OATTR__WB        = 1,
    LS_FLAG__OATTR__WT        = 2,
  LS_FLAG__IHINT__MASK    = BITS(12,13),
    LS_FLAG__IHINT__NO_ALLOC  = 0,
    LS_FLAG__IHINT__WALLOC    = 1,
    LS_FLAG__IHINT__RALLOC    = 2,
    LS_FLAG__IHINT__RWALLOC   = 3,
  LS_FLAG__OHINT__MASK    = BITS(14,15),
    LS_FLAG__OHINT__NO_ALLOC  = 0,
    LS_FLAG__OHINT__WALLOC    = 1,
    LS_FLAG__OHINT__RALLOC    = 2,
    LS_FLAG__OHINT__RWALLOC   = 3,
  LS_FLAG__ITRANSIENT     = BIT(16),
  LS_FLAG__OTRANSIENT     = BIT(17),
  LS_FLAG__SHAREABLE      = BIT(18),
  LS_FLAG__OSHAREABLE     = BIT(19),
  LS_FLAG__WRITE          = BIT(20), // Implied by whether Load*() or Store*() is called, but included anyway

  LS_FLAG__DEFAULT        = 0,
};

static inline uint32_t MaskBySize(uint32_t v, int size) {
  return v & BITS(0, 8*size-1);
}

struct IDevice {
  // Load/store. addr is a physical address. is a size must be in {1,2,4} and
  // specifies the size of the load/store in bytes. flags are LS_FLAG__*.
  // When calling Load, if size is not 4, the implementation must ensure that
  // the bits [size*8:31] are zero. When calling Store, if size is not 4, the
  // caller must ensure that bits [size*8:31] are zero. Failure to meet either
  // of these requirements results in undefined behaviour. Return nonzero on
  // error/BusFault.
  virtual int Load(phys_t addr, int size, uint32_t flags, uint32_t &v) = 0;
  virtual int Store(phys_t addr, int size, uint32_t flags, uint32_t v) = 0;

  virtual std::tuple<bool, bool, bool, uint8_t, bool> IDAUCheck(uint32_t addr) {
    return {false, true, true, 0, false};
  }

  // DEBUG_PIN__*
  virtual uint32_t DebugPins() const { return 0; }
};

/* SimpleSimulatorConfig {{{2
 * --------------------
 * SimpleSimulatorConfig implements the SimulatorConfig concept. Any object can
 * be used so long as it can be copied and it supports the methods shown below.
 *
 * A static implementation with constexpr methods could be used to enable
 * compile-time removal of branches unrelated to the chosen profile.
 */
struct SimpleSimulatorConfig {
  bool HaveMainExt() const { return this->main; }
  bool HaveSecurityExt() const { return this->security; }
  bool HaveFPB() const { return this->fpb; }
  bool HaveDWT() const { return this->dwt; }
  bool HaveITM() const { return this->itm; }
  bool HaveFPExt() const { return this->fpExt; }
  int SysTick() const { return this->sysTick; }
  bool HaveHaltingDebug() const { return this->haltingDebug; }
  bool HaveDSPExt() const { return this->dspExt; }
  uint8_t NumMpuRegionS() const { return this->numMpuRegionS; }
  uint8_t NumMpuRegionNS() const { return this->numMpuRegionNS; }
  uint8_t NumSauRegion() const { return this->numSauRegion; }
  int MaxExc() const { return this->maxExc; }
  uint32_t InitialVtor() const { return this->initialVtor; }
  int IsaVersion() const { return this->isaVersion; }
  uint64_t SystIntFreq() const { return this->systIntFreq; }
  uint64_t SystExtFreq() const { return this->systExtFreq; }
  uint8_t PriorityBits() const { return this->priorityBits; }

  // -----------------------------------------
  bool      main            = true;         // Whether the simulated CPU supports the Main extension.
  bool      security        = true;         // Whether the simulated CPU supports the Security extension.
  bool      fpb             = true;         // Whether the simulated CPU supports the Flash Patch extension.
  bool      dwt             = true;         // Whether the simulated CPU supports the DWT extension.
  bool      itm             = true;         // Whether the simulated CPU supports the ITM extension.
  bool      fpExt           = true;         // Whether the simulated CPU supports the floating point extension.
  int       sysTick         = 2;            // Whether the simulated CPU supports the SysTick feature. 1=Single, 2=Dual
  bool      haltingDebug    = true;         // Whether the simulated CPU supports the halting debug feature.
  bool      dspExt          = false;        // Whether the simulated CPU supports the DSP extension.
  uint8_t   numMpuRegionS   = NUM_MPU_REGION_S;   // Number of MPU regions supported. 0 for no MPU.
  uint8_t   numMpuRegionNS  = NUM_MPU_REGION_NS;  // May not exceed NUM_MPU_REGION_{S,NS}.
  uint8_t   numSauRegion    = NUM_SAU_REGION;     // May not exceed NUM_SAU_REGION. 0 for no SAU.
  //uint32_t  cpuid           = 0xFFFF'FFFF;// Specifies the value of the CPUID register.
  int       maxExc          = NUM_EXC-1;    // Maximum number of exceptions supported.
  uint32_t  initialVtor     = 0;            // Initial VTOR value
  int       isaVersion      = 8;            // ISA version (7 or 8)
  uint64_t  systIntFreq     = 100'000'000;  // SysTick frequency when CLKSOURCE=1 (PE).
  uint64_t  systExtFreq     = 0;            // SysTick frequency when CLKSOURCE=0 (external). 0 to disable.
  uint8_t   priorityBits    = 8;            // Number of priority bits [3, 8]. Ignored for !main.
};

/* DeadlineCaller {{{2
 * ==============
 * Simple utility class which calls a specified callback on another thread at a
 * specified deadline.
 */
struct DeadlineCaller {
  using time_point = std::chrono::steady_clock::time_point;

  DeadlineCaller() {}

  // Can be safely destroyed at any time. Any callback will be cancelled.
  ~DeadlineCaller() {
    {
      std::unique_lock lk{_m};
      _teardown = true;
      _cb       = nullptr;
    }
    _cv.notify_all();
    if (_t.joinable())
      _t.join();
  }

  /* Start {{{3
   * -----
   * Set a new deadline. If there is an existing deadline pending it is
   * cancelled. The function f will be called at or after the deadline and will
   * be passed the opaque value arg. Passing a default-constructed time_point
   * or a null f has the same effect as calling stop.
   */
  void Start(time_point deadline, void (*f)(void *arg), void *arg) {
    if (!deadline.time_since_epoch().count() || !f) {
      Stop();
      return;
    }

    _EnsureThread();

    {
      std::unique_lock lk{_m};
      _deadline = deadline;
      _cb       = f;
      _cbArg    = arg;
    }
    _cv.notify_all();
  }

  /* Stop {{{3
   * ----
   * Cancel any existing deadline, if any.
   */
  void Stop() {
    std::unique_lock lk{_m};
    _cb = nullptr;
  }

private:
  void _EnsureThread() {
    if (_t.joinable())
      return;

    _t = std::thread{[this]() { _TMain(); }};
  }

  /* _TMain {{{3
   * ------
   */
  void _TMain() {
    std::unique_lock lk{_m};
    for (;;) {
      if (_teardown)
        break;

      if (_cb)
        _cv.wait_until(lk, _deadline);
      else
        _cv.wait(lk);

      if (_cb && std::chrono::steady_clock::now() >= _deadline) {
        auto cb    = _cb;
        auto cbArg = _cbArg;
        _cb = nullptr;
        lk.unlock();
        cb(cbArg);
        lk.lock();
      }
    }
  } // }}}3

private:
  std::mutex                  _m;
  std::condition_variable_any _cv;                // protected by _m
  time_point                  _deadline;          //
  bool                        _teardown{};        //
  void                      (*_cb)(void *arg){};  //
  void                       *_cbArg;             //
  std::thread                 _t;
};

/* ISysTickDevice {{{2
 * ==============
 * ISysTickDevice implements the SysTickDevice concept.
 */
struct ISysTickDevice {
  /* SysTickSetConfig {{{3
   * ----------------
   * Called whenever the configuration of a SysTick timer is updated; whenever
   * the CSR, RVR or CVR is written. freq is in Hz. tickInt is the value of
   * REG_SYST_CSR__TICKINT. The reload value is updated to the given value
   * which must be in range [0, 2**24-1]. If curValue is -1, it is not updated,
   * else it is set to the given value.
   */
  virtual void SysTickSetConfig(bool enable, bool tickInt, uint64_t freq, uint32_t reloadValue, int curValue) = 0;

  /* SysTickGetConfig {{{3
   * ----------------
   * Retrieves {enable, tickInt, freq, reloadValue}.
   */
  virtual std::tuple<bool, bool, uint64_t, uint32_t> SysTickGetConfig() = 0;

  /* SysTickGetCurrent {{{3
   * -----------------
   * Gets the current value of the timer. The value must be less than 2**24.
   * The value counts down.
   */
  virtual uint32_t SysTickGetCurrent() = 0;

  /* SysTickGetCountFlag {{{3
   * -------------------
   * Gets and optionally clears the count flag.
   */
  virtual bool SysTickGetCountFlag(bool clear) = 0;

  /* SysTickGetIntrFlag {{{3
   * ------------------
   * Gets and optionally clears the interrupt pending flag. tickInt must have been enabled.
   */
  virtual bool SysTickGetIntrFlag(bool clear) = 0;

  /* SysTickSetCallback {{{3
   * ------------------
   * Sets a callback to be called on an unspecified thread whenever the tick
   * interrupt occurs. Both tickInt and the timer itself must be enabled. To
   * disable the callback after enabling it, pass nullptr.
   *
   * As a special case, when f is specified as nullptr, the method must only
   * return once it is guaranteed that no call to the old callback function is
   * currently taking place or will take place in the future. Changes from one
   * non-nullptr value of f to another non-nullptr value of f do not require
   * the implementation to offer this guarantee.
   */
  virtual void SysTickSetCallback(void (*f)(void *arg), void *arg) = 0;
};

/* SysTickDevice_Real {{{2
 * ==================
 * This is a SysTick emulator which uses real-world time to model the timer. We
 * use a system monotonic clock for this purpose.
 *
 * We implement the emulation as follows. There are two ways of expressing time
 * used by this implementation: Epoch time and SysTick time. Our internal
 * representation is Epoch time and we convert to SysTick time whenever queries
 * are made against us (e.g. when SYST_CVR is read).
 *
 * Epoch time denotes time as the amount of time since an epoch. The epoch used
 * is not the UNIX epoch but the time at which the SysTick emulator was last
 * reconfigured in a way that affects the process of conversion between Epoch
 * and SysTick time. The idea is that between any two such reconfiguration
 * events, there is a fixed, linear (albeit modular) correspondance between
 * Epoch and SysTick time. This means we don't have to perform any periodic
 * computations to "maintain" the timer, and we consume no CPU time just
 * tracking the advancement of time.
 *
 * Suppose there are two reconfiguration events C-1 and C-2. The passage of
 * time might look like this:
 *
 *    Time (t) --->
 *
 *      C-1                                                    C-2
 *       |                                                      |
 *       |                                                      |
 *      E=t                                                    E=t'
 *
 * At C-1 the epoch E is set to the current time t. For any subsequent reading
 * of t (prior to event C-2) we can thus determine the amount of time which has
 * passed since C-1. There is a linear albeit modular correspondance between
 * the current value of (t-E) and the current value of the SysTick timer for
 * this entire period between C-1 and C-2. The SysTick timer is a down-counter,
 * therefore this produces a pattern like this:
 *
 *         SYST_RVR
 *            |\    \
 *            | \    \
 *            |  \    \
 *   SYST_CVR |   \    \
 *            |    \    \
 *            |-----------
 *                   t -->
 *
 * We represent the number of clock cycles since the current epoch E as an
 * unsigned 64-bit integer. The fastest Cortex-M, the Cortex-M7, can run at up
 * to 600 MHz; in this case we would be able to count clock cycles for about
 * 975 years before facing an overflow.
 */
struct SysTickDevice_Real final :ISysTickDevice {
  void SysTickSetConfig(bool enable, bool tickInt, uint64_t freq, uint32_t reloadValue, int curValue) override {
    std::unique_lock lk{_m};

    if (enable != _enable || reloadValue != _reload || freq != _freq || (curValue >= 0 && curValue < BIT(24))) {
      assert(freq);

      // The device has been changed from being enabled to not enabled or vice
      // versa; or the reload value has been changed; or the frequency has been
      // changed; or the current value has been set directly. In any case this
      // means our linear correspondance between Epoch and SysTick time has now
      // changed, so we need to begin a new epoch.
      //
      // Note that the SysTick SYST_CVR register itself clears to zero whenever
      // it is written, rather than to the written value, but we implement the
      // setting of arbitrary current values in this class itself.
      uint32_t newInitialCur = (curValue >= 0 && curValue < BIT(24)) ? curValue : SysTickGetCurrent();

      _epoch      = std::chrono::steady_clock::now();
      _enable     = enable;
      _freq       = freq;
      _reload     = reloadValue;
      _initialCur = newInitialCur;
    }

    _tickInt = tickInt;
    _XUpdateCallback();
  }

  std::tuple<bool, bool, uint64_t, uint32_t> SysTickGetConfig() override {
    return {_enable, _tickInt, _freq, _reload};
  }

  uint32_t SysTickGetCurrent() override {
    auto [cur, _] = _GetCurrentAndEra();
    return cur;
  }

  bool SysTickGetCountFlag(bool clear) override {
    auto [_, era] = _GetCurrentAndEra();

    bool eraChanged = (_lastCountFlagEra != era);
    if (clear)
      _lastCountFlagEra = era;

    return eraChanged;
  }

  bool SysTickGetIntrFlag(bool clear) override {
    auto [_, era] = _GetCurrentAndEra();

    bool intr = (_lastIntrEra != era);
    if (clear)
      _lastIntrEra = era;

    return _tickInt && intr;
  }

  void SysTickSetCallback(void (*f)(void *arg), void *arg) override {
    std::unique_lock lk{_m};

    _cb     = f;
    _cbArg  = arg;

    if (!_cb) {
      // If we are clearing the callback, ensure that _TCallback()
      // has not already loaded old values of (_cb, _cbArg) and is about
      // to call them. When SysTickSetCallback is called with a nullptr cb,
      // it is guaranteed that no further calls to the old callback will
      // occur once it returns.
      std::unique_lock lk2{_mfull};
      return;
    }

    auto [_, era] = _GetCurrentAndEra();
    _lastCbEra = era;

    _XUpdateCallback();
  }

private:
  using time_point = std::chrono::steady_clock::time_point;
  // Functions beginning _X might be called on any thread (i.e., on the thread
  // driving this object or the callback thread). Functions beginning _T are
  // called only on our callback thread.
  //
  // Retrieve the number of virtual SysTick clock cycles which have occured
  // since the last epoch. This is determined by _epoch, the current time, and
  // _freq, our SysTick frequency in Hz. Implemented using fixed point
  // arithmetic to ensure perfect precision.
  uint64_t _GetClockCyclesSinceEpoch() {
    auto now = std::chrono::steady_clock::now();
    auto d   = _enable ? now - _epoch : decltype(now - _epoch){};

    // We temporarily convert to a nanosecond representation here to find the
    // number of nanoseconds to add to the integral number of seconds. This
    // limits the duration d to be about 292 years assuming a signed 64-bit
    // representation.
    uint64_t ds  = std::chrono::duration_cast<std::chrono::seconds>(d).count(); // integer seconds since epoch
    uint64_t dns = (std::chrono::duration_cast<std::chrono::nanoseconds>(d) % std::chrono::nanoseconds(1'000'000'000)).count(); // remainder in ns

    return ds*_freq + (dns*_freq)/1'000'000'000; // number of clock cycles since epoch
  }

  // Retrieve the current value of the current value register, as well as an
  // era value. The current value counts down from 2**24-1. The era value
  // increases by one every time the current value wraps, and thus allows us to
  // determine when to report that the count flag is set.
  std::tuple<uint32_t, uint64_t> _GetCurrentAndEra() {
    uint64_t cycles = _GetClockCyclesSinceEpoch() + (_reload - _initialCur);

    uint32_t cur = _reload - (cycles % (_reload+1));
    uint64_t era = cycles / (_reload+1);

    return {cur, era};
  }

  // Determine the deadline at which the era will change.
  time_point _XGetDeadline() { // must hold lock
    if (!_enable || !_tickInt || !_cb)
      return {};

    //(era+1)*(_reload+1) - (_reload - _initialCur) = ((DLINE - _epoch) AS seconds)*_freq + (((DLINE - _epoch) AS ns)*_freq)/1'000'000'000;
    //((era+1)*(_reload+1) - (_reload - _initialCur)) = ((DLINE - epoch) AS seconds)*_freq; // logical
    //((era+1)*(_reload+1) - (_reload - _initialCur))/_freq = (DLINE - epoch) AS seconds; // logical
    //FromSeconds(((era+1)*(_reload+1) - (_reload - _initialCur))/_freq) = (DLINE - epoch); // logical
    //FromSeconds(((era+1)*(_reload+1) - (_reload - _initialCur))/_freq) + epoch = DLINE; // logical

    uint64_t nsSinceEpoch = ((_lastCbEra+1)*(_reload+1) - (_reload - _initialCur))*1'000'000'000/_freq;

    return _epoch + std::chrono::nanoseconds(nsSinceEpoch);
  }

  // Functions beginning _X might be called on any thread (i.e., on the thread
  // driving this object or the callback thread).
  void _XUpdateCallback() { // must hold lock
    _dc.Start(_XGetDeadline(), [](void *arg) { static_cast<SysTickDevice_Real*>(arg)->_TCallback(); }, this);
  }

  // Functions beginning _T are called on our callback thread.
  void _TCallback() {
    // Additional lock for the entire _TCallback() call which we do not unlock
    // during the cb() invocation. This allows us to ensure that a _TCallback()
    // call is not in progress during a clearing of the callback function
    // (SysTickSetCallback(nullptr, nullptr)). If SysTickSetCallback(nullptr, nullptr) is in progress
    // when we start executing _TCallback(), it may have already acquired _m
    // and will then wait forever for _mfull, so we need to use a deadlock
    // avoidance algorithm here.
    std::unique_lock lk2{_mfull, std::defer_lock};
    std::unique_lock lk {_m,     std::defer_lock};
    std::lock(lk2, lk);
    auto cb     = _cb;
    auto cbArg  = _cbArg;

    if (cb) {
      lk.unlock();
      cb(cbArg);
      lk.lock();

      ++_lastCbEra;
      _XUpdateCallback();
    }
  }

private:
  // Mutex.
  std::mutex  _m, _mfull;

  // Values updated by SetConfig.
  bool        _enable{}, _tickInt{};
  uint64_t    _freq{};
  uint32_t    _reload{};
  uint32_t    _initialCur{};
  time_point  _epoch{};

  // Internal data.
  uint64_t    _lastCountFlagEra{}, _lastIntrEra{}, _lastCbEra{};

  // Callback handling.
  DeadlineCaller  _dc;
  void          (*_cb)(void *arg){};
  void           *_cbArg{};
};

/* IntrBox {{{2
 * =======
 * An interrupt box is used to manage the delivery of external and NMI
 * interrupts to the Simulator. SysTick is handled internally by the simulator.
 *
 * The Simulator itself is not thread-safe and access to it must be
 * synchronized. This utility enables easy threaded delivery of external
 * interrupts and NMI events to the Simulator and allows WFI to be implemented
 * by a top-level loop.
 *
 * When using this class, it is essential that TriggerNMI and TriggerExtInt
 * always be called using this class and the equivalent methods on the
 * Simulator class not be called directly, as IntrBox needs to know about these
 * events so it can cause its WaitForInterrupt method to return. Unlike the
 * equivalent methods on the simulator class, these methods can be called from
 * arbitrary threads.
 *
 * If you want to access the simulator from multiple threads, you can use the
 * mutex accessed via the GetMutex() method to synchronize such access.
 */
template<typename Sim>
struct IntrBox {
  IntrBox(Sim &sim) :_sim(sim) {
    int numSysTick = _sim.GetNumSysTick();
    if (numSysTick)
      _sim.GetSysTick(false).SysTickSetCallback([](void *arg) { static_cast<IntrBox*>(arg)->_TCallback(false); }, this);
    if (numSysTick == 2)
      _sim.GetSysTick(true).SysTickSetCallback([](void *arg) { static_cast<IntrBox*>(arg)->_TCallback(true); }, this);
  }

  ~IntrBox() {
    std::unique_lock lk{_m};

    int numSysTick = _sim.GetNumSysTick();
    if (numSysTick)
      _sim.GetSysTick(false).SysTickSetCallback(nullptr, nullptr);
    if (numSysTick == 2)
      _sim.GetSysTick(true).SysTickSetCallback(nullptr, nullptr);
  }

  // Waits for an interrupt condition. Returns immediately if the simulator
  // already has an exception which it can (but for the fact that PRIMASK might
  // be set) take immediately. Otherwise, returns whenever an exception next
  // occurs, which could be SysTick, or a call to TriggerNMI or TriggerExtInt
  // on this class.
  void WaitForInterrupt() {
    std::unique_lock lk{_m};

    for (;;) {
      if (_sim.IsExceptionPending(/*ignorePrimask=*/true))
        return;

      _cv.wait(lk);
    }
  }

  // Inject an NMI. Unlike the equivalent method on Sim, can be called from any
  // thread.
  void TriggerNMI() {
    std::unique_lock lk{_m};
    _sim.TriggerNMI();
    _XWakeupEvent();
  }

  // Inject an external interrupt. Unlike the equivalent method on Sim,
  // can be called from any thread.
  void TriggerExtInt(uint32_t intrNo, bool setNotClear=true) {
    std::unique_lock lk{_m};
    _sim.TriggerExtInt(intrNo, setNotClear);
    if (setNotClear)
      _XWakeupEvent();
  }

  // Trigger a generic WFI wakeup event. This is useful for waking up from WFI
  // in the event of a non-interrupt wakeup event (e.g. reset, implementation
  // defined wakeup events, etc.)
  // May be called from any thread.
  void WakeupEvent() {
    std::unique_lock lk{_m};
    _XWakeupEvent();
  }

  // Accesses a mutex which can be used to synchronize access to Simulator.
  std::mutex &GetMutex() { return _m; }

private:
  void _XWakeupEvent() { // can be called on any thread, must hold lock
    _cv.notify_all();
  }

  // Called on SysTick callback thread only.
  void _TCallback(bool ns) {
    std::unique_lock lk{_m};
    _XWakeupEvent();
  }

private:
  Sim &_sim;
  std::mutex              _m;
  std::condition_variable _cv;
};

/* Internal Helpers: MonitorState {{{2
 * ==============================
 */
template<typename T>
static inline T SatSub(T x, T y) {
  T r;
  if (__builtin_sub_overflow(x, y, &r))
    return std::numeric_limits<T>::min();
  else
    return r;
}

template<typename T>
static inline T SatAdd(T x, T y) {
  T r;
  if (__builtin_add_overflow(x, y, &r))
    return std::numeric_limits<T>::max();
  else
    return r;
}

struct MonitorState {
  bool ContainsAny(phys_t a, uint32_t sz) {
    ASSERT(sz);
    return size && a >= SatSub<phys_t>(addr, sz-1) && a <= SatAdd<phys_t>(addr, size-1);
  }

  bool ContainsAll(phys_t a, uint32_t sz) {
    ASSERT(sz);
    return size && a >= addr && SatAdd<phys_t>(a, sz-1) <= SatAdd<phys_t>(addr, size-1);
  }

  phys_t    addr{};
  uint32_t  size{}; // zero: open access state, nonzero: exclusive access state
};

/* GlobalMonitor {{{2
 * =============
 * Implements an ARMv8-M global monitor to be shared between PEs.
 *
 * ALL methods of this class are non-thread-safe and require locking the mutex
 * by calling Lock() and holding the std::unique_lock returned by it, before
 * attempting to use any other method of this class. This locking is done
 * outside of GlobalMonitor to provide flexibility in usage patterns (e.g.,
 * calling IsExclusive followed by ClearExclusiveByAddress while processing a
 * STREX, which must be done atomically).
 *
 * For simplicity the mutex used is a recursive mutex, so nested Lock calls can
 * be made. This simplifies our simulator code.
 * 
 * TODO § B7.3 R_HLHS: If the global monitor is not implemented for an address
 * range or memory type: ... (BusFault, etc.)
 */
struct GlobalMonitor {
  using lock_type = std::unique_lock<std::recursive_mutex>;

  GlobalMonitor(bool checkAddresses=true) :_checkAddresses(checkAddresses) {}
  ~GlobalMonitor() { Lock(); }

  lock_type Lock() { return lock_type{_m}; }

  // Mark a range covering at least [addr, addr+size) as belonging to the
  // specified PE.
  void MarkExclusive(phys_t addr, int procID, uint32_t size) {
    auto &s = _states[procID];

    // § B7.3.1 R_MPKM: "A Load-Exclusive instruction by one PE has no effect
    // on the global monitor state for any other PE." We do not call
    // _ClearExclusiveByAddress here.

    s.addr = addr;
    s.size = size;
  }

  // Clear global exclusive monitor for all PEs except the one specified.
  void ClearExclusiveByAddress(phys_t addr, int exceptProcID, uint32_t size) {
    for (auto &x :_states)
      if (x.first != exceptProcID && x.second.ContainsAny(addr, size))
        x.second.size = 0;
  }

  // Is there a global record for an extent of address space covering at least
  // [addr, addr+size) marked as belong to the specified PE?
  bool IsExclusive(phys_t addr, int procID, uint32_t size) {
    auto &s = _states[procID];

    // § B7.3.1 R_MFGC: [...] "If no address is marked as exclusive access for
    // the requesting PE, the store does not succeed."
    if (!s.size)
      return false;

    // "If the address accessed is marked for exclusive access in the global
    // monitor state machine for any other PE than that state machine
    // transitions to Open Access state."
    // This is handled automatically in MemA_with_priv_security; successful
    // stores call ClearExclusiveByAddress on shareable memory (i.e., on memory
    // where the global monitor, i.e. this method, was consulted).

    // § B7.3.1 R_MFGC: [...] "If a different physical address is marked as
    // exclusive access for the requesting PE, it is implementation defined
    // whether the store succeeds or not."
    if (!_checkAddresses)
      return true;

    // § B7.3.1 R_MFGC: [...] "The store is guaranteed to succeed only if
    // the physical address accessed is marked as exclusive access for the
    // requesting PE and both the local monitor and the global monitor state
    // machines for the requesting PE are in the exclusive access state."
    return s.ContainsAll(addr, size);
  }

private:
  // We maintain one _State for each processor ID; "only a single outstanding
  // exclusive access to shareable memory for each PE" is supported by the ISA.
  // The range represented is [addr, addr+size). If size is zero, this record
  // is currently invalid, meaning that the entry for that PE is in the "Open
  // Access" state. If size is nonzero, the entry for that PE is in the
  // "Exclusive Access" state.
  std::recursive_mutex                  _m;
  std::unordered_map<int, MonitorState> _states;
  bool                                  _checkAddresses;
};

/* Simulator {{{2
 * =========
 */
template<typename Device=IDevice, typename SimulatorConfig=SimpleSimulatorConfig, typename SysTickDevice=SysTickDevice_Real, typename GlobalMonitor=GlobalMonitor>
struct Simulator {
  Simulator(Device &dev, GlobalMonitor &gm, const SimulatorConfig &cfg=SimulatorConfig(), int procID=0) :_dev(dev), _cfg(cfg), _procID(procID), _lm(IMPL_DEF_LOCAL_MON_CHECK_ADDR), _gm(gm) {
    ASSERT(cfg.MaxExc() < NUM_EXC);
    ASSERT(cfg.MaxExc() <= _MaxExceptionNum());

    ASSERT(cfg.IsaVersion() >= 7 && cfg.IsaVersion() <= 8);
    if (_HaveSecurityExt())
      ASSERT(cfg.IsaVersion() >= 8);

    if (_HaveMainExt()) {
      if (_HaveSecurityExt())
        ASSERT(cfg.SysTick() == 2);
      else
        ASSERT(cfg.SysTick() == 1);
    }

    ASSERT(cfg.NumMpuRegionS() <= NUM_MPU_REGION_S);
    ASSERT(cfg.NumMpuRegionNS() <= NUM_MPU_REGION_NS);
    ASSERT(cfg.NumSauRegion() <= NUM_SAU_REGION);

    _ColdReset();
  }

  /* Public Functions {{{3
   * ================
   */

  /* TopLevel {{{4
   * --------
   * Steps the core by one iteration, (potentially) executing one instruction.
   */
  void TopLevel() { _TopLevel(); }

  /* ColdReset {{{4
   * ---------
   * Performs a cold reset of the core.
   */
  void ColdReset() { _ColdReset(); }

  /* IsLockedUp {{{4
   * ----------
   * Returns true iff the core is in locked up state.
   */
  bool IsLockedUp() const {
    return GETBITSM(InternalLoad32(REG_DHCSR), REG_DHCSR__S_LOCKUP);
  }

#if 0
  /* IsHalted {{{4
   * --------
   * Returns true iff the core is in the halted (debug) state.
   */
  bool IsHalted() const {
    return GETBITSM(InternalLoad32(REG_DHCSR), REG_DHCSR__S_HALT);
  }
#endif

  /* GetExitCause {{{4
   * ------------
   * After each return from TopLevel, which processes (at most) one
   * instruction, the exit cause value is set to one of EXIT_CAUSE__*, which
   * indicates whether the last instruction processed was an "interesting"
   * instruction; that is, an instruction which may be grounds for changing the
   * control flow of the loop calling TopLevel() (e.g., WFI, WFE). If the
   * last instruction executed was not "interesting", this is set to zero.
   */
  uint32_t GetExitCause() const { return _s.exitCause; }

  /* GetLastInstruction {{{4
   * ------------------
   * Returns the encoding of the last executed instruction. The second return
   * value is the length of the instruction in bytes. If the core is locked up,
   * returns {0, 0}.
   */
  std::tuple<uint32_t, int> GetLastInstruction() const { return {_s.thisInstr, _s.thisInstrLength}; }

  /* TriggerNMI {{{4
   * ----------
   * Externally trigger an NMI exception as pending.
   */
  void TriggerNMI() {
    _SetPending(NMI, true, true);
  }

  /* TriggerExtInt {{{4
   * -------------
   * Externally trigger (or untrigger) an external interrupt. intrNo is the
   * external interrupt number; 16 will be added to it to get the exception
   * number, so this numbering starts at zero, not 16.
   */
  void TriggerExtInt(uint32_t intrNo, bool setNotClear=true) {
    ASSERT(16+intrNo < _cfg.MaxExc());
    _SetPending(16+intrNo, true, setNotClear);
  }

  /* DebugLoad {{{4
   * ---------
   * Performs a debug load from the simulated core's memory map, similar to an
   * access performed via a Cortex-M debug D-AHB slave as might be typically
   * used over JTAG.
   *
   * size:  1, 2 or 4. Width of transfer in bytes.
   * hprot: Value of HPROT[0:6] as it would appear on an AHB-Lite bus. Bit 6
   *   should be set for Non-Secure transfers and cleared for Secure transfers.
   *   The result is placed in the low bits of v. Accesses are always little
   *   endian and must be aligned. Returns nonzero on bus error.
   */
  int DebugLoad(phys_t addr, int size, uint32_t hprot, uint32_t &v) {
    ASSERT(size == 4 || size == 2 || size == 1);
    if (addr % size)
      return -1;

    AddressDescriptor ad{};
    ad.memAttrs.ns      = !!(hprot & BIT(6));
    ad.physAddr         = addr;
    ad.accAttrs.isPriv  = true;
    ad.accAttrs.accType = AccType_NORMAL;

    return _Load(ad, size, v);
  }

  /* DebugStore {{{4
   * ----------
   * Performs a debug store to the simulated core's memory map, similar to an
   * access performed via a Cortex-M debug D-AHB slave as might be typically
   * used over JTAG. See DebugLoad for arguments. Returns nonzero on bus error.
   */
  int DebugStore(phys_t addr, int size, uint32_t hprot, uint32_t v) {
    ASSERT(size == 4 || size == 2 || size == 1);
    if (addr % size)
      return -1;

    AddressDescriptor ad{};
    ad.memAttrs.ns      = !!(hprot & BIT(6));
    ad.physAddr         = addr;
    ad.accAttrs.isPriv  = true;
    ad.accAttrs.accType = AccType_NORMAL;

    return _Store(ad, size, v);
  }

  /* GetCpuState {{{4
   * -----------
   */
  CpuState &GetCpuState() { return _s; }

  /* GetCpuNest {{{4
   * ----------
   */
  CpuNest &GetCpuNest() { return _n; }

  /* GetNumSysTick {{{4
   * -------------
   * Returns the number of SysTick timers implemented. Return value is in range
   * [0,2].
   */
  int GetNumSysTick() { return _HaveSysTick(); }

  /* GetSysTick {{{4
   * ----------
   * Retrieves the specified SysTick device. If no SysTick is implemented, or
   * only one SysTick is implemented and ns is true, the behaviour is
   * undefined.
   */
  SysTickDevice &GetSysTick(bool ns) { return _SystResolve(ns); }

  /* IsExceptionPending {{{4
   * ------------------
   * Determines if an exception is pending to be taken immediately. If
   * ignorePrimask is true, the value of PRIMASK is ignored; that is, PRIMASK
   * is treated as though it is zero. Other priority criteria are still
   * checked. Setting ignorePrimask to true is useful for implementing the WFI
   * wakeup criterion.
   */
  bool IsExceptionPending(bool ignorePrimask) {
    auto [canTakeExc, _1, _2] = _PendingExceptionDetails(ignorePrimask);
    return canTakeExc;
  }

private:
  /* Memory-Mapped Register Implementation {{{3
   * =====================================
   */

  /* _NestReset {{{4
   * ----------
   */
  void _NestReset() {
    _n = CpuNest();

    // REG_DWT_CTRL
    if (_HaveDWT()) {
      _n.dwtCtrl = PUTBITSM(REG_DWT_CTRL__NUMCOMP, NUM_DWT_COMP);
      if (!_HaveMainExt())
        _n.dwtCtrl |= REG_DWT_CTRL__NOTRCPKT | REG_DWT_CTRL__NOCYCCNT | REG_DWT_CTRL__NOPRFCNT;
    }

    // REG_DWT_FUNCTIONn
    if (_HaveDWT()) {
      for (size_t i=0; i<NUM_DWT_COMP; ++i) {
        uint32_t id;
        switch (i) {
          case 0:
            id = (_n.dwtCtrl & REG_DWT_CTRL__NOCYCCNT) ? 0b01010 /* IA DA DA+V */ : 0b01011 /* CC IA DA DA+V */;
            break;
          default:
            id = 0b11110 /* IA IAL DA DAL DV LDV DA+V */;
            break;
        }
        _n.dwtFunction[i] = PUTBITSM(REG_DWT_FUNCTION__ID, id);
      }
    }

    // REG_FP_CTRL
    _n.fpCtrl =
        PUTBITSM(REG_FP_CTRL__REV, 1)
      | PUTBITSM(REG_FP_CTRL__NUM_CODE_LO, GETBITS(NUM_FPB_COMP,0,3))
      | PUTBITSM(REG_FP_CTRL__NUM_CODE_HI, GETBITS(NUM_FPB_COMP,4,6));

    // REG_FPCCR
    _n.fpccrS  = REG_FPCCR__S | REG_FPCCR__LSPEN | REG_FPCCR__ASPEN;
    _n.fpccrNS = REG_FPCCR__ASPEN;

    // REG_VTOR
    _n.vtorS  = _cfg.InitialVtor();
    _n.vtorNS = _cfg.InitialVtor();
  }

  /* NestAccessType {{{4
   * --------------
   */
  enum NestAccessType {
    NAT_SW,         // Load/store from software running on core
    NAT_Internal,   // Core's internal logic is accessing register
    NAT_External,   // JTAG, etc.
  };

  /* _NestCheckRegDWT {{{4
   * ----------------
   */
  bool _NestCheckRegDWT(NestAccessType nat) {
    return _HaveDWT() && (nat != NAT_SW || IMPL_DEF_BASELINE_NO_SW_ACCESS_DWT || _HaveMainExt());
  }

  /* _NestCheckRegFPB {{{4
   * ----------------
   */
  bool _NestCheckRegFPB(NestAccessType nat) {
    return _HaveFPB() && (nat != NAT_SW || IMPL_DEF_BASELINE_NO_SW_ACCESS_FPB || _HaveMainExt());
  }

  /* _NestPrioBits {{{4
   * -------------
   */
  uint8_t _NestPrioBits() {
    uint8_t numBits;

    if (_HaveMainExt()) {
      numBits = _cfg.PriorityBits();
      ASSERT(numBits >= 3);
    } else
      numBits = 2;

    return numBits;
  }

  /* _NestMaskPrio {{{4
   * -------------
   */
  uint8_t _NestMaskPrio(uint8_t m) {
    return m & BITS((8-_NestPrioBits()),7);
  }

  /* _NestAccessClassify {{{4
   * -------------------
   * {targetNS, targetRAZWI, targetFault}
   */
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
    // INTERNAL         ACCESS TO 0xE002_xxxx Non-Secure space            UNREACHABLE

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

  /* _NestLoad32 {{{4
   * -----------
   * Returns nonzero for BusFault.
   */
  int _NestLoad32(phys_t addr, bool isPriv, bool isSecure, uint32_t &v) {
    auto [targetNS, targetRAZWI, targetFault] = _NestAccessClassify(addr, isPriv, isSecure);

    if (targetFault)
      return -1;

    if (targetRAZWI)
      v = 0;
    else if (targetNS)
      v = _NestLoad32Actual(addr |  0x2'0000U, NAT_SW);
    else
      v = _NestLoad32Actual(addr & ~0x2'0000U, NAT_SW);

    return 0;
  }

  /* _NestLoad32Actual {{{4
   * -----------------
   */
  uint32_t _NestLoad32Actual(phys_t addr, NestAccessType nat) {
    phys_t baddr    = addr & ~0x2'0000U;
    bool   isNS     = !!(addr &  0x2'0000U);
    if (isNS)
      ASSERT(_HaveSecurityExt());

    switch (addr) {
      case REG_DWT_CTRL:      return _NestCheckRegDWT(nat) ? _n.dwtCtrl : 0; // Res0

      case REG_DWT_COMP( 0):
      case REG_DWT_COMP( 1):
      case REG_DWT_COMP( 2):
      case REG_DWT_COMP( 3):
      case REG_DWT_COMP( 4):
      case REG_DWT_COMP( 5):
      case REG_DWT_COMP( 6):
      case REG_DWT_COMP( 7):
      case REG_DWT_COMP( 8):
      case REG_DWT_COMP( 9):
      case REG_DWT_COMP(10):
      case REG_DWT_COMP(11):
      case REG_DWT_COMP(12):
      case REG_DWT_COMP(13):
      case REG_DWT_COMP(14):
        return _NestCheckRegDWT(nat) ? _n.dwtComp[(addr - REG_DWT_COMP(0))/16] : 0; // Res0

      case REG_DWT_FUNCTION( 0):
      case REG_DWT_FUNCTION( 1):
      case REG_DWT_FUNCTION( 2):
      case REG_DWT_FUNCTION( 3):
      case REG_DWT_FUNCTION( 4):
      case REG_DWT_FUNCTION( 5):
      case REG_DWT_FUNCTION( 6):
      case REG_DWT_FUNCTION( 7):
      case REG_DWT_FUNCTION( 8):
      case REG_DWT_FUNCTION( 9):
      case REG_DWT_FUNCTION(10):
      case REG_DWT_FUNCTION(11):
      case REG_DWT_FUNCTION(12):
      case REG_DWT_FUNCTION(13):
      case REG_DWT_FUNCTION(14):
        if (_NestCheckRegDWT(nat)) {
          uint32_t n = (addr - REG_DWT_FUNCTION(0))/16;
          uint32_t v = _n.dwtFunction[n];
          if (nat != NAT_Internal)
            _n.dwtFunction[n] &= ~REG_DWT_FUNCTION__MATCHED;
          return v;
        } else
          return 0; // Res0

      case REG_FP_CTRL:       return _NestCheckRegFPB(nat) ? _n.fpCtrl : 0;

      case REG_CPPWR_S:       return _HaveMainExt() ? _n.cppwrS  : 0;
      case REG_CPPWR_NS:      return _HaveMainExt() ? _n.cppwrNS : 0;

      case REG_CFSR_S:        return _HaveMainExt() ? _n.cfsrS   : 0;
      case REG_CFSR_NS:       return _HaveMainExt() ? _n.cfsrNS  : 0;
      case REG_HFSR_S:        return _HaveMainExt() ? _n.hfsrS   : 0;
      case REG_HFSR_NS:       return _HaveMainExt() ? _n.hfsrNS  : 0;
      case REG_DFSR_S:        return _HaveMainExt() || _HaveHaltingDebug() ? _n.dfsrS  : 0;
      case REG_DFSR_NS:       return _HaveMainExt() || _HaveHaltingDebug() ? _n.dfsrNS : 0;

      case REG_MMFAR_S:       return _HaveMainExt() ? _n.mmfarS  : 0;
      case REG_MMFAR_NS:      return _HaveMainExt() ? _n.mmfarNS : 0;
      case REG_BFAR_S:        return _HaveMainExt() ? _n.bfarS   : 0;
      case REG_BFAR_NS:       return _HaveMainExt() ? _n.bfarNS  : 0;

      case REG_SHPR1_S:       return _HaveMainExt() ? _n.shpr1S  : 0;
      case REG_SHPR1_NS:      return _HaveMainExt() ? _n.shpr1NS : 0;
      case REG_SHPR2_S:       return _n.shpr2S;
      case REG_SHPR2_NS:      return _n.shpr2NS;
      case REG_SHPR3_S:       return _n.shpr3S;
      case REG_SHPR3_NS:      return _n.shpr3NS;

      case REG_CCR_S:         return (_n.ccrS  & 0b1110000011100011011) | BIT(0) | BIT(9);
      case REG_CCR_NS:        return (_n.ccrNS & 0b1110000011100011011) | BIT(0) | BIT(9);

      case REG_SCR_S:         return _n.scrS;
      case REG_SCR_NS:        return _n.scrNS;

      case REG_AIRCR_S: {
        uint32_t v = CHGBITSM(_n.aircrS , REG_AIRCR__VECTKEY, 0xFA05);
        return v;
      }
      case REG_AIRCR_NS: {
        uint32_t v = CHGBITSM(_n.aircrNS, REG_AIRCR__VECTKEY, 0xFA05);
        if (_n.aircrS & REG_AIRCR__SYSRESETREQS)
          v &= ~REG_AIRCR__SYSRESETREQ;
        v = CHGBITSM(v, REG_AIRCR__BFHFNMINS, GETBITSM(_n.aircrS, REG_AIRCR__BFHFNMINS));
        return v;
      }

      case REG_CPACR_S:       return _n.cpacrS;
      case REG_CPACR_NS:      return _n.cpacrNS;

      case REG_NSACR:         return _n.nsacr;

      case REG_MPU_TYPE_S:    return PUTBITSM(_NumMpuRegionS(),  REG_MPU_TYPE__DREGION);
      case REG_MPU_TYPE_NS:   return PUTBITSM(_NumMpuRegionNS(), REG_MPU_TYPE__DREGION);

      case REG_MPU_CTRL_S:    return _n.mpuCtrlS;
      case REG_MPU_CTRL_NS:   return _n.mpuCtrlNS;

      case REG_MPU_RNR_S:     return _n.mpuRnrS;
      case REG_MPU_RNR_NS:    return _n.mpuRnrNS;

      case REG_MPU_MAIR0_S:   return _n.mpuMair0S;
      case REG_MPU_MAIR0_NS:  return _n.mpuMair0NS;
      case REG_MPU_MAIR1_S:   return _n.mpuMair1S;
      case REG_MPU_MAIR1_NS:  return _n.mpuMair1NS;

      case REG_MPU_RBAR_S:    return _n.mpuRnrS  < _NumMpuRegionS()  ? _n.mpuRbarS [_n.mpuRnrS ] : 0;
      case REG_MPU_RBAR_A1_S: return _n.mpuRnrS+1< _NumMpuRegionS()  ? _n.mpuRbarS [_n.mpuRnrS+1] : 0;
      case REG_MPU_RBAR_A2_S: return _n.mpuRnrS+2< _NumMpuRegionS()  ? _n.mpuRbarS [_n.mpuRnrS+2] : 0;
      case REG_MPU_RBAR_A3_S: return _n.mpuRnrS+3< _NumMpuRegionS()  ? _n.mpuRbarS [_n.mpuRnrS+3] : 0;
      case REG_MPU_RBAR_NS:   return _n.mpuRnrNS < _NumMpuRegionNS() ? _n.mpuRbarNS[_n.mpuRnrNS ] : 0;
      case REG_MPU_RBAR_A1_NS:return _n.mpuRnrNS+1<_NumMpuRegionNS() ? _n.mpuRbarNS[_n.mpuRnrNS+1] : 0;
      case REG_MPU_RBAR_A2_NS:return _n.mpuRnrNS+2<_NumMpuRegionNS() ? _n.mpuRbarNS[_n.mpuRnrNS+2] : 0;
      case REG_MPU_RBAR_A3_NS:return _n.mpuRnrNS+3<_NumMpuRegionNS() ? _n.mpuRbarNS[_n.mpuRnrNS+3] : 0;

      case REG_MPU_RLAR_S:    return _n.mpuRnrS  < _NumMpuRegionS()  ? _n.mpuRlarS [_n.mpuRnrS ] : 0;
      case REG_MPU_RLAR_A1_S: return _n.mpuRnrS+1< _NumMpuRegionS()  ? _n.mpuRlarS [_n.mpuRnrS+1] : 0;
      case REG_MPU_RLAR_A2_S: return _n.mpuRnrS+2< _NumMpuRegionS()  ? _n.mpuRlarS [_n.mpuRnrS+2] : 0;
      case REG_MPU_RLAR_A3_S: return _n.mpuRnrS+3< _NumMpuRegionS()  ? _n.mpuRlarS [_n.mpuRnrS+3] : 0;
      case REG_MPU_RLAR_NS:   return _n.mpuRnrNS < _NumMpuRegionNS() ? _n.mpuRlarNS[_n.mpuRnrNS] : 0;
      case REG_MPU_RLAR_A1_NS:return _n.mpuRnrNS+1< _NumMpuRegionNS() ? _n.mpuRlarNS[_n.mpuRnrNS+1] : 0;
      case REG_MPU_RLAR_A2_NS:return _n.mpuRnrNS+2< _NumMpuRegionNS() ? _n.mpuRlarNS[_n.mpuRnrNS+2] : 0;
      case REG_MPU_RLAR_A3_NS:return _n.mpuRnrNS+3< _NumMpuRegionNS() ? _n.mpuRlarNS[_n.mpuRnrNS+3] : 0;

      case REG_SAU_CTRL:      return _n.sauCtrl;
      case REG_SAU_TYPE:      return PUTBITSM(_NumSauRegion(), REG_SAU_TYPE__SREGION);
      case REG_SAU_RNR:       return _n.sauRnr;
      case REG_SAU_RBAR:      return _n.sauRnr < _NumSauRegion() ? _n.sauRbar[_n.sauRnr] : 0;
      case REG_SAU_RLAR:      return _n.sauRnr < _NumSauRegion() ? _n.sauRlar[_n.sauRnr] : 0;

      case REG_SFSR_S:        return _HaveMainExt() ? _n.sfsr : 0;
      case REG_SFAR_S:        return _HaveMainExt() ? _n.sfar : 0;

      case REG_VTOR_S:        return _n.vtorS;
      case REG_VTOR_NS:       return _n.vtorNS;

      case REG_DAUTHCTRL:     return (_HaveMainExt() || _HaveHaltingDebug()) && _HaveSecurityExt() ? _n.dauthCtrl : 0;

      case REG_FPCCR_S:
        return _n.fpccrS;

      case REG_FPCCR_NS: {
        uint32_t secureOnlyMask = REG_FPCCR__S | REG_FPCCR__SFRDY | REG_FPCCR__TS | REG_FPCCR__CLRONRETS | REG_FPCCR__LSPENS;
        if (_n.demcr & REG_DEMCR__SDME)
          secureOnlyMask |= REG_FPCCR__MONRDY;
        uint32_t bankedBitsMask = REG_FPCCR__LSPACT | REG_FPCCR__USER | REG_FPCCR__THREAD
          | REG_FPCCR__MMRDY | REG_FPCCR__SPLIMVIOL | REG_FPCCR__UFRDY | REG_FPCCR__ASPEN;
        uint32_t sharedBitsMask = ~bankedBitsMask;

        return ((_n.fpccrNS & bankedBitsMask) | (_n.fpccrS & sharedBitsMask)) & ~secureOnlyMask;
      }

      case REG_FPCAR_S:       return _n.fpcarS;
      case REG_FPCAR_NS:      return _n.fpcarNS;

      case REG_FPDSCR_S:      return _n.fpdscrS;
      case REG_FPDSCR_NS:     return _n.fpdscrNS;

      case REG_ICSR_S:
      case REG_ICSR_NS: {
        uint32_t v = 0;
        // VECTACTIVE
        if (_HaveMainExt() || _HaveHaltingDebug())
          v |= PUTBITSM(REG_ICSR__VECTACTIVE, GETBITSM(_s.xpsr, XPSR__EXCEPTION));
        // RETTOBASE
        if (_HaveMainExt()) {
          int numActive = 0;
          for (int i=0; i<ARRAYLEN(_s.excActive); ++i)
            if (_s.excActive[i])
              ++numActive;
          if (numActive > 1)
            v |= REG_ICSR__RETTOBASE;
        }
        // VECTPENDING
        auto [pendingPrio, pendingExcNo, pendingIsSecure] = _PendingExceptionDetailsActual();
        v |= PUTBITSM(pendingExcNo, REG_ICSR__VECTPENDING);
        // ISRPENDING
        if (_HaveMainExt() || _HaveHaltingDebug())
          for (int i=16; i<NUM_EXC; ++i)
            if (_s.excPending[i]) {
              v |= REG_ICSR__ISRPENDING;
              break;
            }
        // ISRPREEMPT
        if ((_HaveMainExt() || _HaveHaltingDebug()) && pendingExcNo && _ExecutionPriority() > pendingPrio)
          v |= REG_ICSR__ISRPREEMPT;
        // STTNS
        if (!isNS)
          v |= (_n.icsr & REG_ICSR__STTNS);
        // PENDSTSET
        if (    (!isNS || _HaveSysTick() == 2
                 || (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS)))
             && (_s.excPending[SysTick] & BIT((int)isNS)) )
          v |= REG_ICSR__PENDSTSET;
        // PENDSVSET
        if (_s.excPending[PendSV] & BIT((int)isNS))
          v |= REG_ICSR__PENDSVSET;
        // PENDNMISET
        if (_s.excPending[NMI] && (!isNS || !!(_n.aircrS & REG_AIRCR__BFHFNMINS)))
          v |= REG_ICSR__PENDNMISET;
        return v;
      }

      case REG_SHCSR_S:
      case REG_SHCSR_NS: {
        uint32_t v = 0;
        if (_HaveMainExt() && _IsActiveForState(MemManage, !isNS))
          v |= REG_SHCSR__MEMFAULTACT;
        if (_HaveMainExt() && _IsActiveForState(BusFault, !isNS))
          v |= REG_SHCSR__BUSFAULTACT;
        if (_IsActiveForState(HardFault, !isNS))
          v |= REG_SHCSR__HARDFAULTACT;
        if (_HaveMainExt() && _IsActiveForState(UsageFault, !isNS))
          v |= REG_SHCSR__USGFAULTACT;
        if (!isNS && _IsActiveForState(SecureFault, true))
          v |= REG_SHCSR__SECUREFAULTACT;
        if (_IsActiveForState(NMI, !isNS))
          v |= REG_SHCSR__NMIACT;
        if (_IsActiveForState(SVCall, !isNS))
          v |= REG_SHCSR__SVCALLACT;
        if (_IsActiveForState(DebugMonitor, !isNS))
          v |= REG_SHCSR__MONITORACT;
        if (_IsActiveForState(PendSV, !isNS))
          v |= REG_SHCSR__PENDSVACT;
        if (_IsActiveForState(SysTick, !isNS))
          v |= REG_SHCSR__SYSTICKACT;

        if (_HaveMainExt() && _IsPendingForState(UsageFault, !isNS))
          v |= REG_SHCSR__USGFAULTPENDED;
        if (_HaveMainExt() && _IsPendingForState(MemManage, !isNS))
          v |= REG_SHCSR__MEMFAULTPENDED;
        if (_HaveMainExt() && _IsPendingForState(BusFault, !isNS))
          v |= REG_SHCSR__BUSFAULTPENDED;
        if (_IsPendingForState(SVCall, !isNS))
          v |= REG_SHCSR__SVCALLPENDED;

        if (_HaveMainExt() && _IsEnabledForState(MemManage, !isNS))
          v |= REG_SHCSR__MEMFAULTENA;
        if (_HaveMainExt() && _IsEnabledForState(BusFault, !isNS))
          v |= REG_SHCSR__BUSFAULTENA;
        if (_HaveMainExt() && _IsEnabledForState(UsageFault, !isNS))
          v |= REG_SHCSR__USGFAULTENA;
        if (_HaveMainExt() && !isNS && _IsEnabledForState(SecureFault, true))
          v |= REG_SHCSR__SECUREFAULTENA;

        if (_HaveMainExt() && _IsPendingForState(SecureFault, true))
          v |= REG_SHCSR__SECUREFAULTPENDED;

        if (_IsPendingForState(HardFault, !isNS))
          v |= REG_SHCSR__HARDFAULTPENDED;

        return v;
      }

      case REG_DHCSR_S:
      case REG_DHCSR_NS:
        return _n.dhcsr;

      case REG_DEMCR_S:
      case REG_DEMCR_NS:
        return _n.demcr;

      case REG_SYST_CSR_S:
        if (_HaveSysTick()) {
          return _n.systCsrS | PUTBITSM(_SystGetCountFlag(/*ns=*/false, /*clear=*/nat == NAT_SW), REG_SYST_CSR__COUNTFLAG);
        } else
          return 0;
      case REG_SYST_CSR_NS:
        if (_HaveSysTick() == 2)
          return _n.systCsrNS | PUTBITSM(_SystGetCountFlag(/*ns=*/true, /*clear=*/nat == NAT_SW), REG_SYST_CSR__COUNTFLAG);
        else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS))
          return _n.systCsrS | PUTBITSM(_SystGetCountFlag(/*ns=*/false, /*clear=*/nat == NAT_SW), REG_SYST_CSR__COUNTFLAG);
        else
          return 0;
      case REG_SYST_RVR_S:
        return _n.systRvrS;
      case REG_SYST_RVR_NS:
        if (_HaveSysTick() == 2)
          return _n.systRvrNS;
        else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS))
          return _n.systRvrS;
        else
          return 0;
      case REG_SYST_CVR_S:
        return _SystGetCurrent(false);
      case REG_SYST_CVR_NS:
        if (_HaveSysTick() == 2)
          return _SystGetCurrent(true);
        else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS))
          return _SystGetCurrent(false);
        else
          return 0;
      case REG_SYST_CALIB_S:
        if (_HaveSysTick())
          return _n.systCalibS | REG_SYST_CALIB__NOREF;
        else
          return 0;
      case REG_SYST_CALIB_NS:
        if (_HaveSysTick() == 2)
          return _n.systCalibNS | REG_SYST_CALIB__NOREF;
        else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS))
          return _n.systCalibS | REG_SYST_CALIB__NOREF;
        else
          return 0;

      // ----------------------------------------------
      default:
        // REG_NVIC_ISPRn, REG_NVIC_ICPRn
        if ((baddr >= 0xE000'E200 && baddr < 0xE000'E240) || (baddr >= 0xE000'E280 && baddr < 0xE000'E2C0))
          return _NestLoadNvicPendingReg((addr/4)&0xF, /*secure=*/!isNS);

        // REG_NVIC_ISERn, REG_NVIC_ICERn
        if ((baddr >= 0xE000'E100 && baddr < 0xE000'E140) || (baddr >= 0xE000'E180 && baddr < 0xE000'E1C0))
          return _NestLoadNvicEnableReg((addr/4)&0xF, /*secure=*/!isNS);

        // REG_NVIC_IABRn
        if (baddr >= 0xE000'E300 && baddr < 0xE000'E340)
          return _NestLoadNvicActiveReg((addr/4)&0xF, /*secure=*/!isNS);

        // REG_NVIC_ITNSn
        if (addr >= 0xE000E380 && addr < 0xE000E3C0)
          return _n.nvicItns[(addr/4)&0xF];

        // REG_NVIC_IPRn
        if (addr >= 0xE000E400 && addr < 0xE000E5F0)
          return _n.nvicIpr[(addr - 0xE000E400)/4];

        // REG_FP_COMPn
        if ((addr >= 0xE000'2008 && addr < 0xE000'2008 + NUM_FPB_COMP*4) && !(addr % 4))
          return _NestCheckRegFPB(nat) ? _n.fpComp[(addr - 0xE000'2008)/4] : 0;

        // "Privileged accesses to unimplemented registers are Res0."
        // Return 0 for unimplemented registers.
        return 0;
    }
  }

  /* _NestStoreNvicPendingReg {{{4
   * ------------------------
   */
  void _NestStoreNvicPendingReg(uint32_t groupNo, uint32_t v, bool isSecure, bool setNotClear) {
    uint32_t itns   = _n.nvicItns[groupNo];
    int      limit  = (groupNo == 15) ? 15 : 32;
    for (int i=0; i<limit; ++i)
      if (v & BIT(i))
        _SetPending(16 + groupNo*32 + i, isSecure, setNotClear, true);
  }

  /* _NestStoreNvicEnableReg {{{4
   * -----------------------
   */
  void _NestStoreNvicEnableReg(uint32_t groupNo, uint32_t v, bool isSecure, bool setNotClear) {
    uint32_t itns  = _n.nvicItns[groupNo];
    int      limit = (groupNo == 15) ? 15 : 32;
    for (int i=0; i<limit; ++i)
      if (v & BIT(i))
        _SetEnable(16 + groupNo*32 + i, isSecure, setNotClear, true);
  }

  /* _NestLoadNvicPendingReg {{{4
   * -----------------------
   */
  uint32_t _NestLoadNvicPendingReg(uint32_t groupNo, bool isSecure) {
    uint32_t v      = 0;
    uint32_t itns   = _n.nvicItns[groupNo];
    int      limit  = (groupNo == 15) ? 15 : 32;
    for (int i=0; i<limit; ++i)
      if (_IsPendingForState(16+groupNo*32+i, isSecure))
        v |= BIT(i);
    return v;
  }

  /* _NestLoadNvicEnableReg {{{4
   * ----------------------
   */
  uint32_t _NestLoadNvicEnableReg(uint32_t groupNo, bool isSecure) {
    uint32_t v      = 0;
    int      limit  = (groupNo == 15) ? 15 : 32;
    for (int i=0; i<limit; ++i)
      if (_IsEnabledForState(16+groupNo*32+i, isSecure))
        v |= BIT(i);
    return v;
  }

  /* _NestLoadNvicActiveReg {{{4
   * ----------------------
   */
  uint32_t _NestLoadNvicActiveReg(uint32_t groupNo, bool isSecure) {
    uint32_t v      = 0;
    uint32_t itns   = _n.nvicItns[groupNo];
    int      limit  = (groupNo == 15) ? 15 : 32;
    for (int i=0; i<limit; ++i)
      if (_IsActiveForState(16+groupNo*32+i, isSecure))
        v |= BIT(i);
    return v;
  }

  /* _NestStore32 {{{4
   * ------------
   */
  int _NestStore32(phys_t addr, bool isPriv, bool isSecure, uint32_t v) {
    auto [targetNS, targetRAZWI, targetFault] = _NestAccessClassify(addr, isPriv, isSecure);

    if (targetFault)
      return -1;

    if (targetRAZWI)
      return 0;

    if (targetNS)
      _NestStore32Actual(addr |  0x2'0000U, v, NAT_SW);
    else
      _NestStore32Actual(addr & ~0x2'0000U, v, NAT_SW);

    return 0;
  }

  /* _NestStore32Actual {{{4
   * ------------------
   */
  void _NestStore32Actual(phys_t addr, uint32_t v, NestAccessType nat) {
    phys_t baddr    = addr & ~0x2'0000U;
    bool   isNS     = !!(addr &  0x2'0000U);
    if (isNS)
      ASSERT(_HaveSecurityExt());

    switch (addr) {
      case REG_DWT_CTRL:
        // !DWT:  res0
        // !Main: might not be SW accessible
        //
        // 33222222 22221111 11111100 00000000  r: RO. c: 0 on cold reset. 0: Res0.
        // 10987654 32109876 54321098 76543210  m: Res0 if !Main. M: Res1 if !Main. p: Res0 if !PRFCNT. t: Res0 if !TRCPKT/!NOCYCCNT. c: Res0 if !CYCCNT. u: Unknown on cold reset.
        // rrrrr0rr cccccccc 000cuuuu uuuuuuuc
        //     M MM mmpppppp    tcccc cccccccc
        //                      c
        if (_NestCheckRegDWT(nat)) {
          uint32_t roBits = BITS(13,15) | BITS(24,31);
          if (_n.dwtCtrl & REG_DWT_CTRL__NOCYCCNT)
            roBits |= BITS(16,23) | BITS(0,12);
          if (_n.dwtCtrl & REG_DWT_CTRL__NOTRCPKT)
            roBits |= BIT (12);
          if (_n.dwtCtrl & REG_DWT_CTRL__NOPRFCNT)
            roBits |= BITS(16,21);
          v &= ~roBits;
          v |= _n.dwtCtrl & roBits;
          _n.dwtCtrl = v;
        }
        break;

      case REG_DWT_COMP( 0):
      case REG_DWT_COMP( 1):
      case REG_DWT_COMP( 2):
      case REG_DWT_COMP( 3):
      case REG_DWT_COMP( 4):
      case REG_DWT_COMP( 5):
      case REG_DWT_COMP( 6):
      case REG_DWT_COMP( 7):
      case REG_DWT_COMP( 8):
      case REG_DWT_COMP( 9):
      case REG_DWT_COMP(10):
      case REG_DWT_COMP(11):
      case REG_DWT_COMP(12):
      case REG_DWT_COMP(13):
      case REG_DWT_COMP(14):
        if (_NestCheckRegDWT(nat)) {
          uint32_t n      = (addr - REG_DWT_COMP(0))/16;
          if (n < NUM_DWT_COMP) {
            uint32_t roBits = 0;
            if ((GETBITSM(_n.dwtFunction[n], REG_DWT_FUNCTION__MATCH) & 0b1110) == 0b0010)
              roBits |= BIT(0);
            v &= ~roBits;
            _n.dwtComp[n] = v;
          }
        }
        break;

      case REG_DWT_FUNCTION( 0):
      case REG_DWT_FUNCTION( 1):
      case REG_DWT_FUNCTION( 2):
      case REG_DWT_FUNCTION( 3):
      case REG_DWT_FUNCTION( 4):
      case REG_DWT_FUNCTION( 5):
      case REG_DWT_FUNCTION( 6):
      case REG_DWT_FUNCTION( 7):
      case REG_DWT_FUNCTION( 8):
      case REG_DWT_FUNCTION( 9):
      case REG_DWT_FUNCTION(10):
      case REG_DWT_FUNCTION(11):
      case REG_DWT_FUNCTION(12):
      case REG_DWT_FUNCTION(13):
      case REG_DWT_FUNCTION(14):
        if (_NestCheckRegDWT(nat)) {
          uint32_t n      = (addr - REG_DWT_FUNCTION(0))/16;
          if (n < NUM_DWT_COMP) {
            uint32_t roBits = BITS( 6, 9) | BITS(12,31);
            v &= ~roBits;
            v |= _n.dwtFunction[n] & roBits;
            _n.dwtFunction[n] = v;
            if ((GETBITSM(v, REG_DWT_FUNCTION__MATCH) & 0b1110) == 0b0010)
              _n.dwtComp[n] &= ~1;
          }
        }
        break;

      case REG_FP_CTRL:
        if (_NestCheckRegFPB(nat) && (v & BIT(1))) {
          uint32_t roBits = BITS( 1,31);
          v &= ~roBits;
          v |= _n.fpCtrl & roBits;
          _n.fpCtrl = v;
        }
        break;

      case REG_CPPWR_S:
        if (_HaveMainExt())
          // TODO make bits for unsupported coprocessors RAZ/WI
          _n.cppwrS = v & (BITS( 0,15) | BITS(20,23));
        break;

      case REG_CPPWR_NS:
        if (_HaveMainExt())
          _n.cppwrNS = v & (BITS( 0,15) | BITS(20,23));
        break;

      case REG_CFSR_S:
        if (_HaveMainExt()) {
          if (nat == NAT_Internal)
            _n.cfsrS = v;
          else
            _n.cfsrS &= ~v;
        }
        break;

      case REG_CFSR_NS:
        if (_HaveMainExt()) {
          if (nat == NAT_Internal)
            _n.cfsrNS = v;
          else
            _n.cfsrNS &= ~v;
        }
        break;

      case REG_HFSR_S:
        if (_HaveMainExt()) {
          if (nat == NAT_Internal)
            _n.hfsrS = v;
          else
            _n.hfsrS &= ~v;
        }
        break;

      case REG_HFSR_NS:
        if (_HaveMainExt()) {
          if (nat == NAT_Internal)
            _n.hfsrNS = v;
          else
            _n.hfsrNS &= ~v;
        }
        break;

      case REG_DFSR_S:
        if (_HaveMainExt() || _HaveHaltingDebug()) {
          if (nat == NAT_Internal)
            _n.dfsrS = v;
          else
            _n.dfsrS &= ~v;
        }
        break;

      case REG_DFSR_NS:
        if (_HaveMainExt() || _HaveHaltingDebug()) {
          if (nat == NAT_Internal)
            _n.dfsrNS = v;
          else
            _n.dfsrNS &= ~v;
        }
        break;

      case REG_MMFAR_S:
        if (_HaveMainExt())
          _n.mmfarS = v;
        break;

      case REG_MMFAR_NS:
        if (_HaveMainExt())
          _n.mmfarNS = v;
        break;

      case REG_BFAR_S:
        if (_HaveMainExt())
          _n.bfarS = v;
        break;

      case REG_BFAR_NS:
        if (_HaveMainExt())
          _n.bfarNS = v;
        break;

      case REG_SHPR1_S:
        if (_HaveMainExt()) {
          v = CHGBITS(v, 0, 7, _NestMaskPrio(GETBITS(v, 0, 7)));
          v = CHGBITS(v, 8,15, _NestMaskPrio(GETBITS(v, 8,15)));
          v = CHGBITS(v,16,23, _NestMaskPrio(GETBITS(v,16,23)));
          v = CHGBITS(v,24,31, _NestMaskPrio(GETBITS(v,24,31)));
          _n.shpr1S = v;
        }
        break;

      case REG_SHPR1_NS:
        if (_HaveMainExt()) {
          v = CHGBITS(v, 0, 7, _NestMaskPrio(GETBITS(v, 0, 7)));
          v = CHGBITS(v, 8,15, _NestMaskPrio(GETBITS(v, 8,15)));
          v = CHGBITS(v,16,23, _NestMaskPrio(GETBITS(v,16,23)));
          v = CHGBITS(v,24,31, _NestMaskPrio(GETBITS(v,24,31)));
          _n.shpr1NS = v;
        }
        break;

      case REG_SHPR2_S:
        v = CHGBITS(v, 0,23, 0);
        v = CHGBITS(v,24,31, _NestMaskPrio(GETBITS(v,24,31)));
        _n.shpr2S = v;
        break;

      case REG_SHPR2_NS:
        v = CHGBITS(v, 0,23, 0);
        v = CHGBITS(v,24,31, _NestMaskPrio(GETBITS(v,24,31)));
        _n.shpr2NS = v;
        break;

      case REG_SHPR3_S:
        v = CHGBITS(v, 0, 7, _HaveMainExt() ? _NestMaskPrio(GETBITS(v, 0, 7)) : 0); // TODO DEMCR.SDME
        v = CHGBITS(v, 8,15, 0);
        v = CHGBITS(v,16,23, _NestMaskPrio(GETBITS(v,16,23)));
        v = CHGBITS(v,24,31, _NestMaskPrio(GETBITS(v,24,31))); // TODO Type 1 SysTick
        _n.shpr3S = v;
        break;

      case REG_SHPR3_NS:
        v = CHGBITS(v, 0, 7, _HaveMainExt() ? _NestMaskPrio(GETBITS(v, 0, 7)) : 0);
        v = CHGBITS(v, 8,15, 0);
        v = CHGBITS(v,16,23, _NestMaskPrio(GETBITS(v,16,23)));
        v = CHGBITS(v,24,31, _NestMaskPrio(GETBITS(v,24,31)));
        _n.shpr3NS = v;
        break;

      case REG_CCR_S:
        _n.ccrS     = _MaskOrNonMain((v & 0b1110000011100011011) | BIT(0) | BIT(9),
          BITS(16,18)|BIT(10)|BIT( 8)|BIT( 4)|BIT( 1), BIT( 3)); break;

      case REG_CCR_NS:
        _n.ccrNS    = _MaskOrNonMain((v & 0b1110000011100011011) | BIT(0) | BIT(9),
          BITS(16,18)|BIT(10)|BIT( 8)|BIT( 4)|BIT( 1), BIT( 3)); break;

      case REG_SCR_S:
        _n.scrS = v & BITS(1,4);
        break;

      case REG_SCR_NS:
        _n.scrNS = v & BITS(1,4);
        break;

      case REG_AIRCR_S:
        if (GETBITSM(v, REG_AIRCR__VECTKEY) == 0x05FA) {
          uint32_t roMask = BIT(15);
          if (!_HaveMainExt())
            roMask |= BITS( 8,10);
          v = CHGBITSM(v, REG_AIRCR__VECTKEY, 0);
          v &= ~roMask;
          v |= _n.aircrS;
          if (v & REG_AIRCR__SYSRESETREQ)
            _SetPending(Reset, true, true);
          v &= ~REG_AIRCR__SYSRESETREQ;
          if (v & REG_AIRCR__VECTCLRACTIVE)
            ; // TODO
          v &= ~REG_AIRCR__VECTCLRACTIVE;
          _n.aircrS = v;
        }
        break;

      case REG_AIRCR_NS:
        if (GETBITSM(v, REG_AIRCR__VECTKEY) == 0x05FA) {
          // TODO VECTCLRACTIVE
          uint32_t roMask = BIT(15) | BIT(14) | BIT(13) | BIT( 1);
          if (!_HaveMainExt())
            roMask |= BITS( 8,10);
          if (_n.aircrS & REG_AIRCR__SYSRESETREQS)
            roMask |= REG_AIRCR__SYSRESETREQ;
          v = CHGBITSM(v, REG_AIRCR__VECTKEY, 0);
          v &= ~roMask;
          v |= _n.aircrNS;
          if (v & REG_AIRCR__SYSRESETREQ)
            _SetPending(Reset, true, true);
          v &= ~REG_AIRCR__SYSRESETREQ;
          if (v & REG_AIRCR__VECTCLRACTIVE)
            ; // TODO
          v &= ~REG_AIRCR__VECTCLRACTIVE;
          _n.aircrNS = v;
        }
        break;

      case REG_CPACR_S:
        _n.cpacrS = v & (BITS(0,15) | BITS(20,23)); // TODO RAZ/WI for unimplemented coprocessors
        break;

      case REG_CPACR_NS:
        _n.cpacrNS = v & (BITS(0,15) | BITS(20,23));
        break;

      case REG_NSACR:
        _n.nsacr = v & (BITS( 0, 7) | BITS(10,11));
        break;

      case REG_MPU_TYPE_S:
      case REG_MPU_TYPE_NS:
        break;

      case REG_MPU_CTRL_S:
        _n.mpuCtrlS = v & BITS(0,2);
        break;

      case REG_MPU_CTRL_NS:
        _n.mpuCtrlNS = v & BITS(0,2);
        break;

      case REG_MPU_RNR_S:
        _n.mpuRnrS = v & BITS( 0, 7); // TODO CONSTRAINED UNPREDICTABLE
        break;

      case REG_MPU_RNR_NS:
        _n.mpuRnrNS = v & BITS( 0, 7); // TODO CONSTRAINED UNPREDICTABLE
        break;

      case REG_MPU_MAIR0_S:
        _n.mpuMair0S = v;
        break;

      case REG_MPU_MAIR0_NS:
        _n.mpuMair0NS = v;
        break;

      case REG_MPU_MAIR1_S:
        _n.mpuMair1S = v;
        break;

      case REG_MPU_MAIR1_NS:
        _n.mpuMair1NS = v;
        break;

      case REG_MPU_RBAR_S:
        if (_n.mpuRnrS < _NumMpuRegionS())
          _n.mpuRbarS[_n.mpuRnrS] = v;
        break;

      case REG_MPU_RBAR_A1_S:
        if (_n.mpuRnrS+1 < _NumMpuRegionS())
          _n.mpuRbarS[_n.mpuRnrS+1] = v;
        break;

      case REG_MPU_RBAR_A2_S:
        if (_n.mpuRnrS+2 < _NumMpuRegionS())
          _n.mpuRbarS[_n.mpuRnrS+2] = v;
        break;

      case REG_MPU_RBAR_A3_S:
        if (_n.mpuRnrS+3 < _NumMpuRegionS())
          _n.mpuRbarS[_n.mpuRnrS+3] = v;
        break;

      case REG_MPU_RBAR_NS:
        if (_n.mpuRnrNS < _NumMpuRegionNS())
          _n.mpuRbarNS[_n.mpuRnrNS] = v;
        break;

      case REG_MPU_RBAR_A1_NS:
        if (_n.mpuRnrNS+1 < _NumMpuRegionNS())
          _n.mpuRbarNS[_n.mpuRnrNS+1] = v;
        break;

      case REG_MPU_RBAR_A2_NS:
        if (_n.mpuRnrNS+2 < _NumMpuRegionNS())
          _n.mpuRbarNS[_n.mpuRnrNS+2] = v;
        break;

      case REG_MPU_RBAR_A3_NS:
        if (_n.mpuRnrNS+3 < _NumMpuRegionNS())
          _n.mpuRbarNS[_n.mpuRnrNS+3] = v;
        break;

      case REG_MPU_RLAR_S:
        if (_n.mpuRnrS < _NumMpuRegionS())
          _n.mpuRlarS[_n.mpuRnrS] = v;
        break;

      case REG_MPU_RLAR_A1_S:
        if (_n.mpuRnrS+1 < _NumMpuRegionS())
          _n.mpuRlarS[_n.mpuRnrS+1] = v;
        break;

      case REG_MPU_RLAR_A2_S:
        if (_n.mpuRnrS+2 < _NumMpuRegionS())
          _n.mpuRlarS[_n.mpuRnrS+2] = v;
        break;

      case REG_MPU_RLAR_A3_S:
        if (_n.mpuRnrS+3 < _NumMpuRegionS())
          _n.mpuRlarS[_n.mpuRnrS+3] = v;
        break;

      case REG_MPU_RLAR_NS:
        if (_n.mpuRnrNS < _NumMpuRegionNS())
          _n.mpuRlarNS[_n.mpuRnrNS] = v;
        break;

      case REG_MPU_RLAR_A1_NS:
        if (_n.mpuRnrNS+1 < _NumMpuRegionNS())
          _n.mpuRlarNS[_n.mpuRnrNS+1] = v;
        break;

      case REG_MPU_RLAR_A2_NS:
        if (_n.mpuRnrNS+2 < _NumMpuRegionNS())
          _n.mpuRlarNS[_n.mpuRnrNS+2] = v;
        break;

      case REG_MPU_RLAR_A3_NS:
        if (_n.mpuRnrNS+3 < _NumMpuRegionNS())
          _n.mpuRlarNS[_n.mpuRnrNS+3] = v;
        break;

      case REG_SAU_CTRL:
        _n.sauCtrl = v & BITS( 0, 1);
        break;

      case REG_SAU_TYPE:
        break;

      case REG_SAU_RNR:
        if (_NumSauRegion())
          _n.sauRnr = v & REG_SAU_RNR__REGION;
        // TODO CONSTRAINED UNPREDICTABLE
        break;

      case REG_SAU_RBAR:
        if (_n.sauRnr < _NumSauRegion())
          _n.sauRbar[_n.sauRnr] = v & ~BITS( 0, 4);
        break;

      case REG_SAU_RLAR:
        if (_n.sauRnr < _NumSauRegion())
          _n.sauRlar[_n.sauRnr] = v & ~BITS( 2, 4);
        break;

      case REG_SFSR_S:
        if (_HaveMainExt()) {
          if (nat == NAT_Internal)
            _n.sfsr = v;
          else
            _n.sfsr &= ~v;
        }
        break;

      case REG_SFAR_S:
        if (_HaveMainExt())
          _n.sfar = v;
        break;

      case REG_VTOR_S:    _n.vtorS    = v & BITS( 7,31); break;
      case REG_VTOR_NS:   _n.vtorNS   = v & BITS( 7,31); break;

      case REG_DAUTHCTRL:
        if ((_HaveMainExt() || _HaveHaltingDebug()) && _HaveSecurityExt())
          _n.dauthCtrl = v & BITS( 0, 3);
        break;

      case REG_FPCCR_S:
        _n.fpccrS   = v & (BITS( 0, 10) | BITS(26,31));
        break;

      case REG_FPCCR_NS: {
        uint32_t secureOnlyMask = REG_FPCCR__S | REG_FPCCR__SFRDY | REG_FPCCR__TS | REG_FPCCR__CLRONRETS | REG_FPCCR__LSPENS;
        if (_n.demcr & REG_DEMCR__SDME)
          secureOnlyMask |= REG_FPCCR__MONRDY;
        uint32_t bankedBitsMask = REG_FPCCR__LSPACT | REG_FPCCR__USER | REG_FPCCR__THREAD
          | REG_FPCCR__MMRDY | REG_FPCCR__SPLIMVIOL | REG_FPCCR__UFRDY | REG_FPCCR__ASPEN;
        uint32_t sharedBitsMask = ~bankedBitsMask;
        uint32_t roMask = BITS(11,25);

        _n.fpccrS  = (_n.fpccrS & ~(sharedBitsMask & ~secureOnlyMask)) | (v & (sharedBitsMask & ~secureOnlyMask));
        _n.fpccrNS = v & bankedBitsMask;
      } break;


      case REG_FPCAR_S:
        if (_HaveFPExt())
          _n.fpcarS = v & ~BITS( 0, 2);
        break;

      case REG_FPCAR_NS:
        if (_HaveFPExt())
          _n.fpcarNS = v & ~BITS( 0, 2);
        break;

      case REG_FPDSCR_S:  if (_HaveFPExt()) _n.fpdscrS  = v & BITS(22,26); break;
      case REG_FPDSCR_NS: if (_HaveFPExt()) _n.fpdscrNS = v & BITS(22,26); break;

      case REG_ICSR_S:
      case REG_ICSR_NS:
        // PENDSTSET, PENDSTCLR
        if (v & (REG_ICSR__PENDSTSET | REG_ICSR__PENDSTCLR))
           if (   (_HaveSysTick() == 2)
               || (_HaveSysTick() == 1 && (!(_n.icsr & REG_ICSR__STTNS)) == !isNS))
             _SetPending(SysTick, !isNS, !!(v & REG_ICSR__PENDSTSET));
        // PENDSVCLR
        // PENDSVSET
        if (v & (REG_ICSR__PENDSVSET | REG_ICSR__PENDSVCLR))
          _SetPending(PendSV, !isNS, !!(v & REG_ICSR__PENDSVSET), true);
        // PENDNMICLR
        // PENDNMISET
        if (v & (REG_ICSR__PENDNMISET | REG_ICSR__PENDNMICLR))
          if (!isNS || !!(_n.aircrS & REG_AIRCR__BFHFNMINS))
            _SetPending(NMI, true, !!(v & REG_ICSR__PENDNMISET), true);
        if (!isNS) {
          uint32_t rwMask = 0;
          if (_HaveSysTick() == 1)
            rwMask |= REG_ICSR__STTNS;
          _n.icsr = v & rwMask;
        }
        break;

      case REG_SHCSR_S:
      case REG_SHCSR_NS:
        // ----- Actives
        if (_HaveMainExt()) {
          _SetActive(MemManage, !isNS, !!(v & REG_SHCSR__MEMFAULTACT));
          _SetActive(BusFault, !isNS, !!(v & REG_SHCSR__BUSFAULTACT));
        }
        if (!(v & REG_SHCSR__HARDFAULTACT))
          _SetActive(HardFault, !isNS, false);
        if (_HaveMainExt())
          _SetActive(UsageFault, !isNS, !!(v & REG_SHCSR__USGFAULTACT));
        if (_HaveMainExt() && _HaveSecurityExt())
          _SetActive(SecureFault, !isNS, !!(v & REG_SHCSR__SECUREFAULTACT));
        if (!(v & REG_SHCSR__NMIACT))
          // TODO: NMIACT: "This field ignores writes if the value being written
          // is one, AIRCR.BFHFNMINS is zero, the access is from Non-secure state,
          // the access is not via the NS alias, or the access is from a debugger
          // when DHCSR.S_SDE is zero. This bit can only be cleared by access
          // from the Secure state to the NS alias."
          _SetActive(NMI, !isNS, false);
        _SetActive(SVCall, !isNS, !!(v & REG_SHCSR__SVCALLACT));
        if (_HaveMainExt())
          _SetActive(DebugMonitor, !isNS, !!(v & REG_SHCSR__MONITORACT));
        _SetActive(PendSV, !isNS, !!(v & REG_SHCSR__PENDSVACT));
        _SetActive(SysTick, !isNS, !!(v & REG_SHCSR__SYSTICKACT));

        // ----- Pendings
        if (_HaveMainExt()) {
          _SetPending(UsageFault, !isNS, !!(v & REG_SHCSR__USGFAULTPENDED), true);
          _SetPending(MemManage, !isNS, !!(v & REG_SHCSR__MEMFAULTPENDED), true);
          _SetPending(BusFault, !isNS, !!(v & REG_SHCSR__BUSFAULTPENDED), true);
          _SetPending(SVCall, !isNS, !!(v & REG_SHCSR__SVCALLPENDED), true);
          _SetPending(HardFault, !isNS, !!(v & REG_SHCSR__HARDFAULTPENDED), true);
          if (_HaveSecurityExt())
            _SetPending(SecureFault, !isNS, !!(v & REG_SHCSR__SECUREFAULTPENDED), true);
        }

        // ----- Enables
        if (_HaveMainExt()) {
          _SetEnable(MemManage, !isNS, !!(v & REG_SHCSR__MEMFAULTENA), true);
          _SetEnable(BusFault, !isNS, !!(v & REG_SHCSR__BUSFAULTENA), true);
          _SetEnable(UsageFault, !isNS, !!(v & REG_SHCSR__USGFAULTENA), true);
          if (_HaveSecurityExt())
            _SetEnable(SecureFault, !isNS, !!(v & REG_SHCSR__SECUREFAULTENA), true);
        }
        break;

      case REG_DHCSR_S:
      case REG_DHCSR_NS:
        if (nat == NAT_Internal)
          _n.dhcsr = v;
        else if (GETBITS(v,16,31) == 0xA05F) {
          uint32_t rwBits = BITS(0,3) | BIT(5);
          if (nat != NAT_External)
            rwBits &= ~BIT(0);
          _n.dhcsr = (_n.dhcsr & BITS(16,31)) | (v & rwBits);
        }
        break;

      case REG_DEMCR_S:
      case REG_DEMCR_NS:
        if (nat == NAT_Internal)
          _n.demcr = v;
        else {
          uint32_t roBits = REG_DEMCR__SDME | REG_DEMCR__MON_PEND | REG_DEMCR__MON_EN
            | BITS( 1, 3) | BITS(12,15) | BITS(21,23) | BITS(25,31);
          if (!_HaveMainExt())
            roBits |= REG_DEMCR__MON_REQ | REG_DEMCR__MON_STEP | REG_DEMCR__VC_SFERR | REG_DEMCR__VC_INTERR
              | REG_DEMCR__VC_BUSERR | REG_DEMCR__VC_STATERR | REG_DEMCR__VC_CHKERR | REG_DEMCR__VC_NOCPERR
              | REG_DEMCR__VC_MMERR;
          if (!_HaveSecurityExt() || !_HaveHaltingDebug())
            roBits |= REG_DEMCR__VC_SFERR;
          if (!_HaveHaltingDebug())
            roBits |= REG_DEMCR__VC_HARDERR | REG_DEMCR__VC_INTERR | REG_DEMCR__VC_BUSERR | REG_DEMCR__VC_STATERR
              | REG_DEMCR__VC_CHKERR | REG_DEMCR__VC_NOCPERR | REG_DEMCR__VC_MMERR | REG_DEMCR__VC_CORERESET;
          if (_HaveMainExt()) {
            _SetPending(DebugMonitor, !isNS, !!(v & REG_DEMCR__MON_PEND));
            _SetEnable (DebugMonitor, !isNS, !!(v & REG_DEMCR__MON_EN));
          }
          _n.demcr = v;
        }
        break;

      case REG_SYST_CSR_S:
        if (_HaveSysTick()) {
          if (!_SystCalcFreq(false))
            v |= REG_SYST_CSR__CLKSOURCE;
          _n.systCsrS = v & BITS(0,2);
          _SystUpdate(/*ns=*/false, /*clearCount=*/false);
        }
        break;
      case REG_SYST_CSR_NS:
        if (_HaveSysTick() == 2) {
          if (!_SystCalcFreq(false))
            v |= REG_SYST_CSR__CLKSOURCE;
          _n.systCsrNS = v & BITS(0,2);
          _SystUpdate(/*ns=*/true, /*clearCount=*/false);
        } else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS)) {
          if (!_SystCalcFreq(false))
            v |= REG_SYST_CSR__CLKSOURCE;
          _n.systCsrS = v & BITS(0,2);
          _SystUpdate(/*ns=*/false, /*clearCount=*/false);
        }
        break;

      case REG_SYST_RVR_S:
        if (_HaveSysTick()) {
          _n.systRvrS = v & BITS(0,23);
          _SystUpdate(/*ns=*/false, /*clearCount=*/false);
        }
        break;
      case REG_SYST_RVR_NS:
        if (_HaveSysTick() == 2) {
          _n.systRvrNS = v & BITS(0,23);
          _SystUpdate(/*ns=*/true, /*clearCount=*/false);
        } else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS)) {
          _n.systRvrS = v & BITS(0,23);
          _SystUpdate(/*ns=*/false, /*clearCount=*/false);
        }
        break;

      case REG_SYST_CVR_S:
        if (_HaveSysTick())
          _SystUpdate(/*ns=*/false, /*clearCount=*/true);
        break;
      case REG_SYST_CVR_NS:
        if (_HaveSysTick() == 2)
          _SystUpdate(/*ns=*/true, /*clearCount=*/true);
        else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS))
          _SystUpdate(/*ns=*/false, /*clearCount=*/true);
        break;

      case REG_SYST_CALIB_S:
        if (_HaveSysTick())
          _n.systCalibS = v & (BITS(0,23) | BIT(30));
        break;
      case REG_SYST_CALIB_NS:
        if (_HaveSysTick() == 2)
          _n.systCalibNS = v & (BITS(0,23) | BIT(30));
        else if (_HaveSysTick() == 1 && !!(_n.icsr & REG_ICSR__STTNS))
          _n.systCalibNS = v & (BITS(0,23) | BIT(30));
        break;

      // ----------------------------------------------

      default:
        // REG_FP_COMPn
        if ((addr >= 0xE000'2008 && addr < 0xE000'2008 + NUM_FPB_COMP*4) && !(addr % 4)) {
          if (!_NestCheckRegFPB(nat))
            return;

          _n.fpComp[(addr - 0xE000'2008)/4] = v;
          return;
        }

        // REG_NVIC_ICPRn
        if (baddr >= 0xE000'E280 && baddr < 0xE000'E2C0) {
          _NestStoreNvicPendingReg((baddr/4)&0xF, v, /*secure=*/!isNS, /*setNotClear=*/false);
          return;
        }

        // REG_NVIC_ISPRn
        if (baddr >= 0xE000'E200 && baddr < 0xE000'E240) {
          _NestStoreNvicPendingReg((baddr/4)&0xF, v, /*secure=*/!isNS, /*setNotClear=*/true);
          return;
        }

        // REG_NVIC_ICERn
        if (baddr >= 0xE000'E180 && baddr < 0xE000'E1C0) {
          _NestStoreNvicEnableReg((baddr/4)&0xF, v, /*isSecure=*/!isNS, /*setNotClear=*/false);
          return;
        }

        // REG_NVIC_ISERn
        if (baddr >= 0xE000'E100 && baddr < 0xE000'E140) {
          _NestStoreNvicEnableReg((baddr/4)&0xF, v, /*isSecure=*/!isNS, /*setNotClear=*/true);
          return;
        }

        // REG_NVIC_ITNSn
        if (addr >= 0xE000E380 && addr < 0xE000E3C0) {
          uint32_t n    = (addr/4) & 0xF;
          uint32_t loEx = n*32 + 16;
          uint32_t hiEx = loEx + 32;
          uint32_t mask;
          if (loEx >= NUM_EXC)
            mask = 0;
          else {
            if (hiEx >= NUM_EXC)
              hiEx = NUM_EXC-1;
            mask = BITS(0, hiEx-loEx-1);
          }

          _n.nvicItns[n] = v & mask;
          return;
        }

        // REG_NVIC_IPRn
        if (addr >= 0xE000E400 && addr < 0xE000E5F0) {
          uint32_t n = (addr/4) & 0xF;
          uint32_t loEx = n*4 + 16;
          uint32_t mask = 0;
          if (loEx < NUM_EXC)
            mask |= BITS(0, 7);
          if (loEx+1 < NUM_EXC)
            mask |= BITS(8,15);
          if (loEx+2 < NUM_EXC)
            mask |= BITS(16,23);
          if (loEx+3 < NUM_EXC)
            mask |= BITS(24,31);

          _n.nvicIpr[n] = v & mask;
          return;
        }

        printf("Unsupported nest store 0x%08x <- 0x%08x\n", addr, v);
        abort();
    }
  }

  /* _SystResolve {{{4
   * ------------
   */
  SysTickDevice &_SystResolve(bool ns) {
    assert(_HaveSysTick());
    assert(!ns || _HaveSysTick() == 2);
    return ns ? _sysTickNS : _sysTickS;
  }

  /* _SystCalcFreq {{{4
   * -------------
   */
  uint64_t _SystCalcFreq(bool clkSource) {
    return clkSource ? _cfg.SystIntFreq() : _cfg.SystExtFreq();
  }

  /* _SystGetCountFlag {{{4
   * -----------------
   * Get, and optionally clear, the count flag for a SysTick timer.
   */
  bool _SystGetCountFlag(bool ns, bool clear) {
    return _SystResolve(ns).SysTickGetCountFlag(clear);
  }

  /* _SystGetIntrFlag {{{4
   * ----------------
   * Get, and optionally clear, the interrupt flag for a SysTick timer.
   */
  bool _SystGetIntrFlag(bool ns, bool clear) {
    return _SystResolve(ns).SysTickGetIntrFlag(clear);
  }

  /* _SystGetCurrent {{{4
   * ---------------
   */
  uint32_t _SystGetCurrent(bool ns) {
    return _SystResolve(ns).SysTickGetCurrent();
  }

  /* _SystUpdate {{{4
   * -----------
   */
  void _SystUpdate(bool ns, bool clearCount) {
    uint32_t reloadValue = ns ? _n.systRvrNS : _n.systRvrS;
    uint32_t csr         = ns ? _n.systCsrNS : _n.systCsrS;

    bool enable     = !!(csr & REG_SYST_CSR__ENABLE);
    bool tickInt    = !!(csr & REG_SYST_CSR__TICKINT);
    bool clkSource  = !!(csr & REG_SYST_CSR__CLKSOURCE);

    uint64_t freq = _SystCalcFreq(clkSource);

    auto &st = _SystResolve(ns);
    st.SysTickSetConfig(enable, tickInt, freq, reloadValue, clearCount ? 0 : -1);
  }

  /* LocalMonitor {{{3
   * ============
   * Implements an ARMv8-M local monitor for a specific PE.
   */
  struct LocalMonitor {
    LocalMonitor(bool checkAddresses=true) :_checkAddresses(checkAddresses) {}

    void MarkExclusive(phys_t addr, uint32_t size) {
      _s.addr = addr;
      _s.size = size;
    }

    bool IsExclusive(phys_t addr, uint32_t size) {
      return _s.size && (!_checkAddresses || _s.ContainsAll(addr, size));
    }

    void ClearExclusive() {
      _s.size = 0;
    }

  private:
    MonitorState  _s{};
    bool          _checkAddresses;
  };

  /* Architectural Support Functions {{{3
   * ===============================
   */

  /* _IsSEE {{{4
   * ------
   */
  static bool _IsSEE(const Exception &e) { return e.GetType() == ExceptionType::SEE; }

  /* _IsUNDEFINED {{{4
   * ------------
   */
  static bool _IsUNDEFINED(const Exception &e) { return e.GetType() == ExceptionType::UNDEFINED; }

  /* _IsExceptionTaken {{{4
   * -----------------
   */
  static bool _IsExceptionTaken(const Exception &e) { return e.GetType() == ExceptionType::EndOfInstruction; }

  /* _MaskOrNonMain {{{4
   * --------------
   */
  uint32_t _MaskOrNonMain(uint32_t x, uint32_t maskBits, uint32_t orBits) {
    if (_HaveMainExt())
      return x;
    return (x & ~maskBits) | orBits;
  }

  /* InternalLoad32 {{{4
   * --------------
   */
  uint32_t InternalLoad32(phys_t addr) {
    ASSERT(addr >= 0xE000'0000);
    return _NestLoad32Actual(addr, NAT_Internal);
  }

  /* InternalStore32 {{{4
   * ---------------
   */
  void InternalStore32(phys_t addr, uint32_t v) {
    ASSERT(addr >= 0xE000'0000);
    _NestStore32Actual(addr, v, NAT_Internal);
  }

  /* InternalOr32 {{{4
   * ------------
   */
  void InternalOr32(phys_t addr, uint32_t x) {
    InternalStore32(addr, InternalLoad32(addr) | x);
  }

  /* InternalMask32 {{{4
   * --------------
   */
  void InternalMask32(phys_t addr, uint32_t x) {
    InternalStore32(addr, InternalLoad32(addr) & ~x);
  }

  /* _InternalLoadMpuSecureRegion {{{4
   * ----------------------------
   */
  std::tuple<uint32_t,uint32_t> _InternalLoadMpuSecureRegion(size_t idx) {
    printf("Bus internal load MPU secure region %zu\n", idx);
    if (idx >= _NumMpuRegionS())
      return {0,0}; // {RBAR,RLAR}

    return {_n.mpuRbarS[idx], _n.mpuRlarS[idx]};
  }

  /* _InternalLoadMpuNonSecureRegion {{{4
   * -------------------------------
   */
  std::tuple<uint32_t,uint32_t> _InternalLoadMpuNonSecureRegion(size_t idx) {
    printf("Bus internal load MPU non-secure region %zu\n", idx);
    if (idx >= _NumMpuRegionNS())
      return {0,0}; // {RBAR,RLAR}

    return {_n.mpuRbarNS[idx], _n.mpuRlarNS[idx]};
  }

  /* _InternalLoadSauRegion {{{4
   * ----------------------
   */
  std::tuple<uint32_t,uint32_t> _InternalLoadSauRegion(size_t idx) {
    printf("Bus internal load SAU region %zu\n", idx);
    if (idx >= _NumSauRegion())
      return {0,0}; // {RBAR,RLAR}

    return {_n.sauRbar[idx], _n.sauRlar[idx]};
  }

  /* _ThisInstrAddr {{{4
   * --------------
   */
  uint32_t _ThisInstrAddr() { return _s.pc; }

  /* _ThisInstr {{{4
   * ----------
   * Returns a 32-bit value which contains the bitstring encoding of the
   * current instruction. In the case of 16-bit instructions, the instruction
   * is packed into the bottom 16 bits with upper 16 bits zeroed. In the case
   * of 32-bit instructions, the instruction is treated as two halfwords, with
   * the first halfword of the instruction in the top 16 bits and the second
   * halfword in the bottom 16 bits.
   */
  uint32_t _ThisInstr() { return _s.thisInstr; }

  /* _IsSecure {{{4
   * ---------
   */
  bool _IsSecure() { return _HaveSecurityExt() && _s.curState == SecurityState_Secure; }

  /* _HaveMainExt {{{4
   * ------------
   */
  bool _HaveMainExt() { return _cfg.HaveMainExt(); }

  /* _HaveSecurityExt {{{4
   * ----------------
   */
  bool _HaveSecurityExt() { return _cfg.HaveSecurityExt(); }

  /* _SetThisInstrDetails {{{4
   * --------------------
   */
  void _SetThisInstrDetails(uint32_t opcode, int len, uint32_t defaultCond) {
    _s.thisInstr            = opcode;
    _s.thisInstrLength      = len;
    _s.thisInstrDefaultCond = defaultCond;
    _s.curCondOverride      = -1;
  }

  /* _VFPSmallRegisterBank {{{4
   * ---------------------
   */
  bool _VFPSmallRegisterBank() { return false; }

  /* _HaveDebugMonitor {{{4
   * -----------------
   */
  bool _HaveDebugMonitor() { return _HaveMainExt(); }

  /* _MaxExceptionNum {{{4
   * ----------------
   */
  int _MaxExceptionNum() { return _HaveMainExt() ? 511 : 47; }

  /* _GetD {{{4
   * -----
   */
  uint64_t _GetD(int n) {
    assert(n >= 0 && n <= 31);
    assert(!(n >= 16 && _VFPSmallRegisterBank())); // UNDEFINED
    return _s.d[n];
  }

  /* _SetD {{{4
   * -----
   */
  void _SetD(int n, uint64_t value) {
    assert(n >= 0 && n <= 31);
    assert(!(n >= 16 && _VFPSmallRegisterBank())); // UNDEFINED
    _s.d[n] = value;
  }

  /* _GetS {{{4
   * -----
   */
  uint32_t _GetS(int n) {
    assert(n >= 0 && n <= 31);
    if (!(n%2))
      return (uint32_t)_GetD(n/2);
    else
      return (uint32_t)(_GetD(n/2)>>32);
  }

  /* _SetS {{{4
   * -----
   */
  void _SetS(int n, uint32_t value) {
    assert(n >= 0 && n <= 31);
    if (!(n%2))
      _SetD(n/2, (_GetD(n/2) & ~UINT64_C(0xFFFF'FFFF)) | uint64_t(value));
    else
      _SetD(n/2, (_GetD(n/2) & ~UINT64_C(0xFFFF'FFFF'0000'0000)) | (uint64_t(value)<<32));
  }

  /* _ClearExclusiveLocal {{{4
   * --------------------
   */
  void _ClearExclusiveLocal(int procID) {
    _lm.ClearExclusive();
  }

  /* _ProcessorID {{{4
   * ------------
   */
  int _ProcessorID() { return _procID; }

  /* _SetEventRegister {{{4
   * -----------------
   */
  void _SetEventRegister() { _s.event = true; }

  /* _ClearEventRegister {{{4
   * -------------------
   */
  void _ClearEventRegister() { _s.event = false; }

  /* _EventRegistered {{{4
   * ----------------
   */
  bool _EventRegistered() { return _s.event; }

  /* _SendEvent {{{4
   * ----------
   */
  void _SendEvent() {
    // XXX: This is supposed to set the event register of every PE in a multiprocessor system.
    // The manual does not state whether this should include this PE or not. A literal reading
    // says yes, so we do so.
    _SetEventRegister();
  }

  /* _InstructionSynchronizationBarrier {{{4
   * ----------------------------------
   */
  void _InstructionSynchronizationBarrier(uint8_t option) {
    // See _DataMemoryBarrier.
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }

  /* _DataSynchronizationBarrier {{{4
   * ---------------------------
   */
  void _DataSynchronizationBarrier(uint8_t option) {
    // See _DataMemoryBarrier.
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }

  /* _DataMemoryBarrier {{{4
   * ------------------
   */
  void _DataMemoryBarrier(uint8_t option) {
    // Emulated program has requested a data memory barrier. Though it's very
    // unlikely with the complexity of the simulator that we will need to
    // perform such a barrier on the host it's not conceptually impossible, so
    // we execute a barrier just in case.
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }

  /* _HaveFPB {{{4
   * --------
   */
  bool _HaveFPB() { return _cfg.HaveFPB(); }

  /* _FPB_BreakpointMatch {{{4
   * --------------------
   */
  void _FPB_BreakpointMatch() {
    _GenerateDebugEventResponse();
  }

  /* _DefaultExcInfo {{{4
   * ---------------
   */
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

  /* _HaveDWT {{{4
   * --------
   */
  bool _HaveDWT() { return _cfg.HaveDWT(); }

  /* _HaveITM {{{4
   * --------
   */
  bool _HaveITM() { return _cfg.HaveITM(); }

  /* _HaveFPExt {{{4
   * ----------
   */
  bool _HaveFPExt() { return _cfg.HaveFPExt(); }

  /* _NoninvasiveDebugAllowed {{{4
   * ------------------------
   */
  bool _NoninvasiveDebugAllowed() {
    return _ExternalNoninvasiveDebugEnabled() || _HaltingDebugAllowed();
  }

  /* _SecureNoninvasiveDebugAllowed {{{4
   * ------------------------------
   */
  bool _SecureNoninvasiveDebugAllowed() {
    if (!_NoninvasiveDebugAllowed())
      return false;

    if (GETBITSM(InternalLoad32(REG_DHCSR), REG_DHCSR__S_SDE))
      return true;

    if (GETBITSM(InternalLoad32(REG_DAUTHCTRL), REG_DAUTHCTRL__SPNIDENSEL))
      return !!GETBITSM(InternalLoad32(REG_DAUTHCTRL), REG_DAUTHCTRL__INTSPNIDEN);

    return _ExternalSecureNoninvasiveDebugEnabled();
  }

  /* _HaltingDebugAllowed {{{4
   * --------------------
   */
  bool _HaltingDebugAllowed() {
    return _ExternalInvasiveDebugEnabled() || GETBITSM(InternalLoad32(REG_DHCSR), REG_DHCSR__S_HALT);
  }

  /* _ExternalInvasiveDebugEnabled {{{4
   * -----------------------------
   */
  bool _ExternalInvasiveDebugEnabled() {
    return !!(_dev.DebugPins() & DEBUG_PIN__DBGEN);
  }

  /* _ExternalNoninvasiveDebugEnabled {{{4
   * --------------------------------
   */
  bool _ExternalNoninvasiveDebugEnabled() {
    return _ExternalInvasiveDebugEnabled() || !!(_dev.DebugPins() & DEBUG_PIN__NIDEN);
  }

  /* _IsDWTEnabled {{{4
   * -------------
   */
  bool _IsDWTEnabled() {
    return _HaveDWT() && GETBITSM(InternalLoad32(REG_DEMCR), REG_DEMCR__TRCENA) && _NoninvasiveDebugAllowed();
  }

  /* _SecureHaltingDebugAllowed {{{4
   * --------------------------
   */
  bool _SecureHaltingDebugAllowed() {
    if (!_HaltingDebugAllowed())
      return false;
    else if (InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__SPIDENSEL)
      return !!(InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__INTSPIDEN);
    else
      return _ExternalSecureInvasiveDebugEnabled();
  }

  /* _ExternalSecureInvasiveDebugEnabled {{{4
   * -----------------------------------
   */
  bool _ExternalSecureInvasiveDebugEnabled() {
    return _ExternalInvasiveDebugEnabled() && !!(_dev.DebugPins() & DEBUG_PIN__SPIDEN);
  }

  /* _ExternalSecureNoninvasiveDebugEnabled {{{4
   * --------------------------------------
   */
  bool _ExternalSecureNoninvasiveDebugEnabled() {
    return _ExternalNoninvasiveDebugEnabled() && !!(_dev.DebugPins() & (DEBUG_PIN__SPIDEN | DEBUG_PIN__SPNIDEN));
  }

  /* _NumMpuRegionS {{{4
   * --------------
   */
  uint8_t _NumMpuRegionS() { return _cfg.NumMpuRegionS(); }

  /* _NumMpuRegionNS {{{4
   * ---------------
   */
  uint8_t _NumMpuRegionNS() { return _cfg.NumMpuRegionNS(); }

  /* _NumSauRegion {{{4
   * -------------
   */
  uint8_t _NumSauRegion() { return _cfg.NumSauRegion(); }

  /* _CurrentCond {{{4
   * ------------
   */
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

  /* _SecureDebugMonitorAllowed {{{4
   * --------------------------
   */
  bool _SecureDebugMonitorAllowed() {
    if (InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__SPIDENSEL)
      return !!(InternalLoad32(REG_DAUTHCTRL) & REG_DAUTHCTRL__INTSPIDEN);
    else
      return _ExternalSecureSelfHostedDebugEnabled();
  }

  /* _ExternalSecureSelfHostedDebugEnabled {{{4
   * -------------------------------------
   */
  bool _ExternalSecureSelfHostedDebugEnabled() {
    uint32_t debugPins = _dev.DebugPins();
    return !!(debugPins & DEBUG_PIN__DBGEN) && !!(debugPins & DEBUG_PIN__SPIDEN);
  }

  /* _ResetSCSRegs {{{4
   * -------------
   */
  void _ResetSCSRegs() {
    _NestReset();
  }

  /* _IsCPEnabled {{{4
   * ------------
   */
  std::tuple<bool, bool> _IsCPEnabled(int cp) {
    return _IsCPEnabled(cp, _CurrentModeIsPrivileged(), _IsSecure());
  }

  /* _CurrentModeIsPrivileged {{{4
   * ------------------------
   */
  bool _CurrentModeIsPrivileged() {
    return _CurrentModeIsPrivileged(_IsSecure());
  }
  bool _CurrentModeIsPrivileged(bool isSecure) {
    bool npriv = isSecure ? GETBITSM(_s.controlS, CONTROL__NPRIV) : GETBITSM(_s.controlNS, CONTROL__NPRIV);
    return (_CurrentMode() == PEMode_Handler || !npriv);
  }

  /* _CurrentMode {{{4
   * ------------
   */
  PEMode _CurrentMode() {
    return GETBITSM(_s.xpsr, XPSR__EXCEPTION) == NoFault ? PEMode_Thread : PEMode_Handler;
  }

  /* _ConditionPassed {{{4
   * ----------------
   */
  bool _ConditionPassed() {
    return _ConditionHolds(_CurrentCond());
  }

  /* _GetPC {{{4
   * ------
   */
  uint32_t _GetPC() {
    return _GetR(15);
  }

  /* _ThrowUnaligned {{{4
   * ---------------
   * Custom function not corresponding to the ISA manual. Throws unaligned
   * usage fault. For use implementing UNPREDICTABLE where a permitted
   * implementation is to raise an UNALIGNED UsageFault.
   */
  void _ThrowUnaligned() {
    InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNALIGNED);
    auto excInfo = _CreateException(UsageFault, false, false/*UNKNOWN*/);
    _HandleException(excInfo);
  }

  /* _ZeroExtend {{{4
   * -----------
   * Exposition only
   */
  static uint32_t _ZeroExtend(uint32_t v, uint32_t w) {
    return v;
  }

  /* _Align {{{4
   * ------
   */
  static uint32_t _Align(uint32_t x, uint32_t align) {
    return x & ~(align-1);
  }

  /* _BranchWritePC {{{4
   * --------------
   */
  void _BranchWritePC(uint32_t address) {
    _BranchTo(address & ~BIT(0));
  }

  /* _ALUWritePC {{{4
   * -----------
   */
  void _ALUWritePC(uint32_t address) {
    _BranchWritePC(address);
  }

  /* _InITBlock {{{4
   * ----------
   */
  bool _InITBlock() {
    return !!(_GetITSTATE() & BITS(0,3));
  }

  /* _LastInITBlock {{{4
   * --------------
   */
  bool _LastInITBlock() {
    return GETBITS(_GetITSTATE(),0,3) == 0b1000;
  }

  /* _LSL_C {{{4
   * ------
   */
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

  /* _LSR_C {{{4
   * ------
   */
  static std::tuple<uint32_t, bool> _LSR_C(uint32_t x, int shift) {
    ASSERT(shift > 0);

    uint32_t result   = x>>shift;
    bool     carryOut = !!(x & BIT(shift-1));

    return {result, carryOut};
  }

  /* _ASR_C {{{4
   * ------
   */
  static std::tuple<uint32_t, bool> _ASR_C(uint32_t x, int shift) {
    ASSERT(shift > 0);

    int32_t xs = (int32_t)x;

    uint32_t result   = (uint32_t)(xs>>shift);
    bool     carryOut = !!(x & BIT(shift-1));

    return {result, carryOut};
  }

  /* _LSL {{{4
   * ----
   */
  static uint32_t _LSL(uint32_t x, int shift) {
    ASSERT(shift >= 0);
    if (!shift)
      return x;

    auto [result, _] = _LSL_C(x, shift);
    return result;
  }

  /* _LSR {{{4
   * ----
   */
  static uint32_t _LSR(uint32_t x, int shift) {
    ASSERT(shift >= 0);
    if (!shift)
      return x;

    auto [result, _] = _LSR_C(x, shift);
    return result;
  }

  /* _ROR_C {{{4
   * ------
   */
  static std::tuple<uint32_t, bool> _ROR_C(uint32_t x, int shift) {
    ASSERT(shift);

    uint32_t m        = shift % 32;
    uint32_t result   = _LSR(x, m) | _LSL(x, 32-m);
    bool     carryOut = !!(result & BIT(31));

    return {result, carryOut};
  }

  /* _ROR {{{4
   * ----
   */
  static uint32_t _ROR(uint32_t x, int shift) {
    if (!shift)
      return x;

    auto [result, _] = _ROR_C(x, shift);
    return result;
  }

  /* _RRX_C {{{4
   * ------
   */
  static std::tuple<uint32_t, bool> _RRX_C(uint32_t x, bool carryIn) {
    uint32_t result   = (carryIn ? BIT(31) : 0) | (x>>1);
    bool    carryOut  = !!(x & BIT(0));

    return {result, carryOut};
  }

  /* _Shift_C {{{4
   * --------
   */
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

  /* _IsZero {{{4
   * -------
   */
  static bool _IsZero(uint32_t x) {
    return !x;
  }

  /* _IsZeroBit {{{4
   * ----------
   */
  static bool _IsZeroBit(uint32_t x) {
    return _IsZero(x);
  }

  /* _LookUpRName {{{4
   * ------------
   */
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

  /* _BranchToNS {{{4
   * -----------
   */
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

  /* _FunctionReturn {{{4
   * ---------------
   */
  ExcInfo _FunctionReturn() {
    auto exc = _DefaultExcInfo();

    // Pull the return address and IPSR off the Secure stack
    PEMode    mode      = _CurrentMode();
    RName     spName    = _LookUpSP_with_security_mode(true, mode);
    uint32_t  framePtr  = _GetSP(spName);

    if (!_IsAligned(framePtr, 8))
      THROW_UNPREDICTABLE();

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
        InternalOr32(REG_CFSR, REG_CFSR__UFSR__INVPC);

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

  /* _BXWritePC {{{4
   * ----------
   */
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

  /* _BLXWritePC {{{4
   * -----------
   */
  void _BLXWritePC(uint32_t addr, bool allowNonSecure) {
    // If in the Secure state and transitions to the Non-secure state are
    // allowed then the target state is specified by the LSB of the target
    // address.
    if (_HaveSecurityExt() && allowNonSecure && _IsSecure())
      _BranchToNS(addr);
    else {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__T, GETBIT(addr, 0));
      // If EPSR.T == 0 then an exception is taken on the next instruction:
      // UsageFault('Invalid State') if the Main Extension is implemented;
      // HardFault otherwise
      _BranchTo(addr & ~BIT(0));
    }
  }

  /* _LoadWritePC {{{4
   * ------------
   */
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

  /* _GetPRIMASK {{{4
   * -----------
   */
  uint32_t _GetPRIMASK() {
    return _IsSecure() ? _s.primaskS : _s.primaskNS;
  }

  /* _SetPRIMASK {{{4
   * -----------
   */
  void _SetPRIMASK(uint32_t v) {
    if (_IsSecure())
      _s.primaskS = v;
    else
      _s.primaskNS = v;
  }

  /* _GetFAULTMASK {{{4
   * -------------
   */
  uint32_t _GetFAULTMASK() {
    return _IsSecure() ? _s.faultmaskS : _s.faultmaskNS;
  }

  /* _SetFAULTMASK {{{4
   * -------------
   */
  void _SetFAULTMASK(uint32_t v) {
    if (_IsSecure())
      _s.faultmaskS = v;
    else
      _s.faultmaskNS = v;
  }

  /* _AddWithCarry {{{4
   * -------------
   */
  std::tuple<uint32_t, bool, bool> _AddWithCarry(uint32_t x, uint32_t y, bool carryIn) {
    uint32_t unsignedSum;
    int32_t  signedSum;
    bool     carryOut, overflow;

    carryOut  = __builtin_add_overflow(x, y, &unsignedSum);
    carryOut |= __builtin_add_overflow(unsignedSum, (uint32_t)carryIn, &unsignedSum);

    overflow  = __builtin_add_overflow((int32_t)x, (int32_t)y, &signedSum);
    overflow |= __builtin_add_overflow(signedSum, (int32_t)carryIn, &signedSum);

    return {unsignedSum, carryOut, overflow};
  }

  /* _SignExtend {{{4
   * -----------
   * Corresponds to SignExtend(), but has an additional parameter to represent input width.
   */
  uint32_t _SignExtend(uint32_t x, uint32_t inWidth, uint32_t outWidth) {
    if (x & BIT(inWidth-1)) {
      return x | BITS(inWidth,outWidth-1);
    } else
      return x;
  }

  /* _BitCount {{{4
   * ---------
   */
  static uint32_t _BitCount(uint32_t x) {
    return __builtin_popcount(x);
  }

  /* _T32ExpandImm_C {{{4
   * ---------------
   */
  static std::tuple<uint32_t, bool> _T32ExpandImm_C(uint32_t imm12, bool carryIn) {
    if (GETBITS(imm12,10,11) == 0b00) {
      uint32_t imm32;
      switch (GETBITS(imm12, 8, 9)) {
        case 0b00:
          imm32 = _ZeroExtend(GETBITS(imm12, 0, 7), 32);
          break;
        case 0b01:
          if (GETBITS(imm12, 0, 7) == 0)
            THROW_UNPREDICTABLE();
          imm32 = (GETBITS(imm12, 0, 7)<<16) | GETBITS(imm12, 0, 7);
          break;
        case 0b10:
          if (GETBITS(imm12, 0, 7) == 0)
            THROW_UNPREDICTABLE();
          imm32 = (GETBITS(imm12, 0, 7)<<24) | (GETBITS(imm12, 0, 7)<<8);
          break;
        case 0b11:
          if (GETBITS(imm12, 0, 7) == 0)
            THROW_UNPREDICTABLE();
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

  /* _T32ExpandImm {{{4
   * -------------
   */
  uint32_t _T32ExpandImm(uint32_t imm12) {
    auto [imm32, _] = _T32ExpandImm_C(imm12, GETBITSM(_s.xpsr, XPSR__C));
    return imm32;
  }

  /* _Shift {{{4
   * ------
   */
  uint32_t _Shift(uint32_t value, SRType srType, int amount, bool carryIn) {
    auto [result, _] = _Shift_C(value, srType, amount, carryIn);
    return result;
  }

  /* _DecodeImmShift {{{4
   * ---------------
   */
  std::tuple<SRType, int> _DecodeImmShift(uint32_t srType, uint32_t imm5) {
    switch (srType) {
      case 0b00:
        return {SRType_LSL, imm5};
      case 0b01:
        return {SRType_LSR, imm5 ? imm5 : 32};
      case 0b10:
        return {SRType_ASR, imm5 ? imm5 : 32};
      case 0b11:
        if (!imm5)
          return {SRType_RRX, 1};
        else
          return {SRType_ROR, imm5};
      default:
        abort();
    }
  }

  /* _SetITSTATEAndCommit {{{4
   * --------------------
   */
  void _SetITSTATEAndCommit(uint8_t it) {
    _s.nextInstrITState = it;
    _s.itStateChanged = true;
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__IT_ICI_LO, it>>2);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__IT_ICI_HI, it&3);
  }

  /* _HaveSysTick {{{4
   * ------------
   */
  int _HaveSysTick() { return _cfg.SysTick(); }

  /* _NextInstrAddr {{{4
   * --------------
   */
  uint32_t _NextInstrAddr() {
    if (_s.pcChanged)
      return _s.nextInstrAddr;
    else
      return _ThisInstrAddr() + _ThisInstrLength();
  }

  /* _ThisInstrLength {{{4
   * ----------------
   */
  int _ThisInstrLength() { return _s.thisInstrLength; }

  /* _CalcDescriptorFlags {{{4
   * --------------------
   */
  static inline uint32_t _CalcDescriptorFlags(AddressDescriptor memAddrDesc) {
    uint32_t flags = 0;

    if (memAddrDesc.accAttrs.isWrite)
      flags |= LS_FLAG__WRITE;
    if (memAddrDesc.accAttrs.isPriv)
      flags |= LS_FLAG__PRIV;
    flags |= PUTBITSM(memAddrDesc.accAttrs.accType, LS_FLAG__ATYPE__MASK);
    if (memAddrDesc.memAttrs.memType == MemType_Device)
      flags |= LS_FLAG__DEVICE;
    flags |= PUTBITSM(memAddrDesc.memAttrs.device, LS_FLAG__DEVTYPE__MASK);
    flags |= PUTBITSM(memAddrDesc.memAttrs.innerAttrs, LS_FLAG__IATTR__MASK);
    flags |= PUTBITSM(memAddrDesc.memAttrs.outerAttrs, LS_FLAG__OATTR__MASK);
    flags |= PUTBITSM(memAddrDesc.memAttrs.innerHints, LS_FLAG__IHINT__MASK);
    flags |= PUTBITSM(memAddrDesc.memAttrs.outerHints, LS_FLAG__OHINT__MASK);
    if (memAddrDesc.memAttrs.ns)
      flags |= LS_FLAG__NS;
    if (memAddrDesc.memAttrs.innerTransient)
      flags |= LS_FLAG__ITRANSIENT;
    if (memAddrDesc.memAttrs.outerTransient)
      flags |= LS_FLAG__OTRANSIENT;
    if (memAddrDesc.memAttrs.shareable)
      flags |= LS_FLAG__SHAREABLE;
    if (memAddrDesc.memAttrs.outerShareable)
      flags |= LS_FLAG__OSHAREABLE;

    return flags;
  }

  /* _Load {{{4
   * -----
   */
  int _Load(AddressDescriptor memAddrDesc, int size, uint32_t &v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000) {
      if (size != 4)
        // Non-32 bit accesses to SCS are UNPREDICTABLE; generate BusFault.
        return 1;

      return _NestLoad32(memAddrDesc.physAddr, memAddrDesc.accAttrs.isPriv, !memAddrDesc.memAttrs.ns, v);
    }

    return _dev.Load(memAddrDesc.physAddr, size, _CalcDescriptorFlags(memAddrDesc), v);
  }

  /* _Store {{{4
   * ------
   */
  int _Store(AddressDescriptor memAddrDesc, int size, uint32_t v) {
    if (memAddrDesc.physAddr >= 0xE000'0000 && memAddrDesc.physAddr < 0xE010'0000) {
      if (size != 4)
        // Non-32 bit accesses to SCS are UNPREDICTABLE; generate BusFault.
        return 1;

      return _NestStore32(memAddrDesc.physAddr, memAddrDesc.accAttrs.isPriv, !memAddrDesc.memAttrs.ns, v);
    }

    return _dev.Store(memAddrDesc.physAddr, size, _CalcDescriptorFlags(memAddrDesc), v);
  }

  /* _GetMem {{{4
   * -------
   */
  std::tuple<bool,uint32_t> _GetMem(AddressDescriptor memAddrDesc, int size) {
    uint32_t v;
    if (_Load(memAddrDesc, size, v))
      return {true, 0};
    else
      return {false, v};
  }

  /* _SetMem {{{4
   * -------
   */
  bool _SetMem(AddressDescriptor memAddrDesc, int size, uint32_t v) {
    return !!_Store(memAddrDesc, size, MaskBySize(v, size));
  }

  /* _HaveHaltingDebug {{{4
   * -----------------
   */
  bool _HaveHaltingDebug() { return _cfg.HaveHaltingDebug(); }

  /* _CanHaltOnEvent {{{4
   * ---------------
   */
  bool _CanHaltOnEvent(bool isSecure) {
    if (!_HaveSecurityExt())
      assert(!isSecure);
    return _HaveHaltingDebug() && _HaltingDebugAllowed() && !!(InternalLoad32(REG_DHCSR) & REG_DHCSR__C_DEBUGEN)
      && !(InternalLoad32(REG_DHCSR) & REG_DHCSR__S_HALT) && (!isSecure || !!(InternalLoad32(REG_DHCSR) & REG_DHCSR__S_SDE));
  }

  /* _CanPendMonitorOnEvent {{{4
   * ----------------------
   */
  bool _CanPendMonitorOnEvent(bool isSecure, bool checkPri) {
    if (!_HaveSecurityExt())
      assert(!isSecure);
    return _HaveDebugMonitor() && !_CanHaltOnEvent(isSecure)
      && !!(InternalLoad32(REG_DEMCR) & REG_DEMCR__MON_EN)
      && !(InternalLoad32(REG_DHCSR) & REG_DHCSR__S_HALT)
      && (!isSecure || !!(InternalLoad32(REG_DEMCR) & REG_DEMCR__SDME))
      && (!checkPri || _ExceptionPriority(DebugMonitor, isSecure, true) < _ExecutionPriority());
  }

  /* _ThisInstrITState {{{4
   * -----------------
   */
  uint8_t _ThisInstrITState() {
    if (_HaveMainExt())
      return (GETBITSM(_s.xpsr, XPSR__IT_ICI_LO)<<2) | GETBITSM(_s.xpsr, XPSR__IT_ICI_HI);
    else
      return 0;
  }

  /* _GetITSTATE {{{4
   * -----------
   */
  uint8_t _GetITSTATE() { return _ThisInstrITState(); }

  /* _SetITSTATE {{{4
   * -----------
   */
  void _SetITSTATE(uint8_t value) {
    _s.nextInstrITState = value;
    _s.itStateChanged = true;
  }

  /* _GetSP {{{4
   * ------
   */
  uint32_t _GetSP(RName spreg) {
    assert(    spreg == RNameSP_Main_NonSecure
           || (spreg == RNameSP_Main_Secure       && _HaveSecurityExt())
           ||  spreg == RNameSP_Process_NonSecure
           || (spreg == RNameSP_Process_Secure    && _HaveSecurityExt()));
    return _s.r[spreg] & ~3;
  }

  /* _SetSP {{{4
   * ------
   */
  ExcInfo _SetSP(RName spreg, bool excEntry, uint32_t value) {
    ExcInfo excInfo = _DefaultExcInfo();
    auto [limit, applyLimit] = _LookUpSPLim(spreg);
    if (applyLimit && value < limit) {
      if (excEntry)
        _s.r[spreg] = limit;

      if (_HaveMainExt())
        InternalOr32(REG_CFSR, REG_CFSR__UFSR__STKOF);

      excInfo = _CreateException(UsageFault, false, false/*UNKNOWN*/);
      if (!excEntry)
        _HandleException(excInfo);
    } else
      _s.r[spreg] = value & ~3;

    TRACE("SP(%u) <- 0x%x\n", spreg, _s.r[spreg]);
    return excInfo;
  }

  /* _Stack {{{4
   * ------
   */
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

  /* _GetLR {{{4
   * ------
   */
  uint32_t _GetLR() { return _GetR(14); }

  /* _SetLR {{{4
   * ------
   */
  void _SetLR(uint32_t v) { _SetR(14, v); }

  /* _HaveDSPExt {{{4
   * -----------
   */
  bool _HaveDSPExt() { return _cfg.HaveDSPExt(); }

  /* _GetR {{{4
   * -----
   */
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

  /* _SetR {{{4
   * -----
   */
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

  /* _LookUpSP_with_security_mode {{{4
   * ----------------------------
   */
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

  /* _LookUpSP {{{4
   * ---------
   */
  RName _LookUpSP() {
    return _LookUpSP_with_security_mode(_IsSecure(), _CurrentMode());
  }

  /* _LookUpSPLim {{{4
   * ------------
   */
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

  /* _IsReqExcPriNeg {{{4
   * ---------------
   */
  bool _IsReqExcPriNeg(bool secure) {
    bool neg = _IsActiveForState(NMI, secure) || _IsActiveForState(HardFault, secure);
    if (_HaveMainExt()) {
      uint32_t faultMask = secure ? _s.faultmaskS : _s.faultmaskNS;
      if (faultMask & 1)
        neg = true;
    }
    return neg;
  }

  /* _GetSP {{{4
   * ------
   */
  uint32_t _GetSP() { return _GetR(13); }

  /* _SetSP {{{4
   * ------
   */
  void _SetSP(uint32_t value) { _SetRSPCheck(13, value); }

  /* _GetSP_Main {{{4
   * -----------
   */
  uint32_t _GetSP_Main() { return _IsSecure() ? _GetSP_Main_Secure() : _GetSP_Main_NonSecure(); }

  /* _SetSP_Main {{{4
   * -----------
   */
  void _SetSP_Main(uint32_t value) {
    if (_IsSecure())
      _SetSP_Main_Secure(value);
    else
      _SetSP_Main_NonSecure(value);
  }

  /* _GetSP_Main_NonSecure {{{4
   * ---------------------
   */
  uint32_t _GetSP_Main_NonSecure() {
    return _GetSP(RNameSP_Main_NonSecure);
  }

  /* _SetSP_Main_NonSecure {{{4
   * ---------------------
   */
  void _SetSP_Main_NonSecure(uint32_t value) {
    _SetSP(RNameSP_Main_NonSecure, false, value);
  }

  /* _SetSP_Main_Secure {{{4
   * ------------------
   */
  void _SetSP_Main_Secure(uint32_t value) {
    _SetSP(RNameSP_Main_Secure, false, value);
  }

  /* _GetSP_Main_Secure {{{4
   * ------------------
   */
  uint32_t _GetSP_Main_Secure() {
    return _GetSP(RNameSP_Main_Secure);
  }

  /* _GetSP_Process {{{4
   * --------------
   */
  uint32_t _GetSP_Process() {
    return _IsSecure() ? _GetSP_Process_Secure() : _GetSP_Process_NonSecure();
  }

  /* _SetSP_Process {{{4
   * --------------
   */
  void _SetSP_Process(uint32_t value) {
    if (_IsSecure())
      _SetSP_Process_Secure(value);
    else
      _SetSP_Process_NonSecure(value);
  }

  /* _GetSP_Process_NonSecure {{{4
   * ------------------------
   */
  uint32_t _GetSP_Process_NonSecure() {
    return _GetSP(RNameSP_Process_NonSecure);
  }

  /* _SetSP_Process_NonSecure {{{4
   * ------------------------
   */
  void _SetSP_Process_NonSecure(uint32_t value) {
    _SetSP(RNameSP_Process_NonSecure, false, value);
  }

  /* _GetSP_Process_Secure {{{4
   * ---------------------
   */
  uint32_t _GetSP_Process_Secure() {
    return _GetSP(RNameSP_Process_Secure);
  }

  /* _SetSP_Process_Secure {{{4
   * ---------------------
   */
  void _SetSP_Process_Secure(uint32_t value) {
    _SetSP(RNameSP_Process_Secure, false, value);
  }

  /* _SetRSPCheck {{{4
   * ------------
   */
  void _SetRSPCheck(int n, uint32_t v) {
    if (n == 13)
      _SetSP(_LookUpSP(), false, v);
    else
      _SetR(n, v);
  }

  /* _Lockup {{{4
   * -------
   */
  void _Lockup(bool termInst) {
    InternalOr32(REG_DHCSR, REG_DHCSR__S_LOCKUP);
    _BranchToAndCommit(0xEFFF'FFFE);
    if (termInst)
      _EndOfInstruction();
  }

  /* _BranchToAndCommit {{{4
   * ------------------
   */
  void _BranchToAndCommit(uint32_t addr) {
    _s.r[RName_PC]    = addr & ~1;
    _s.pcChanged      = true;
    _s.nextInstrAddr  = addr & ~1;
    _s.pendingReturnOperation = false;
  }

  /* _BranchTo {{{4
   * ---------
   */
  void _BranchTo(uint32_t addr) {
    _s.nextInstrAddr = addr;
    _s.pcChanged     = true;
    _s.pendingReturnOperation = false;
  }

  /* _PendReturnOperation {{{4
   * --------------------
   */
  void _PendReturnOperation(uint32_t retValue) {
    _s.nextInstrAddr          = retValue;
    _s.pcChanged              = true;
    _s.pendingReturnOperation = true;
  }

  /* _IsActiveForState {{{4
   * -----------------
   */
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

  /* _IsPendingForState {{{4
   * ------------------
   * XXX
   */
  bool _IsPendingForState(int exc, bool isSecure) {
    if (!_HaveSecurityExt())
      isSecure = false;

    bool pending;
    if (_IsExceptionTargetConfigurable(exc))
      pending = (_s.excPending[exc] && _ExceptionTargetsSecure(exc, isSecure) == isSecure);
    else {
      int idx = isSecure ? 0 : 1;
      pending = !!(_s.excPending[exc] & BIT(idx));
    }

    return pending;
  }

  /* _IsEnabledForState {{{4
   * ------------------
   * XXX
   */
  bool _IsEnabledForState(int exc, bool isSecure) {
    if (!_HaveSecurityExt())
      isSecure = false;

    bool enabled;
    if (_IsExceptionTargetConfigurable(exc))
      enabled = (_s.excEnable[exc] && _ExceptionTargetsSecure(exc, isSecure) == isSecure);
    else {
      int idx = isSecure ? 0 : 1;
      enabled = !!(_s.excEnable[exc] & BIT(idx));
    }

    return enabled;
  }

  /* _IsExceptionTargetConfigurable {{{4
   * ------------------------------
   */
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

  /* _GetVector {{{4
   * ----------
   */
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

  /* _ValidateAddress {{{4
   * ----------------
   */
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
      InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNALIGNED);
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

  /* _MemO {{{4
   * -----
   */
  uint32_t _MemO(uint32_t addr, int size) {
    auto [excInfo, value] = _MemA_with_priv_security(addr, size, AccType_ORDERED, _FindPriv(), _IsSecure(), true);
    _HandleException(excInfo);
    return value;
  }

  void _MemO(uint32_t addr, int size, uint32_t value) {
    auto excInfo = _MemA_with_priv_security(addr, size, AccType_ORDERED, _FindPriv(), _IsSecure(), true, value);
    _HandleException(excInfo);
  }

  /* _MemU {{{4
   * -----
   */
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

  /* _MemU_unpriv {{{4
   * ------------
   */
  uint32_t _MemU_unpriv(uint32_t addr, int size) {
    return _MemU_with_priv(addr, size, false);
  }

  void _MemU_unpriv(uint32_t addr, int size, uint32_t value) {
    _MemU_with_priv(addr, size, false, value);
  }

  /* _MemU_with_priv {{{4
   * ---------------
   */
  uint32_t _MemU_with_priv(uint32_t addr, int size, bool priv) {
    uint32_t value;

    // Do aligned access, take alignment fault, or do sequence of bytes
    if (addr == _Align(addr, size)) {
      value = _MemA_with_priv(addr, size, priv, true);
    } else if (InternalLoad32(REG_CCR) & REG_CCR__UNALIGN_TRP) {
      InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNALIGNED);
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
    // Do aligned access, take alignment fault, or do sequence of bytes
    if (addr == _Align(addr, size))
      _MemA_with_priv(addr, size, priv, true, value);
    else if (InternalLoad32(REG_CCR) & REG_CCR__UNALIGN_TRP) {
      InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNALIGNED);
      auto excInfo = _CreateException(UsageFault, false, UNKNOWN_VAL(false));
      _HandleException(excInfo);
    } else { // if unaligned access
      // PPB (0xE0000000 to 0xE010000) is always little endian
      if ((InternalLoad32(REG_AIRCR) & REG_AIRCR__ENDIANNESS) && GETBITS(addr,20,31) != 0xE00)
        value = _BigEndianReverse(value, size);
      for (int i=0; i<size; ++i)
        _MemA_with_priv(addr+i, 1, priv, false, GETBITS(value, 8*i, 8*i+7));
    }
  }

  /* _MemA {{{4
   * -----
   */
  uint32_t _MemA(uint32_t addr, int size) {
    return _MemA_with_priv(addr, size, _FindPriv(), true);
  }

  void _MemA(uint32_t addr, int size, uint32_t value) {
    _MemA_with_priv(addr, size, _FindPriv(), true, value);
  }

  /* _MemA_with_priv {{{4
   * ---------------
   */
  uint32_t _MemA_with_priv(uint32_t addr, int size, bool priv, bool aligned) {
    auto [excInfo, value] = _MemA_with_priv_security(addr, size, AccType_NORMAL, priv, _IsSecure(), aligned);
    _HandleException(excInfo);
    return value;
  }

  void _MemA_with_priv(uint32_t addr, int size, bool priv, bool aligned, uint32_t value) {
    auto excInfo = _MemA_with_priv_security(addr, size, AccType_NORMAL, priv, _IsSecure(), aligned, value);
    _HandleException(excInfo);
  }

  /* _MemA_with_priv_security {{{4
   * ------------------------
   */
  std::tuple<ExcInfo, uint32_t> _MemA_with_priv_security(uint32_t addr, int size, AccType accType, bool priv, bool secure, bool aligned) {
    ExcInfo excInfo = _DefaultExcInfo();
    if (!_IsAligned(addr, size)) {
      if (_HaveMainExt())
        InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNALIGNED);
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
            InternalOr32(REG_CFSR, REG_CFSR__BFSR__UNSTKERR);
          else if (accType == AccType_NORMAL || accType == AccType_ORDERED) {
            uint32_t bfar = InternalLoad32(REG_BFAR);
            bfar = CHGBITSM(bfar, REG_BFAR__ADDRESS, addr);
            InternalStore32(REG_BFAR, bfar);
            InternalOr32(REG_CFSR, REG_CFSR__BFSR__BFARVALID | REG_CFSR__BFSR__PRECISERR);
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
        InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNALIGNED);
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
            InternalOr32(REG_CFSR, REG_CFSR__BFSR__STKERR);
          else if (accType == AccType_LAZYFP)
            InternalOr32(REG_CFSR, REG_CFSR__BFSR__LSPERR);
          else if (accType == AccType_NORMAL || accType == AccType_ORDERED) {
            InternalStore32(REG_BFAR, addr);
            InternalOr32(REG_CFSR, REG_CFSR__BFSR__BFARVALID | REG_CFSR__BFSR__PRECISERR);
          }
        }

        if (!negativePri || !(InternalLoad32(REG_CCR) & REG_CCR__BFHFNMIGN))
          excInfo = _CreateException(BusFault, false, false/*UNKNOWN*/);
      }
    }

    return excInfo;
  }

  /* _ClearExclusiveByAddress {{{4
   * ------------------------
   */
  void _ClearExclusiveByAddress(uint32_t addr, int exclProcID, int size) {
    auto lk = _gm.Lock();
    _gm.ClearExclusiveByAddress(addr, exclProcID, size);
  }

  /* _IsAligned {{{4
   * ----------
   */
  bool _IsAligned(uint32_t addr, int size) {
    assert(size == 1 || size == 2 || size == 4 || size == 8);
    uint32_t mask = (size-1);
    return !(addr & mask);
  }

  /* _MPUCheck {{{4
   * ---------
   */
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
        THROW_UNPREDICTABLE();
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
          std::tie(rbar,rlar) = _InternalLoadMpuSecureRegion(r);
        else
          std::tie(rbar,rlar) = _InternalLoadMpuNonSecureRegion(r);

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

  /* _MAIRDecode {{{4
   * -----------
   */
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
        THROW_UNPREDICTABLE();
    } else {
      unpackInner = true;
      memAttrs.memType        = MemType_Normal;
      memAttrs.device         = DeviceType_GRE; // UNKNOWN
      memAttrs.outerHints     = GETBITS(attrField, 4, 5);
      memAttrs.shareable      = (sh & BIT(1));
      memAttrs.outerShareable = (sh == 0b10);
      if (sh == 0b01)
        THROW_UNPREDICTABLE();

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
        THROW_UNPREDICTABLE();
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
          THROW_UNPREDICTABLE();
      }
    }

    return memAttrs;
  }

  /* _CheckPermission {{{4
   * ----------------
   */
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
        default: THROW_UNPREDICTABLE();
      }

    if (!fault)
      return _DefaultExcInfo();

    if (_HaveMainExt()) {
      uint8_t fsr = 0;
      switch (accType) {
        case AccType_IFETCH:
          fsr |= REG_CFSR__MMFSR__IACCVIOL;
          break;
        case AccType_STACK:
          if (isWrite)
            fsr |= REG_CFSR__MMFSR__MSTKERR;
          else
            fsr |= REG_CFSR__MMFSR__MUNSTKERR;
          break;
        case AccType_LAZYFP:
          fsr |= REG_CFSR__MMFSR__MLSPERR;
          break;
        case AccType_NORMAL:
        case AccType_ORDERED:
          fsr |= REG_CFSR__MMFSR__MMARVALID;
          fsr |= REG_CFSR__MMFSR__DACCVIOL;
          break;
        default:
          assert(false);
          break;
      }

      if (isSecure) {
        InternalOr32(REG_CFSR_S, fsr);
        if (fsr & REG_CFSR__MMFSR__MMARVALID)
          InternalStore32(REG_MMFAR_S, addr);
      } else {
        InternalOr32(REG_CFSR_NS, fsr);
        if (fsr & REG_CFSR__MMFSR__MMARVALID)
          InternalStore32(REG_MMFAR_NS, addr);
      }
    }

    return _CreateException(MemManage, true, isSecure);
  }

  /* _BigEndianReverse {{{4
   * -----------------
   */
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

  /* _DWT_DataMatch {{{4
   * --------------
   */
  void _DWT_DataMatch(uint32_t daddr, int dsize, uint32_t dvalue, bool read, bool nsReq) {
    bool triggerDebugEvent  = false;
    bool debugEvent         = false;

    uint32_t numComp = GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP);
    if (!_HaveDWT() || !numComp)
      return;

    for (uint32_t i=0; i<numComp; ++i) {
      if (_IsDWTConfigUnpredictable(i))
        THROW_UNPREDICTABLE();

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

  /* _DWT_DataAddressMatch {{{4
   * ---------------------
   * Check for match of access at "daddr". "dsize", "read" and "nsReq" are the
   * attributes for the access. Note that for a load or store instruction,
   * "nsReq" is the current security state of the PE, but this is not
   * necessarily true for a hardware stack push/pop or vector table access.
   * "nsReq" might not be the same as the "nsAttr" attribute the PE finally
   * uses to make the access.
   *
   * If comparators 'm' and 'm+1' form a Data Address Range comparator, then
   * this function returns the range match result when N=m+1.
   */
  bool _DWT_DataAddressMatch(int N, uint32_t daddr, int dsize, bool read, bool nsReq) {
    ASSERT(N < GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP)
        && (dsize == 1 || dsize == 2 || dsize == 4)
        && _Align(daddr, dsize) == daddr);

    bool validMatch = _DWT_ValidMatch(N, !nsReq);
    bool validAddr  = ((GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH) & 0b0100) == 0b0100);

    if (!validMatch || !validAddr)
      return false;

    bool linkedToAddr, linkedToData;
    if (N != GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP)-1) {
      linkedToAddr = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N+1)), REG_DWT_FUNCTION__MATCH) == 0b0111); // Data Address Limit
      linkedToData = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N+1)), REG_DWT_FUNCTION__MATCH) == 0b1011); // Linked Data Value
    } else {
      linkedToAddr = false;
      linkedToData = false;
    }

    bool matchLSC, linked;
    switch (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH) & 3) {
      case 0b00: matchLSC = true; linked = false; break;
      case 0b01: matchLSC = !read; linked = false; break;
      case 0b10: matchLSC = read; linked = false; break;
      case 0b11:
        ASSERT(N > 0);
        switch (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__MATCH) & 3) {
          case 0b00: matchLSC = true; linked = true; break;
          case 0b01: matchLSC = !read; linked = true; break;
          case 0b10: matchLSC = read; linked = true; break;
          default: abort();
        }
        break;
    }

    bool matchAddr;
    if (!linkedToAddr) {
      uint32_t vsize = BIT(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__DATAVSIZE));
      auto [matchEQ, matchGT] = _DWT_AddressCompare(daddr, InternalLoad32(REG_DWT_COMP(N)), dsize, vsize);

      if (linked) {
        validMatch = _DWT_ValidMatch(N-1, !nsReq);
        auto [lowerEQ, lowerGT] = _DWT_AddressCompare(daddr, InternalLoad32(REG_DWT_COMP(N-1)), dsize, 1);
        matchAddr = validMatch && (lowerEQ || lowerGT) && !matchGT;
      } else
        matchAddr = matchEQ;
    } else
      matchAddr = false;

    return matchAddr && matchLSC;
  }

  /* _DWT_AddressCompare {{{4
   * -------------------
   * Returns a pair of values. The first result is whether the (masked)
   * addresses are equal, where the access address (addr) is masked according
   * to REG_DWT_FUNCTION(n)__DATAVSIZE and the comparator address (compAddr) is
   * masked according to the access size. The second result is whether the
   * (unmasked) addr is greater than the (unmasked) compAddr.
   */
  std::tuple<bool, bool> _DWT_AddressCompare(uint32_t addr, uint32_t compAddr, int size, int compSize) {
    // addr must be a multiple of size. Unaligned accesses are split into smaller accesses.
    ASSERT(_Align(addr, size) == addr);

    // compAddr must be a multiple of compSize.
    if (_Align(compAddr, compSize) != compSize)
      THROW_UNPREDICTABLE();

    bool addrMatch    = (_Align(addr, compSize) == _Align(compAddr, size));
    bool addrGreater  = (addr > compAddr);
    return {addrMatch, addrGreater};
  }

  /* _DWT_ValidMatch {{{4
   * ---------------
   * Returns TRUE if this match is permitted by the current authentication controls.
   */
  bool _DWT_ValidMatch(int N, bool secureMatch) {
    if (!_HaveSecurityExt())
      ASSERT(!secureMatch);

    // Check for disabled.
    if (!_NoninvasiveDebugAllowed()
     || !GETBITSM(InternalLoad32(REG_DEMCR), REG_DEMCR__TRCENA)
     || !GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH))
      return false;

    if (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ACTION) == 0b01) {
      bool hltEn = _CanHaltOnEvent(secureMatch);
      // Ignore priority when checking whether DebugMonitor activates DWT matches
      bool monEn = _HaveDebugMonitor() && _CanPendMonitorOnEvent(secureMatch, false);
      return hltEn || monEn;
    } else
      return !secureMatch || _SecureNoninvasiveDebugAllowed();
  }

  /* _DWT_DataValueMatch {{{4
   * -------------------
   * Check for match of access of "dvalue" at "daddr". "dsize", "read" and
   * "nsReq" are the attributes for the access. Note that for a load or store
   * instruction, "nsReq" is the current Security state of the PE, but this is
   * not necessarily true for a hardware stack push/pop or vector table access.
   * "nsReq" might not be the same as the "nsAttr" attribute the PE finally
   * uses to make the access.
   */
  bool _DWT_DataValueMatch(int N, uint32_t daddr, uint32_t dvalue, int dsize, bool read, bool nsReq) {
    ASSERT(N < GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP)
        && (dsize == 1 || dsize == 2 || dsize == 4)
        && _Align(daddr, dsize) == daddr);

    bool validMatch = _DWT_ValidMatch(N, !nsReq);
    bool validData  = (GETBITS(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH), 2, 3) == 0b10);

    if (!validMatch || !validData)
      return false;

    bool matchLSC, linked;
    switch (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH) & 3) {
      case 0b00: matchLSC = true; linked = false; break;
      case 0b01: matchLSC = !read; linked = false; break;
      case 0b10: matchLSC = read; linked = false; break;
      case 0b11:
        ASSERT(N > 0);
        switch (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__MATCH) & 3) {
          case 0b00: matchLSC = true; linked = true; break;
          case 0b01: matchLSC = !read; linked = true; break;
          case 0b10: matchLSC = read; linked = true; break;
          default: abort();
        }
        break;
    }

    uint32_t vsize = BIT(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__DATAVSIZE));

    uint32_t dmask;
    if (linked) {
      dmask = 0b0000; // Filled in below if there is an address match
      if (_DWT_DataAddressMatch(N-1, daddr, dsize, read, nsReq)) {
        if (vsize == 1 && dsize == 1)
          dmask = CHGBIT(dmask, 0, 1);
        else if (vsize == 1 && dsize == 2)
          dmask = CHGBIT(dmask, GETBITS(InternalLoad32(REG_DWT_COMP(N-1)), 0, 0), 1);
        else if (vsize == 1 && dsize == 4)
          dmask = CHGBIT(dmask, GETBITS(InternalLoad32(REG_DWT_COMP(N-1)), 0, 1), 1);
        else if (vsize == 2 && dsize == 2)
          dmask = CHGBITS(dmask, 0, 1, 0b11);
        else if (vsize == 2 && dsize == 4)
          dmask = CHGBITS(dmask, GETBITS(InternalLoad32(REG_DWT_COMP(N-1)), 0, 1),
                                 GETBITS(InternalLoad32(REG_DWT_COMP(N-1)), 0, 1)+1, 0b11);
        else if (vsize == 4 && dsize == 4)
          dmask = 0b1111;
        else
          dmask = 0b0000; // vsize > dsize: no match
      }
    } else {
      switch (dsize) {
        case 1: dmask = 0b0001; break;
        case 2: dmask = 0b0011; break;
        case 4: dmask = 0b1111; break;
        default: abort();
      }
    }

    // Split both values into byte lanes: DCBA and dcba.
    // This function relies on the values being correctly replicated across
    // REG_DWT_COMP(N).
    uint32_t D = GETBITS(dvalue,24,31);
    uint32_t C = GETBITS(dvalue,16,23);
    uint32_t B = GETBITS(dvalue, 8,15);
    uint32_t A = GETBITS(dvalue, 0, 7);

    uint32_t d = GETBITS(InternalLoad32(REG_DWT_COMP(N)),24,31);
    uint32_t c = GETBITS(InternalLoad32(REG_DWT_COMP(N)),16,23);
    uint32_t b = GETBITS(InternalLoad32(REG_DWT_COMP(N)), 8,15);
    uint32_t a = GETBITS(InternalLoad32(REG_DWT_COMP(N)), 0, 7);

    // Partial results.
    bool Dd = (GETBIT(dmask, 3) && D == d);
    bool Cc = (GETBIT(dmask, 2) && C == c);
    bool Bb = (GETBIT(dmask, 1) && B == b);
    bool Aa = (GETBIT(dmask, 0) && A == a);

    // Combined partial results.
    bool BAba = Bb && Aa;
    bool DCdc = Dd && Cc;
    bool DCBAdcba = Dd && Cc && Bb && Aa;

    // Generate full results.
    bool matchData;
    if (vsize == 1)
      matchData = (Dd || Cc || Bb || Aa);
    else if (vsize == 2 && (dsize == 2 || dsize == 4))
      matchData = (DCdc || BAba);
    else if (vsize == 4 && dsize == 4)
      matchData = DCBAdcba;
    else
      matchData = false;

    return matchData && matchLSC;
  }

  /* _IsDWTConfigUnpredictable {{{4
   * -------------------------
   * Checks for the UNPREDICTABLE cases for various combinations of MATCH and
   * ACTION for each comparator.
   */
  bool _IsDWTConfigUnpredictable(int N) {
    bool noTrace = (!_HaveMainExt() || GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NOTRCPKT) || !_HaveITM());

    // First pass check of MATCH field — coarse checks.
    switch (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH)) {
      case 0b0000: // Disabled
        return false;
      case 0b0001: // Cycle counter match
        if (!_HaveMainExt() || GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NOCYCCNT)
         || !(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(0)))
          return true;
        break;
      case 0b0010: // Instruction address
      case 0b0011:
        if (!(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(1))
         || GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__DATAVSIZE) != 0b01
         || GETBIT(InternalLoad32(REG_DWT_COMP(N)), 0))
          return true;
        break;
      case 0b0100: // Data address
      case 0b0101:
      case 0b0110:
      case 0b0111: {
        uint32_t lsb = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__DATAVSIZE);
        if (!(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(3))
         || (lsb > 0 && !_IsZero(GETBITS(InternalLoad32(REG_DWT_COMP(N)), 0, lsb-1))))
          return true;
      } break;
      case 0b1100: // Data address with value
      case 0b1101:
      case 0b1110: {
        if (noTrace)
          return true;
        uint32_t lsb = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__DATAVSIZE);
        if (!(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(3))
          || (lsb > 0 && !_IsZero(GETBITS(InternalLoad32(REG_DWT_COMP(N)), 0, lsb-1))))
          return true;
      } break;
      case 0b1000: // Data value
      case 0b1001:
      case 0b1010:
      case 0b1011: {
        uint32_t vsize = BIT(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__DATAVSIZE));
        if (!_HaveMainExt()
         || !(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(2))
         || (vsize != 4 && GETBITS(InternalLoad32(REG_DWT_COMP(N)), 16, 31) != GETBITS(InternalLoad32(REG_DWT_COMP(N)), 0, 15))
         || (vsize == 1 && GETBITS(InternalLoad32(REG_DWT_COMP(N)),  8, 15) != GETBITS(InternalLoad32(REG_DWT_COMP(N)), 0,  7)))
          return true;
      } break;
      default:
       return true;
    }

    // Second pass MATCH check — linked and limit comparators.
    switch (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH)) {
      case 0b0011: { // Instruction address limit
        uint32_t m = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__MATCH);
        if (!N
         || !(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(4))
         || (m == 0b0001 || m == 0b0011 || ((m & 0b1100) == 0b0100) || (m & 0b1000))
         || (InternalLoad32(REG_DWT_COMP(N)) <= InternalLoad32(REG_DWT_COMP(N-1))))
          return true;
        if (!m)
          return false;
      } break;

      case 0b0111: { // Data address limit
        uint32_t m = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__MATCH);
        if (!N
         || !(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(4))
         || (m == 0b0001 || ((m & 0b1110) == 0b0010) || m == 0b0111 || (m & 0b1100) == 0b1000)
         || (InternalLoad32(REG_DWT_COMP(N)) <= InternalLoad32(REG_DWT_COMP(N-1))))
          return true;
        if (!m)
          return false;
      } break;

      case 0b1011: { // Linked data value
        uint32_t m = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__MATCH);
        if (!N
         || !(GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ID) & BIT(4))
         || (m == 0b0001 || ((m & 0b1110) == 0b0010) || m == 0b0111 || (m & 0b1100) == 0b1000)
         || GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N  )), REG_DWT_FUNCTION__DATAVSIZE) !=
            GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__DATAVSIZE))
          return true;
        if (!m)
          return false;
      } break;
    }

    // Check DATAVSIZE is permitted.
    if (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__DATAVSIZE) == 0b11)
      return true;

    // Check the ACTION is allowed for the MATCH type.
    switch (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__ACTION)) {
      case 0b00: { // CMPMATCH trigger only
        uint32_t m = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH);
        if (m == 0b1100 || m == 0b1101 || m == 0b1110)
          return true;
      } break;
      case 0b01: { // Debug event
        uint32_t m = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH);
        if (m == 0b0011 || m == 0b0111 || m == 0b1100 || m == 0b1101 || m == 0b1110)
          return true;
      } break;
      case 0b10: { // Data Trace Match or Data Value packet
        uint32_t m = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH);
        if (noTrace || m == 0b0011 || m == 0b0111)
          return true;
      } break;
      case 0b11: { // Other Data Trace Packet
        uint32_t m = GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH);
        if (noTrace
         || (m == 0b0010 || m == 0b1000 || m == 0b1001 || m == 0b1010)
         || (m == 0b0011 && GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__ACTION) != 0b00)
         || (m == 0b0111 && (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__MATCH) & 0b1100) == 0b0100
                         && (   GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__ACTION) == 0b01
                             || GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__ACTION) == 0b10))
         || (m == 0b0111 && (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__MATCH) & 0b1100) == 0b1100
                         && (   GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__ACTION) == 0b00
                             || GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N-1)), REG_DWT_FUNCTION__ACTION) == 0b01)))
          return true;
      } break;
    }

    return false; // Passes checks
  }

  /* _SetDWTDebugEvent {{{4
   * -----------------
   */
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

  /* _DefaultMemoryAttributes {{{4
   * ------------------------
   */
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

  /* _DefaultPermissions {{{4
   * -------------------
   */
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

  /* _SetPending {{{4
   * -----------
   * XXX. This function has been augmented relative to the ISA manual's
   * definition to add an additional check parameter allowing us to choose to
   * test _ExceptionTargetsSecure, which is useful for implementing NVIC
   * registers.
   */
  void _SetPending(int exc, bool isSecure, bool setNotClear, bool check=false) {
    if (!_HaveSecurityExt())
      isSecure = false;

    if (_IsExceptionTargetConfigurable(exc)) {
      if (!check || _ExceptionTargetsSecure(exc, isSecure) == isSecure)
        _s.excPending[exc] = setNotClear ? 0b11 : 0b00;
    } else {
      uint32_t idx = isSecure ? 0 : 1;
      _s.excPending[exc] = CHGBITS(_s.excPending[exc], idx, idx, setNotClear);
    }
  }

  /* _SetEnable {{{4
   * ----------
   * XXX. This does not appear in the ISA manual but has been constructed
   * analagously to _SetPending.
   */
  void _SetEnable(int exc, bool isSecure, bool setNotClear, bool check=false) {
    if (!_HaveSecurityExt())
      isSecure = false;

    if (_IsExceptionTargetConfigurable(exc)) {
      if (!check || _ExceptionTargetsSecure(exc, isSecure) == isSecure)
        _s.excEnable[exc] = setNotClear ? 0b11 : 0b00;
    } else {
      uint32_t idx = isSecure ? 0 : 1;
      _s.excEnable[exc] = CHGBITS(_s.excEnable[exc], idx, idx, setNotClear);
    }
  }

  /* _NextInstrITState {{{4
   * -----------------
   */
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

  /* _PendingExceptionDetails {{{4
   * ------------------------
   */
  // XXX: Unfortunately the ARM ISA manual exceptionally does not give a definition
  // for this function. Its definition has been estimated via reference to qemu's codebase.
  //
  // XXX: This function has been augmented relative to the ARM ISA manual to add an argument
  // `ignorePrimask`, which is useful for implementing WFI.
  //
  // TODO: DHCSR.C_MASKINTS
  std::tuple<bool, int, bool> _PendingExceptionDetails(bool ignorePrimask=false) {
    // XXX: Not specified exactly where SysTick should be checked, so we choose
    // to check it here like everything else.
    bool systIntrS  = (_HaveSysTick() && _SystGetIntrFlag(false, true));
    bool systIntrNS = (_HaveSysTick() == 2 && _SystGetIntrFlag(true, true));
    if (systIntrS)
      _SetPending(SysTick, true, true);
    if (systIntrNS)
      _SetPending(SysTick, false, true);

    // _NvicPendingPriority() has a value higher than the highest possible
    // priority value if there is no pending interrupt so there is an interrupt
    // to be handled iff this is true.
    auto [pendingPrio, pendingExcNo, excIsSecure] = _PendingExceptionDetailsActual();
    bool canTakePendingExc = (_ExecutionPriority(ignorePrimask) > pendingPrio);

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

  /* _RawExecutionPriority {{{4
   * ---------------------
   */
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

  /* _HighestPri {{{4
   * -----------
   */
  int _HighestPri() {
    return 256;
  }

  /* _RestrictedNSPri {{{4
   * ----------------
   */
  int _RestrictedNSPri() {
    return 0x80;
  }

  /* _FindPriv {{{4
   * ---------
   */
  bool _FindPriv() {
    return _CurrentModeIsPrivileged();
  }

  /* _ExceptionEntry {{{4
   * ---------------
   */
  ExcInfo _ExceptionEntry(int excType, bool toSecure, bool instExecOk) {
    ExcInfo exc = _PushStack(toSecure, instExecOk);
    if (exc.fault == NoFault)
      exc = _ExceptionTaken(excType, false, toSecure, false);
    return exc;
  }

  /* _PushStack {{{4
   * ----------
   */
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

  /* _MergeExcInfo {{{4
   * -------------
   */
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

  /* _ReturnState {{{4
   * ------------
   */
  std::tuple<uint32_t, uint8_t> _ReturnState(bool instExecOk) {
    if (instExecOk)
      return {_NextInstrAddr(), _NextInstrITState()};
    else
      return {_ThisInstrAddr(), _ThisInstrITState()};
  }

  /* _DerivedLateArrival {{{4
   * -------------------
   */
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

  /* _ComparePriorities {{{4
   * ------------------
   */
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

  /* _ActivateException {{{4
   * ------------------
   */
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

  /* _SetActive {{{4
   * ----------
   */
  void _SetActive(int exc, bool isSecure, bool setNotClear) {
    if (!_HaveSecurityExt())
      isSecure = false;

    if (_IsExceptionTargetConfigurable(exc)) {
      if (_ExceptionTargetsSecure(exc, isSecure/*UNKNOWN TODO*/) == isSecure)
        _s.excActive[exc] = setNotClear ? 0b11 : 0b00;
    } else {
      uint32_t idx = isSecure ? 0 : 1;
      _s.excActive[exc] = CHGBITS(_s.excActive[exc], idx, idx, setNotClear ? 1 : 0);
    }
  }

  /* _TailChain {{{4
   * ----------
   */
  ExcInfo _TailChain(int excNo, bool excIsSecure, uint32_t excReturn) {
    if (!_HaveFPExt())
      excReturn = CHGBITSM(excReturn, EXC_RETURN__FTYPE, 1);
    excReturn = CHGBITSM(excReturn, EXC_RETURN__PREFIX, 0xFF);
    _SetLR(excReturn);

    return _ExceptionTaken(excNo, true, excIsSecure, false);
  }

  /* _ConsumeExcStackFrame {{{4
   * ---------------------
   */
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

  /* _ExceptionReturn {{{4
   * ----------------
   */
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

  /* _ExceptionActiveBitCount {{{4
   * ------------------------
   */
  int _ExceptionActiveBitCount() {
    int count = 0;
    for (int i=0; i<=_MaxExceptionNum(); ++i)
      for (int j=0; j<2; ++j)
        if (_IsActiveForState(i, !j))
          ++count;
    return count;
  }

  /* _DeActivate {{{4
   * -----------
   */
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

  /* _SleepOnExit {{{4
   * ------------
   */
  void _SleepOnExit() {
    // Handling this is the responsibility of the code calling into the Simulator.
    _s.exitCause |= EXIT_CAUSE__SLEEP_ON_EXIT;
  }

  /* _WaitForInterrupt {{{4
   * -----------------
   */
  void _WaitForInterrupt() {
    // Handling this is the responsibility of the code calling into the Simulator.
    _s.exitCause |= EXIT_CAUSE__WFI;
  }

  /* _WaitForEvent {{{4
   * -------------
   */
  void _WaitForEvent() {
    // Handling this is the responsibility of the code calling into the Simulator.
    _s.exitCause |= EXIT_CAUSE__WFE;
  }

  /* _IsIrqValid {{{4
   * -----------
   */
  bool _IsIrqValid(int e) {
    return e >= 16 && e <= _cfg.MaxExc();
  }

  /* _PopStack {{{4
   * ---------
   */
  ExcInfo _PopStack(uint32_t excReturn) {
    PEMode    mode      = (GETBITSM(excReturn, EXC_RETURN__MODE) ? PEMode_Thread : PEMode_Handler);
    bool      toSecure  = _HaveSecurityExt() && GETBITSM(excReturn, EXC_RETURN__S);
    RName     spName    = _LookUpSP_with_security_mode(toSecure, mode);
    uint32_t  framePtr  = _GetSP(spName);
    if (!_IsAligned(framePtr, 8))
      THROW_UNPREDICTABLE();

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
        InternalOr32(REG_CFSR, REG_CFSR__UFSR__INVPC);
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

  /* _CheckCPEnabled {{{4
   * ---------------
   */
  ExcInfo _CheckCPEnabled(int cp) {
    return _CheckCPEnabled(cp, _CurrentModeIsPrivileged(), _IsSecure());
  }

  ExcInfo _CheckCPEnabled(int cp, bool priv, bool secure) {
    auto [enabled, toSecure] = _IsCPEnabled(cp, priv, secure);
    ExcInfo excInfo;
    if (!enabled) {
      if (toSecure)
        InternalOr32(REG_CFSR_S, REG_CFSR__UFSR__NOCP);
      else
        InternalOr32(REG_CFSR_NS, REG_CFSR__UFSR__NOCP);
      excInfo = _CreateException(UsageFault, true, toSecure);
    } else
      excInfo = _DefaultExcInfo();
    return excInfo;
  }

  /* _ValidateExceptionReturn {{{4
   * ------------------------
   */
  std::tuple<ExcInfo, uint32_t> _ValidateExceptionReturn(uint32_t excReturn, int retExcNo) {
    bool error = false;
    assert(_CurrentMode() == PEMode_Handler);
    if (GETBITS(excReturn, 7,23) != BITS(0,16) || GETBITS(excReturn, 1, 1))
      THROW_UNPREDICTABLE();
    if (!_HaveFPExt() && !GETBITSM(excReturn, EXC_RETURN__FTYPE))
      THROW_UNPREDICTABLE();

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
          InternalOr32(REG_CFSR, REG_CFSR__UFSR__INVPC);
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

  /* _ExceptionTaken {{{4
   * ---------------
   */
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

  /* _PushCalleeStack {{{4
   * ----------------
   */
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

  /* _SCS_UpdateStatusRegs {{{4
   * ---------------------
   */
  void _SCS_UpdateStatusRegs() {
    // TODO
  }

  /* _ConstrainUnpredictableBool {{{4
   * ---------------------------
   */
  bool _ConstrainUnpredictableBool(bool x) { return x; }

  /* _ExceptionPriority {{{4
   * ------------------
   */
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

  /* _TopLevel {{{4
   * ---------
   * Called once for each tick the PE is not in a sleep state. Handles all
   * instruction processing, including fetching the opcode, decode and execute.
   * It also handles pausing execution when in the lockup state.
   */
  void _TopLevel() {
    // Implementation-specific: Set exit cause value
    _s.exitCause = 0;

    // If the PE has locked up then abort execution of this instruction. Set
    // the length of the current instruction to 0 so NextInstrAddr() reports
    // the correct lockup address.
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
        //TRACE("fetched %d-bit insn: 0x%08x\n", is16bit ? 16 : 32, instr);

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
                  InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNDEFINSTR);
                else {
                  InternalOr32(REG_CFSR, REG_CFSR__UFSR__NOCP);
                  toSecure = cpFaultState;
                }
              }
            } else
              InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNDEFINSTR);

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
        _InstructionAdvance(ok);
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

  /* _EndOfInstruction {{{4
   * -----------------
   */
  void _EndOfInstruction() {
    throw Exception(ExceptionType::EndOfInstruction);
  }

  /* _CreateException {{{4
   * ----------------
   */
  ExcInfo _CreateException(int exc, bool forceSecurity, bool isSecure, bool isSync=true) {
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

  /* _UpdateSecureDebugEnable {{{4
   * ------------------------
   */
  void _UpdateSecureDebugEnable() {
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

  /* _ColdReset {{{4
   * ----------
   * Does not correspond to any function in the ISA manual psuedocode; implements a cold
   * reset as described in the manual. A cold reset is a superset of a warm reset.
   */
  void _ColdReset() {
    // TODO
    _TakeReset();
  }

  /* _TakeReset {{{4
   * ----------
   * This implements a warm reset.
   */
  void _TakeReset() {
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

  /* _SteppingDebug {{{4
   * --------------
   */
  bool _SteppingDebug() {
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

  /* _FetchInstr {{{4
   * -----------
   */
  std::tuple<uint32_t,bool> _FetchInstr(uint32_t addr) {
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
        InternalOr32(REG_CFSR, REG_CFSR__UFSR__INVSTATE);
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

  /* _GenerateDebugEventResponse {{{4
   * ---------------------------
   */
  bool _GenerateDebugEventResponse() {
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

  /* _FPB_CheckBreakPoint {{{4
   * --------------------
   */
  bool _FPB_CheckBreakPoint(uint32_t iaddr, int size, bool isIFetch, bool isSecure) {
    bool match = _FPB_CheckMatchAddress(iaddr);
    if (!match && size == 4 && _FPB_CheckMatchAddress(iaddr+2))
      match = _ConstrainUnpredictableBool(true/*Unpredictable_FPBreakpoint*/);
    return match;
  }

  /* _FPB_CheckMatchAddress {{{4
   * ----------------------
   */
  bool _FPB_CheckMatchAddress(uint32_t iaddr) {
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

  /* _ExceptionDetails {{{4
   * -----------------
   */
  std::tuple<bool,bool> _ExceptionDetails(int exc, bool isSecure, bool isSync) {
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

  /* _HandleException {{{4
   * ----------------
   */
  void _HandleException(const ExcInfo &excInfo) {
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

  /* _InstructionAdvance {{{4
   * -------------------
   */
  void _InstructionAdvance(bool instExecOk) {
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

  /* _ConditionHolds {{{4
   * ---------------
   */
  bool _ConditionHolds(uint32_t cond) {
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

  /* _SetMonStep {{{4
   * -----------
   */
  void _SetMonStep(bool monStepActive) {
    if (!monStepActive)
      return;

    if (!(InternalLoad32(REG_DEMCR) & REG_DEMCR__MON_STEP))
      THROW_UNPREDICTABLE();

    if (_ExceptionPriority(DebugMonitor, _IsSecure(), true) < _ExecutionPriority()) {
      InternalOr32(REG_DEMCR, REG_DEMCR__MON_PEND);
      InternalOr32(REG_DFSR, REG_DFSR__HALTED);
    }
  }

  /* _ExceptionTargetsSecure {{{4
   * -----------------------
   */
  bool _ExceptionTargetsSecure(int excNo, bool isSecure) {
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
          // XXX. The ISA manual does not give a definition here but suggests
          // that targetSecure should equal whether the S or NS SysTick timer
          // is raising. This is only applicable if we are in a context of handling
          // an exception and this function is called from other places. However,
          // it *should* never be called for SysTick if _HaveSysTick() == 2, because
          // then !_IsExceptionTargetConfigurable(SysTick), and this function
          // never be called when !_IsExceptionTargetConfigurable() for SysTick specifically
          // unless we are actually handling an actually pending exception. In this circumstance,
          // we will be called by _PendingExceptionDetailsActual() and isSecure will be
          // equal to the correct value.
          targetSecure = isSecure;
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

  /* _IsCPInstruction {{{4
   * ----------------
   */
  std::tuple<bool, int> _IsCPInstruction(uint32_t instr) {
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

  /* _DWT_InstructionMatch {{{4
   * ---------------------
   */
  void _DWT_InstructionMatch(uint32_t iaddr) {
    bool triggerDebugEvent  = false;
    bool debugEvent         = false;

    if (!_HaveDWT() || _IsZero(InternalLoad32(REG_DWT_CTRL) & REG_DWT_CTRL__NUMCOMP))
      // No comparator support.
      return;

    for (int i=0; i<GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP); ++i) {
      if (_IsDWTConfigUnpredictable(i))
        THROW_UNPREDICTABLE();

      bool instrAddrMatch = _DWT_InstructionAddressMatch(i, iaddr);
      if (!instrAddrMatch)
        continue;

      // Instruction Address
      if (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__MATCH) == 0b0010) {
        InternalOr32(REG_DWT_FUNCTION(i), REG_DWT_FUNCTION__MATCHED);
        debugEvent = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__ACTION) == 0b01);
      }

      // Instruction Address Limit
      else if (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i)), REG_DWT_FUNCTION__MATCH) == 0b0011) {
        ASSERT(i > 0);
        InternalOr32(REG_DWT_FUNCTION(i  ), REG_DWT_FUNCTION__MATCHED); // UNKNOWN
        InternalOr32(REG_DWT_FUNCTION(i-1), REG_DWT_FUNCTION__MATCHED);
        debugEvent = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(i-1)), REG_DWT_FUNCTION__ACTION) == 0b01);
      }

      triggerDebugEvent = triggerDebugEvent || debugEvent;
    }

    if (triggerDebugEvent)
      debugEvent = _SetDWTDebugEvent(_IsSecure());

    assert(false); // TODO
  }

  /* _DWT_InstructionAddressMatch {{{4
   * ----------------------------
   */
  bool _DWT_InstructionAddressMatch(int N, uint32_t iaddr) {
    ASSERT(N < GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP) && _Align(iaddr, 2) == iaddr);

    bool secureMatch = _IsSecure();
    bool validMatch  = _DWT_ValidMatch(N, secureMatch);
    bool validInstr  = ((GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH) & 0b1110) == 0b0010);

    if (!validMatch || !validInstr)
      return false;

    bool linkedToInstr;
    if (N != GETBITSM(InternalLoad32(REG_DWT_CTRL), REG_DWT_CTRL__NUMCOMP)-1)
      linkedToInstr = (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N+1)), REG_DWT_FUNCTION__MATCH) == 0b0011);
    else
      linkedToInstr = false;

    bool linked;
    if (GETBITSM(InternalLoad32(REG_DWT_FUNCTION(N)), REG_DWT_FUNCTION__MATCH) == 0b0011)
      linked = true;
    else
      linked = false;

    bool matchAddr;
    if (!linkedToInstr) {
      auto [matchEQ, matchGT] = _DWT_AddressCompare(iaddr, InternalLoad32(REG_DWT_COMP(N)), 2, 2);
      if (linked) {
        validMatch = _DWT_ValidMatch(N-1, secureMatch);
        auto [lowerEQ, lowerGT] = _DWT_AddressCompare(iaddr, InternalLoad32(REG_DWT_COMP(N-1)), 2, 2);
        matchAddr = (validMatch && (lowerEQ || lowerGT) && !matchGT);
      } else
        matchAddr = matchEQ;
    } else
      matchAddr = false;

    return matchAddr;
  }

  /* _IsCPEnabled {{{4
   * ------------
   */
  std::tuple<bool, bool> _IsCPEnabled(int cp, bool priv, bool secure) {
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
        THROW_UNPREDICTABLE();
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

  /* _GetMemI {{{4
   * --------
   */
  uint16_t _GetMemI(uint32_t addr) {
    uint16_t value;
    auto [excInfo, memAddrDesc] = _ValidateAddress(addr, AccType_IFETCH, _FindPriv(), _IsSecure(), false, true);
    if (excInfo.fault == NoFault) {
      bool error;
      std::tie(error, value) = _GetMem(memAddrDesc, 2);
      if (error) {
        value = UINT16_MAX; // UNKNOWN
        InternalOr32(REG_CFSR, REG_CFSR__BFSR__IBUSERR);
        excInfo = _CreateException(BusFault, false, false/*UNKNOWN*/);
        TRACE("fetch failed\n");
      }
    } else TRACE("fetch addr validate failed 0x%x\n", addr);

    _HandleException(excInfo);
    if (_IsDWTEnabled())
      _DWT_InstructionMatch(addr);
    return value;
  }

  /* _ExecutionPriority {{{4
   * ------------------
   * XXX: This has been augmented relative to the ISA manual psuedocode to add
   * an argument `ignorePrimask`, to assist in the implementation of WFI.
   */
  int _ExecutionPriority(bool ignorePrimask=false) {
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

    if (!ignorePrimask) {
      if (_s.primaskNS & 1) {
        if (!(InternalLoad32(REG_AIRCR_S) & REG_AIRCR__PRIS))
          boostedPri = 0;
        else if (boostedPri > priSNsPri)
          boostedPri = priSNsPri;
      }

      if (_s.primaskS & 1)
        boostedPri = 0;
    }

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

  /* _SecurityCheck {{{4
   * --------------
   */
  SAttributes _SecurityCheck(uint32_t addr, bool isInstrFetch, bool isSecure) {
    SAttributes result = {};

    bool idauExempt = false;
    bool idauNs = true;
    bool idauNsc = true;

    if (IMPL_DEF_IDAU_PRESENT)
      std::tie(idauExempt, idauNs, idauNsc, result.iregion, result.irvalid) = _IDAUCheck(addr & ~BITS(0,4));

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
          auto [rbar,rlar] = _InternalLoadSauRegion(r);
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

  /* _IDAUCheck {{{4
   * ----------
   */
  std::tuple<bool, bool, bool, uint8_t, bool> _IDAUCheck(uint32_t addr) {
    return _dev.IDAUCheck(addr);
  }

  /* _LowestSetBit {{{4
   * -------------
   */
  static int _LowestSetBit(uint32_t x) {
    ASSERT(x);
    return CTZL(x);
  }

  /* _DecodeRegShift {{{4
   * ---------------
   */
  SRType _DecodeRegShift(uint32_t srType) {
    switch (srType) {
      case 0b00: return SRType_LSL;
      case 0b01: return SRType_LSR;
      case 0b10: return SRType_ASR;
      case 0b11: return SRType_ROR;
      default: abort();
    }
  }

  /* _BKPTInstrDebugEvent {{{4
   * --------------------
   */
  void _BKPTInstrDebugEvent() {
    if (!_GenerateDebugEventResponse()) {
      auto excInfo = _CreateException(HardFault, false, UNKNOWN_VAL(false));
      _HandleException(excInfo);
    }
  }

  /* _Hint_Yield {{{4
   * -----------
   * Performs a Yield hint.
   */
  void _Hint_Yield() {
    // TODO
    _s.exitCause |= EXIT_CAUSE__YIELD;
  }

  /* _Hint_Debug {{{4
   * -----------
   * Generate a hint to the debug system.
   */
  void _Hint_Debug(uint32_t option) {
    // TODO
    _s.exitCause |= EXIT_CAUSE__DBG;
  }

  /* _Hint_PreloadData {{{4
   * -----------------
   * Performs a preload data hint.
   */
  void _Hint_PreloadData(uint32_t addr) {
    // TODO
  }

  /* _Hint_PreloadDataForWrite {{{4
   * -------------------------
   * Performs a preload data hint for write.
   */
  void _Hint_PreloadDataForWrite(uint32_t addr) {
    // TODO
  }

  /* _Hint_PreloadInstr {{{4
   * ------------------
   * Performs a preload instruction hint.
   */
  void _Hint_PreloadInstr(uint32_t addr) {
    // TODO
  }

  /* _CallSupervisor {{{4
   * ---------------
   */
  void _CallSupervisor() {
    auto excInfo = _CreateException(SVCall, false, UNKNOWN_VAL(false));
    _HandleException(excInfo);
  }

  /* _TTResp {{{4
   * -------
   */
  uint32_t _TTResp(uint32_t addr, bool alt, bool forceUnpriv) {
    uint32_t resp = 0;

    // Only allow security checks if currently in Secure state.
    bool addrSecure;
    if (_IsSecure()) {
      auto sAttributes = _SecurityCheck(addr, false, _IsSecure());
      if (sAttributes.srvalid) {
        resp = CHGBITSM(resp, TT_RESP__SREGION, sAttributes.sregion);
        resp = CHGBITSM(resp, TT_RESP__SRVALID, 1);
      }
      if (sAttributes.irvalid) {
        resp = CHGBITSM(resp, TT_RESP__IREGION, sAttributes.iregion);
        resp = CHGBITSM(resp, TT_RESP__IRVALID, 1);
      }

      addrSecure = sAttributes.ns ? 0 : 1;
      resp = CHGBITSM(resp, TT_RESP__S, addrSecure);
    }

    // MPU region information only available when privileged or when
    // inspecting the other MPU state.
    bool otherDomain = (alt != _IsSecure());
    if (_CurrentModeIsPrivileged() || alt) {
      auto [write, read, region, hit] = _IsAccessible(addr, forceUnpriv, otherDomain);
      if (hit) {
        resp = CHGBITSM(resp, TT_RESP__MREGION, region);
        resp = CHGBITSM(resp, TT_RESP__MRVALID, 1);
      }
      resp = CHGBITSM(resp, TT_RESP__R,  read);
      resp = CHGBITSM(resp, TT_RESP__RW, write);
      if (_IsSecure()) {
        resp = CHGBITSM(resp, TT_RESP__NSR,  (read && !addrSecure));
        resp = CHGBITSM(resp, TT_RESP__NSRW, (write && !addrSecure));
      }
    }

    return resp;
  }

  /* _IsAccessible {{{4
   * -------------
   */
  std::tuple<bool, bool, uint8_t, bool> _IsAccessible(uint32_t addr, bool forceUnpriv, bool isSecure) {
    bool write, read;

    // Work out which privilege level the current mode in the Non-secure state
    // is subject to.
    bool isPrivileged;
    if (forceUnpriv)
      isPrivileged = false;
    else
      isPrivileged = (_CurrentMode() == PEMode_Handler
        || (isSecure ? !GETBITSM(_s.controlS,  CONTROL__NPRIV)
                     : !GETBITSM(_s.controlNS, CONTROL__NPRIV)));

    auto [_, perms] = _MPUCheck(addr, AccType_NORMAL, isPrivileged, isSecure);
    if (!perms.apValid) {
      write = false;
      read  = false;
    } else
      switch (perms.ap) {
        case 0b00:
          if (isPrivileged) write = true,  read = true;
          else              write = false, read = false;
          break;
        case 0b01:
          write = true, read = true;
          break;
        case 0b10:
          if (isPrivileged) write = false, read = true;
          else              write = false, read = false;
          break;
        case 0b11:
          write = false, read = true;
          break;
        default:
          abort();
      }

    return {write, read, perms.region, perms.regionValid};
  }

  /* _SetExclusiveMonitors {{{4
   * ---------------------
   */
  void _SetExclusiveMonitors(uint32_t addr, int size) {
    bool isSecure = (_s.curState == SecurityState_Secure);
    auto [excInfo, memAddrDesc] = _ValidateAddress(addr, AccType_NORMAL, _FindPriv(), isSecure, false, true);
    _HandleException(excInfo);

    if (memAddrDesc.memAttrs.shareable)
      _MarkExclusiveGlobal(memAddrDesc.physAddr, _ProcessorID(), size);

    _MarkExclusiveLocal(memAddrDesc.physAddr, _ProcessorID(), size);
  }

  /* _MarkExclusiveGlobal {{{4
   * --------------------
   * Records in a global record that the PE has requested "exclusive access"
   * covering at least size bytes from the address.
   */
  void _MarkExclusiveGlobal(uint32_t addr, int processorID, int size) {
    auto lk = _gm.Lock();
    _gm.MarkExclusive(addr, processorID, size);
  }

  /* _MarkExclusiveLocal {{{4
   * -------------------
   * Records in a local record that the PE has requested "exclusive access"
   * covering at least size bytes from the address.
   */
  void _MarkExclusiveLocal(uint32_t addr, int processorID, int size) {
    _lm.MarkExclusive(addr, size);
  }

  /* _ExclusiveMonitorsPass {{{4
   * ----------------------
   */
  bool _ExclusiveMonitorsPass(uint32_t addr, int size) {
    // It is IMPLEMENTATION DEFINED whether the detection of memory aborts
    // happens before or after the check on the local Exclusive Monitor. As a
    // result a failure of the local monitor can occur on some implementations
    // even if the memory access would give a memory abort.

    ExcInfo           excInfo;
    AddressDescriptor memAddrDesc;
    if (addr != _Align(addr, size)) {
      InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNALIGNED);
      excInfo = _CreateException(UsageFault, false, UNKNOWN_VAL(false));
    } else
      std::tie(excInfo, memAddrDesc) =
        _ValidateAddress(addr, AccType_NORMAL, _FindPriv(), _IsSecure(), true, true);

    _HandleException(excInfo);

    bool passed = _IsExclusiveLocal(memAddrDesc.physAddr, _ProcessorID(), size);
    if (memAddrDesc.memAttrs.shareable)
      passed = passed && _IsExclusiveGlobal(memAddrDesc.physAddr, _ProcessorID(), size);

    if (passed)
      _ClearExclusiveLocal(_ProcessorID());

    return passed;
  }

  /* _IsExclusiveGlobal {{{4
   * ------------------
   * Checks if PE has marked in a global record an address range as an
   * "exclusive access requested" that covers at least the size bytes from
   * address.
   */
  bool _IsExclusiveGlobal(uint32_t addr, int processorID, int size) {
    auto lk = _gm.Lock();
    return _gm.IsExclusive(addr, processorID, size);
  }

  /* _IsExclusiveLocal {{{4
   * -----------------
   * Checks if PE has marked in a local record an address range as an
   * "exclusive access requested" that covers at least the size bytes from
   * address.
   */
  bool _IsExclusiveLocal(uint32_t addr, int processorID, int size) {
    return _lm.IsExclusive(addr, size);
  }

  /* _CountLeadingZeroBits {{{4
   * ---------------------
   */
  static uint32_t _CountLeadingZeroBits(uint32_t x) {
    return x ? __builtin_clz(x) : 32;
  }

  /* _IntegerZeroDivideTrappingEnabled {{{4
   * ---------------------------------
   */
  bool _IntegerZeroDivideTrappingEnabled() {
    // DIV_0_TRP bit in CCR is RAZ/WI if the Main Extension is not implememented
    return !!(InternalLoad32(REG_CCR) & REG_CCR__DIV_0_TRP);
  }

  /* _GenerateIntegerZeroDivide {{{4
   * --------------------------
   */
  void _GenerateIntegerZeroDivide() {
    InternalOr32(REG_CFSR, REG_CFSR__UFSR__DIVBYZERO);

    auto excInfo = _CreateException(UsageFault, false, UNKNOWN_VAL(false));
    _HandleException(excInfo);
  }

  /* _ExecuteCPCheck {{{4
   * ---------------
   */
  void _ExecuteCPCheck(int cp) {
    // Check access to coprocessor is enabled.
    auto excInfo = _CheckCPEnabled(cp);
    _HandleException(excInfo);
  }

  /* _GenerateCoprocessorException {{{4
   * -----------------------------
   */
  void _GenerateCoprocessorException() {
    InternalOr32(REG_CFSR, REG_CFSR__UFSR__UNDEFINSTR);
    auto excInfo = _CreateException(UsageFault, false, UNKNOWN_VAL(false));
    _HandleException(excInfo);
  }

  /* _Coproc_Accepted {{{4
   * ----------------
   */
  bool _Coproc_Accepted(int cpNum, uint32_t instr) {
    // TODO
    return false;
  }

  /* _Coproc_DoneLoading {{{4
   * -------------------
   * Check whether enough 32-bit words have been loaded for an LDC instruction.
   */
  bool _Coproc_DoneLoading(int cpNum, uint32_t instr) {
    // TODO
    return true;
  }

  /* _Coproc_DoneStoring {{{4
   * -------------------
   * Check whether enough 32-bit words have been stored for a STC instruction.
   */
  bool _Coproc_DoneStoring(int cpNum, uint32_t instr) {
    // TODO
    return true;
  }

  /* _Coproc_GetOneWord {{{4
   * ------------------
   * Gets the 32-bit word for an MRC instruction from the coprocessor.
   */
  uint32_t _Coproc_GetOneWord(int cpNum, uint32_t instr) {
    return 0xFFFF'FFFF; // TODO
  }

  /* _Coproc_GetTwoWords {{{4
   * -------------------
   * Get two 32-bit words for an MRRC instruction from the coprocessor.
   */
  std::tuple<uint32_t, uint32_t> _Coproc_GetTwoWords(int cpNum, uint32_t instr) {
    return {0xFFFF'FFFF, 0xFFFF'FFFF}; // TODO
  }

  /* _Coproc_GetWordToStore {{{4
   * ----------------------
   * Gets the next 32-bit word to store for a STC instruction from the coprocessor.
   */
  uint32_t _Coproc_GetWordToStore(int cpNum, uint32_t instr) {
    return 0xFFFF'FFFF; // TODO
  }

  /* _Coproc_InternalOperation {{{4
   * -------------------------
   * Instructs a coprocessor to perform the internal operation requested by a
   * CDP instruction.
   */
  void _Coproc_InternalOperation(int cpNum, uint32_t instr) {
    // TODO
  }

  /* _Coproc_SendLoadedWord {{{4
   * ----------------------
   * Sends a loaded 32-bit word for an LDC instruction to the coprocessor.
   */
  void _Coproc_SendLoadedWord(uint32_t word, int cpNum, uint32_t instr) {
    // TODO
  }

  /* _Coproc_SendOneWord {{{4
   * -------------------
   * Sends the 32-bit word for an MCR instruction to the coprocessor.
   */
  void _Coproc_SendOneWord(uint32_t word, int cpNum, uint32_t instr) {
    // TODO
  }

  /* _Coproc_SendTwoWords {{{4
   * --------------------
   * Send two 32-bit words for an MCRR instruction to the coprocessor.
   */
  void _Coproc_SendTwoWords(uint32_t word2, uint32_t word1, int cpNum, uint32_t instr) {
    // TODO
  }

  /* _UnsignedSatQ {{{4
   * -------------
   */
  std::tuple<uint32_t, bool> _UnsignedSatQ(uint32_t i, int N) {
    uint32_t result;
    bool     saturated;

    if (i > (BIT(N)-1)) {
      result    = BIT(N)-1;
      saturated = true;
    } else if (i < 0) { // no-op
      result    = 0;
      saturated = true;
    } else {
      result    = i;
      saturated = false;
    }

    return {GETBITS(result, 0, N-1), saturated};
  }

  /* _SignedSatQ {{{4
   * -----------
   */
  std::tuple<int32_t, bool> _SignedSatQ(uint32_t i, int N) {
    uint32_t result;
    bool     saturated;

    if (int32_t(i) > (int32_t)(BIT(N-1)-1)) {
      result    = BIT(N-1)-1;
      saturated = true;
    } else if (int32_t(i) < -(int32_t)BIT(N-1)) {
      result    = -BIT(N-1);
      saturated = true;
    } else {
      result    = i;
      saturated = false;
    }

    return {GETBITS(result, 0, N-1), saturated};
  }

  /* _DecodeExecute {{{3
   * --------------
   * This function is not defined by the ISA definition and must be generated
   * from all of the instruction definitions. Our actual implementation is in
   * _DecodeExecute(16|32) and wrapped by this.
   */
  void _DecodeExecute(uint32_t instr, uint32_t pc, bool is16bit) {
    if (is16bit)
      _DecodeExecute16(instr, pc);
    else
      _DecodeExecute32(instr, pc);
  }

  /* Decode/Execute (16-Bit Instructions) {{{3
   * ====================================
   */

  /* _DecodeExecute16 {{{4
   * ----------------
   */
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
        _DecodeExecute16_010000(instr, pc);
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
        _DecodeExecute16_0101xx(instr, pc);
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
        _DecodeExecute16_1000xx(instr, pc);
        break;

      case 0b10'0100:
      case 0b10'0101:
      case 0b10'0110:
      case 0b10'0111:
        // Load/store (SP-relative)
        _DecodeExecute16_1001xx(instr, pc);
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
        _DecodeExecute16_1100xx(instr, pc);
        break;

      case 0b11'0100:
      case 0b11'0101:
      case 0b11'0110:
      case 0b11'0111:
        // Conditional branch, and supervisor call
        _DecodeExecute16_1101xx(instr, pc);
        break;

      // (see § C2.1: Top level T32 instruction encoding)
      case 0b11'1000:
      case 0b11'1001:
        // B — T2 variant
        _DecodeExecute16_11100(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_00xxxx {{{4
   * -----------------------
   */
  void _DecodeExecute16_00xxxx(uint32_t instr, uint32_t pc) {
    // Shift (immediate), add, subtract, move and compare
    uint32_t op0 = GETBITS(instr,13,13);
    uint32_t op1 = GETBITS(instr,11,12);
    uint32_t op2 = GETBITS(instr,10,10);

    switch ((op0<<2)|op1) {
      case 0b0'11:
        if (!op2) {
          // Add, subtract (three low registers)
          _DecodeExecute16_000110(instr, pc);
        } else {
          // Add, subtract (two low registers and immediate)
          _DecodeExecute16_000111(instr, pc);
        }
        break;

      case 0b0'00:
      case 0b0'01:
      case 0b0'10:
        // MOV (register) — T2 variant
        _DecodeExecute16_000xxx(instr, pc);
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

  /* _DecodeExecute16_000xxx {{{4
   * -----------------------
   */
  void _DecodeExecute16_000xxx(uint32_t instr, uint32_t pc) {
    // MOV (register) § C2.4.90 T2
    // ---- DECODE --------------------------------------------------
    uint32_t op   = GETBITS(instr,11,12);
    uint32_t imm5 = GETBITS(instr, 6,10);
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rd   = GETBITS(instr, 0, 2);

    ASSERT(op != 0b11);

    uint32_t  d = Rd;
    uint32_t  m = Rm;
    bool      setflags = !_InITBlock();
    auto [shiftT, shiftN] = _DecodeImmShift(op, imm5);

    if (!op && !imm5 && _InITBlock())
      CUNPREDICTABLE_UNDEFINED();

    TRACEI(MOV_reg, T2, "d=%u m=%u S=%u shiftT=%u shiftN=%u", d, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOV_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_000110 {{{4
   * -----------------------
   */
  void _DecodeExecute16_000110(uint32_t instr, uint32_t pc) {
    // Add, subtract (three low registers)
    uint32_t S = GETBITS(instr, 9, 9);

    if (!S) {
      // ADD (register)
      _DecodeExecute16_000110_0(instr, pc);
    } else {
      // SUB (register)
      _DecodeExecute16_000110_1(instr, pc);
    }
  }

  /* _DecodeExecute16_000110_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_000110_0(uint32_t instr, uint32_t pc) {
    // ADD (register) § C2.4.7 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 6, 8);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rd = GETBITS(instr, 0, 2);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(ADD_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_000110_1 {{{4
   * -------------------------
   */
  void _DecodeExecute16_000110_1(uint32_t instr, uint32_t pc) {
    // SUB (register) § C2.4.200 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 6, 8);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rd = GETBITS(instr, 0, 2);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(SUB_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_000111 {{{4
   * -----------------------
   */
  void _DecodeExecute16_000111(uint32_t instr, uint32_t pc) {
    // Add, subtract (two low registers and immediate)
    uint32_t S = GETBITS(instr, 9, 9);

    if (!S) {
      // ADD (immediate)
      _DecodeExecute16_000111_0(instr, pc);
    } else {
      // SUB (immediate)
      _DecodeExecute16_000111_1(instr, pc);
    }
  }

  /* _DecodeExecute16_000111_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_000111_0(uint32_t instr, uint32_t pc) {
    // ADD (immediate) § C2.4.5 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm3 = GETBITS(instr, 6, 8);
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rd   = GETBITS(instr, 0, 2);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !_InITBlock();
    uint32_t imm32    = _ZeroExtend(imm3, 32);

    TRACEI(ADD_imm, T1, "d=%u n=%u S=%u imm32=0x%x", d, n, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute16_000111_1 {{{4
   * -------------------------
   */
  void _DecodeExecute16_000111_1(uint32_t instr, uint32_t pc) {
    // SUB (immediate) § C2.4.198 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm3 = GETBITS(instr, 6, 8);
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rd   = GETBITS(instr, 0, 2);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !_InITBlock();
    uint32_t imm32    = _ZeroExtend(imm3, 32);

    TRACEI(SUB_imm, T1, "d=%u n=%u S=%u imm32=0x%x", d, n, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute16_00100x {{{4
   * -----------------------
   */
  void _DecodeExecute16_00100x(uint32_t instr, uint32_t pc) {
    // MOV (immediate) § C2.4.89 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t  d         = Rd;
    bool      setflags  = !_InITBlock();
    uint32_t  imm32     = _ZeroExtend(imm8, 32);
    bool      carry     = GETBITSM(_s.xpsr, XPSR__C);

    TRACEI(MOV_imm, T1, "d=%u S=%u imm32=0x%x carry=%u", d, setflags, imm32, carry);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOV_immediate(d, setflags, imm32, carry);
  }

  /* _DecodeExecute16_00101x {{{4
   * -----------------------
   */
  void _DecodeExecute16_00101x(uint32_t instr, uint32_t pc) {
    // CMP (immediate) § C2.4.30 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);

    TRACEI(CMP_imm, T1, "n=%u imm32=0x%x R[n]=0x%x", n, imm32, _GetR(n));

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMP_immediate(n, imm32);
  }

  /* _DecodeExecute16_00110x {{{4
   * -----------------------
   */
  void _DecodeExecute16_00110x(uint32_t instr, uint32_t pc) {
    // ADD (immediate) § C2.4.5 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rdn  = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    bool     setflags = !_InITBlock();
    uint32_t imm32    = _ZeroExtend(imm8, 32);

    TRACEI(ADD_imm, T2, "d/n=%u S=%u imm32=0x%x", d, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute16_00111x {{{4
   * -----------------------
   */
  void _DecodeExecute16_00111x(uint32_t instr, uint32_t pc) {
    // SUB (immediate) § C2.4.198 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rdn  = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    bool     setflags = !_InITBlock();
    uint32_t imm32    = _ZeroExtend(imm8, 32);

    TRACEI(SUB_imm, T2, "d=%u n=%u S=%u imm32=0x%x", d, n, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute16_001xxx {{{4
   * -----------------------
   */
  void _DecodeExecute16_001xxx(uint32_t instr, uint32_t pc) {
    uint32_t op   = GETBITS(instr,11,12);
    uint32_t Rd   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    switch (op) {
      case 0b00:
        // MOV (immediate)
        _DecodeExecute16_00100x(instr, pc);
        break;

      case 0b01:
        // CMP (immediate)
        _DecodeExecute16_00101x(instr, pc);
        break;

      case 0b10:
        // ADD (immediate)
        _DecodeExecute16_00110x(instr, pc);
        break;

      case 0b11:
        // SUB (immediate)
        _DecodeExecute16_00111x(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_010000 {{{4
   * -----------------------
   */
  void _DecodeExecute16_010000(uint32_t instr, uint32_t pc) {
    // Data processing (two low registers)
    uint32_t op = GETBITS(instr, 6, 9);

    switch (op) {
      case 0b0000:
        // AND (register)
        _DecodeExecute16_010000_0000(instr, pc);
        break;

      case 0b0001:
        // EOR (register)
        _DecodeExecute16_010000_0001(instr, pc);
        break;

      case 0b0010:
        // MOV, MOVS (register-shifted register) - Logical shift left variant
        _DecodeExecute16_010000_0xxx_MOVsh(instr, pc);
        break;

      case 0b0011:
        // MOV, MOVS (register-shifted register) — Logical shift right variant
        _DecodeExecute16_010000_0xxx_MOVsh(instr, pc);
        break;

      case 0b0100:
        // MOV, MOVS (register-shifted register) — Arithmetic shift right variant
        _DecodeExecute16_010000_0xxx_MOVsh(instr, pc);
        break;

      case 0b0101:
        // ADC (register)
        _DecodeExecute16_010000_0101(instr, pc);
        break;

      case 0b0110:
        // SBC (register)
        _DecodeExecute16_010000_0110(instr, pc);
        break;

      case 0b0111:
        // MOV, MOVS (register-shifted register) — Rotate right variant
        _DecodeExecute16_010000_0xxx_MOVsh(instr, pc);
        break;

      case 0b1000:
        // TST (register)
        _DecodeExecute16_010000_1000(instr, pc);
        break;

      case 0b1001:
        // RSB (immediate)
        _DecodeExecute16_010000_1001(instr, pc);
        break;

      case 0b1010:
        // CMP (register)
        _DecodeExecute16_010000_1010(instr, pc);
        break;

      case 0b1011:
        // CMN (register)
        _DecodeExecute16_010000_1011(instr, pc);
        break;

      case 0b1100:
        // ORR (register)
        _DecodeExecute16_010000_1100(instr, pc);
        break;

      case 0b1101:
        // MUL
        _DecodeExecute16_010000_1101(instr, pc);
        break;

      case 0b1110:
        // BIC (register)
        _DecodeExecute16_010000_1110(instr, pc);
        break;

      case 0b1111:
        // MVN (register)
        _DecodeExecute16_010000_1111(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_010000_0000 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_0000(uint32_t instr, uint32_t pc) {
    // AND (register) § C2.4.10 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rdn  = GETBITS(instr, 0, 2);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(AND_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_AND_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_0001 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_0001(uint32_t instr, uint32_t pc) {
    // EOR (register) § C2.4.37 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rdn  = GETBITS(instr, 0, 2);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(EOR_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_EOR_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_0xxx_MOVsh {{{4
   * ----------------------------------
   */
  void _DecodeExecute16_010000_0xxx_MOVsh(uint32_t instr, uint32_t pc) {
    // MOV, MOVS (register-shifted register) § C2.4.91 T1
    // ---- DECODE --------------------------------------------------
    uint32_t op   = GETBITS(instr, 6, 9);
    uint32_t Rs   = GETBITS(instr, 3, 5);
    uint32_t Rdm  = GETBITS(instr, 0, 2);

    ASSERT(op == 0b0010 || op == 0b0011 || op == 0b0100 || op == 0b0111);

    uint32_t d = Rdm;
    uint32_t m = Rdm;
    uint32_t s = Rs;
    bool setflags = !_InITBlock();
    SRType shiftT = _DecodeRegShift((GETBIT(op, 2)<<1) | GETBIT(op, 0));

    TRACEI(MOV_reg_shifted_reg, T1, "d=%u m=%u s=%u S=%u shiftT=%u", d, m, s, setflags, shiftT);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOV_MOVS_register_shifted_register(d, m, s, setflags, shiftT);
  }

  /* _DecodeExecute16_010000_0101 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_0101(uint32_t instr, uint32_t pc) {
    // ADC (register) § C2.4.2 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rdn  = GETBITS(instr, 0, 2);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(ADC_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADC_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_0110 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_0110(uint32_t instr, uint32_t pc) {
    // SBC (register) § C2.4.141 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rdn  = GETBITS(instr, 0, 2);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(SBC_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SBC_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_1000 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1000(uint32_t instr, uint32_t pc) {
    // TST (register) § C2.4.212 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 5);
    uint32_t Rn = GETBITS(instr, 0, 2);

    uint32_t n        = Rn;
    uint32_t m        = Rm;
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(TST_reg, T1, "n=%u m=%u shiftT=%u shiftN=%u", n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_TST_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_1001 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1001(uint32_t instr, uint32_t pc) {
    // RSB (immediate) § C2.4.135 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rd = GETBITS(instr, 0, 2);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !_InITBlock();
    uint32_t imm32    = 0;

    TRACEI(RSB_imm, T1, "d=%u n=%u S=%u imm32=0x%x", d, n, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_RSB_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute16_010000_1010 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1010(uint32_t instr, uint32_t pc) {
    // CMP (register) § C2.4.31 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 5);
    uint32_t Rn = GETBITS(instr, 0, 2);

    uint32_t n        = Rn;
    uint32_t m        = Rm;
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(CMP_reg, T1, "n=%u m=%u shiftT=%u shiftN=%u", n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMP_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_1011 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1011(uint32_t instr, uint32_t pc) {
    // CMN (register) § C2.4.29 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 5);
    uint32_t Rn = GETBITS(instr, 0, 2);

    uint32_t n        = Rn;
    uint32_t m        = Rm;
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(CMN_reg, T1, "n=%u m=%u shiftT=%u shiftN=%u", n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMN_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_1100 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1100(uint32_t instr, uint32_t pc) {
    // ORR (register) § C2.4.104 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rdn  = GETBITS(instr, 0, 2);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    auto     shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(ORR_reg, T1, "d/n=%u m=%u S=%u", d, m, setflags);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ORR_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_1101 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1101(uint32_t instr, uint32_t pc) {
    // MUL § C2.4.97 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rdm  = GETBITS(instr, 0, 2);

    uint32_t d        = Rdm;
    uint32_t n        = Rn;
    uint32_t m        = Rdm;
    bool     setflags = !_InITBlock();

    TRACEI(MUL, T1, "d=%u n=%u m=%u S=%u", d, n, m, setflags);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MUL(d, n, m, setflags);
  }

  /* _DecodeExecute16_010000_1110 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1110(uint32_t instr, uint32_t pc) {
    // BIC (register) § C2.4.19 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rdn  = GETBITS(instr, 0, 2);

    uint32_t d        = Rdn;
    uint32_t n        = Rdn;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(BIC_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_BIC_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010000_1111 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010000_1111(uint32_t instr, uint32_t pc) {
    // MVN (register) § C2.4.99 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm   = GETBITS(instr, 3, 5);
    uint32_t Rd   = GETBITS(instr, 0, 2);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    bool     setflags = !_InITBlock();
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(MVN_reg, T1, "d=%u m=%u S=%u shiftT=%u shiftN=%u", d, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MVN_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010001 {{{4
   * -----------------------
   */
  void _DecodeExecute16_010001(uint32_t instr, uint32_t pc) {
    // Special data instructions and branch and exchange
    uint32_t op0 = GETBITS(instr, 8, 9);
    switch (op0) {
      case 0b11:
        // Branch and exchange
        _DecodeExecute16_010001_11(instr, pc);
        break;

      default:
        // Add, subtract, compare, move (two high registers)
        _DecodeExecute16_010001_xx(instr, pc);
        break;
    }
  }

  /* _DecodeExecute16_010001_11 {{{4
   * --------------------------
   */
  void _DecodeExecute16_010001_11(uint32_t instr, uint32_t pc) {
    // Branch and exchange
    uint32_t L = GETBITS(instr, 7, 7);

    if (!L) {
      // BX, BXNS
      return _DecodeExecute16_010001_11_0(instr, pc);
    } else {
      // BLX, BLXNS
      return _DecodeExecute16_010001_11_1(instr, pc);
    }
  }

  /* _DecodeExecute16_010001_11_0 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010001_11_0(uint32_t instr, uint32_t pc) {
    // BX, BXNS § C2.4.23 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 6);
    uint32_t NS = GETBITS(instr, 2, 2);

    CHECK01(BITS(0,1), 0);

    uint32_t m              = Rm;
    bool     allowNonSecure = !!NS;

    if (!_IsSecure() && allowNonSecure)
      THROW_UNDEFINED();

    if (m == 13 || m == 15)
      THROW_UNPREDICTABLE();

    if (_InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(BX, T1, "m=%u allowNonSecure=%u R[m]=0x%x", m, allowNonSecure, _GetR(m));

    // ---- EXECUTE -------------------------------------------------
    _Exec_BX(m, allowNonSecure);
  }

  /* _DecodeExecute16_010001_11_1 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010001_11_1(uint32_t instr, uint32_t pc) {
    // BLX, BLXNS § C2.4.22 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 6);
    uint32_t NS = GETBITS(instr, 2, 2);

    CHECK01(BITS(0,1), 0);

    uint32_t m = Rm;
    bool allowNonSecure = !!NS;

    if (!_IsSecure() && allowNonSecure)
      THROW_UNDEFINED();

    if (m == 13 || m == 15)
      THROW_UNPREDICTABLE();

    if (_InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(BLX, T1, "m=%u allowNonSecure=%u", m, allowNonSecure);

    // ---- EXECUTE -------------------------------------------------
    _Exec_BLX(m, allowNonSecure);
  }

  /* _DecodeExecute16_010001_10 {{{4
   * --------------------------
   */
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
        THROW_UNPREDICTABLE();

    TRACEI(MOV_reg, T1, "d=%u m=%u", d, m);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOV_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010001_xx {{{4
   * --------------------------
   */
  void _DecodeExecute16_010001_xx(uint32_t instr, uint32_t pc) {
    // Add, subtract, compare, move (two high registers)
    uint32_t op   = GETBITS(instr, 8, 9);
    uint32_t D    = GETBITS(instr, 7, 7);
    uint32_t Rs   = GETBITS(instr, 3, 6);
    uint32_t Rd   = GETBITS(instr, 0, 2);
    uint32_t D_Rd = (D<<3) | Rd;

    switch (op) {
      case 0b00:
        if (Rs == 0b1101) {
          // ADD (SP plus register) - T1
          _DecodeExecute16_010001_00_a(instr, pc);
        } else if (D_Rd == 0b1101) {
          // ADD (SP plus register) — T2
          _DecodeExecute16_010001_00_b(instr, pc);
        } else {
          // ADD (register)
          _DecodeExecute16_010001_00_c(instr, pc);
        }
        break;

      case 0b01:
        // CMP (register)
        _DecodeExecute16_010001_01(instr, pc);
        break;

      case 0b10:
        // MOV (register)
        _DecodeExecute16_010001_10(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_010001_00_a {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010001_00_a(uint32_t instr, uint32_t pc) {
    // ADD (SP plus register) § C2.4.4 T1
    // ---- DECODE --------------------------------------------------
    uint32_t DM     = GETBITS(instr, 7, 7);
    uint32_t Rdm    = GETBITS(instr, 0, 2);
    uint32_t DM_Rdm = (DM<<3) | Rdm;

    uint32_t d        = DM_Rdm;
    uint32_t m        = DM_Rdm;
    bool     setflags = false;

    if (d == 15 && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(ADD_SP_plus_reg, T1, "d=%u m=%u S=%u shiftT=%u shiftN=%u", d, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_SP_plus_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010001_00_b {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010001_00_b(uint32_t instr, uint32_t pc) {
    // ADD (SP plus register) § C2.4.4 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rm     = GETBITS(instr, 3, 6);

    ASSERT(Rm != 0b1101);

    uint32_t d        = 13;
    uint32_t m        = Rm;
    bool     setflags = false;
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    TRACEI(ADD_SP_plus_reg, T2, "d=%u m=%u S=%u shiftT=%u shiftN=%u", d, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_SP_plus_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010001_00_c {{{4
   * ----------------------------
   */
  void _DecodeExecute16_010001_00_c(uint32_t instr, uint32_t pc) {
    // ADD (register) § C2.4.7 T2
    // ---- DECODE --------------------------------------------------
    uint32_t DN   = GETBITS(instr, 7, 7);
    uint32_t Rm   = GETBITS(instr, 3, 6);
    uint32_t Rdn  = GETBITS(instr, 0, 2);

    uint32_t DN_Rdn = (DN<<3) | Rdn;

    ASSERT(!(DN_Rdn == 0b1101 || Rm == 0b1101));

    uint32_t d        = DN_Rdn;
    uint32_t n        = DN_Rdn;
    uint32_t m        = Rm;
    bool     setflags = false;
    SRType   shiftT   = SRType_LSL;
    uint32_t shiftN   = 0;

    if (d == 15 && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    if (d == 15 && m == 15)
      THROW_UNPREDICTABLE();

    TRACEI(ADD_reg, T2, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute16_010001_01 {{{4
   * --------------------------
   */
  void _DecodeExecute16_010001_01(uint32_t instr, uint32_t pc) {
    // CMP (register) § C2.4.31 T2
    // ---- DECODE --------------------------------------------------
    uint32_t N    = GETBITS(instr, 7, 7);
    uint32_t Rm   = GETBITS(instr, 3, 6);
    uint32_t Rn   = GETBITS(instr, 0, 2);
    uint32_t N_Rn = (N<<3) | Rn;

    uint32_t n      = N_Rn;
    uint32_t m      = Rm;
    SRType   shiftT = SRType_LSL;
    uint32_t shiftN = 0;

    if (n < 8 && m < 8)
      CUNPREDICTABLE_UNDEFINED();

    if (n == 15 || m == 15)
      THROW_UNPREDICTABLE();

    TRACEI(CMP_reg, T2, "n=%u m=%u shiftT=%u shiftN=%u", n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMP_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute16_01001x {{{4
   * -----------------------
   */
  void _DecodeExecute16_01001x(uint32_t instr, uint32_t pc) {
    // LDR (literal) § C2.4.53 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t t     = Rt;
    uint32_t imm32 = _ZeroExtend(imm8<<2, 32);
    bool     add   = true;

    TRACEI(LDR_lit, T1, "t=%u imm32=0x%x", t, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_literal(t, imm32, add);
  }

  /* _DecodeExecute16_0101xx {{{4
   * -----------------------
   */
  void _DecodeExecute16_0101xx(uint32_t instr, uint32_t pc) {
    // Load/store (register offset)
    uint32_t L = GETBITS(instr,11,11);
    uint32_t B = GETBITS(instr,10,10);
    uint32_t H = GETBITS(instr, 9, 9);

    uint32_t L_B_H = (L<<2) | (B<<1) | H;

    switch (L_B_H) {
      case 0b0'0'0:
        // STR (register)
        _DecodeExecute16_010100_0(instr, pc);
        break;

      case 0b0'0'1:
        // STRH (register)
        _DecodeExecute16_010100_1(instr, pc);
        break;

      case 0b0'1'0:
        // STRB (register)
        _DecodeExecute16_010101_0(instr, pc);
        break;

      case 0b0'1'1:
        // LDRSB (register)
        _DecodeExecute16_010101_1(instr, pc);
        break;

      case 0b1'0'0:
        // LDR (register)
        _DecodeExecute16_010110_0(instr, pc);
        break;

      case 0b1'0'1:
        // LDRH (register)
        _DecodeExecute16_010110_1(instr, pc);
        break;

      case 0b1'1'0:
        // LDRB (register)
        _DecodeExecute16_010111_0(instr, pc);
        break;

      case 0b1'1'1:
        // LDRSH (register)
        _DecodeExecute16_010111_1(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_010100_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010100_0(uint32_t instr, uint32_t pc) {
    // STR (register) § C2.4.184 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 6, 8);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rt = GETBITS(instr, 0, 2);

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    auto      shiftT = SRType_LSL;
    uint32_t  shiftN = 0;

    TRACEI(STR_reg, T1, "t=%u n=%u m=%u", t, n, m);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STR_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_010100_1 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010100_1(uint32_t instr, uint32_t pc) {
    // STRH (register) § C2.4.193 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 6, 8);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rt = GETBITS(instr, 0, 2);

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    auto      shiftT = SRType_LSL;
    uint32_t  shiftN = 0;

    TRACEI(STRH_reg, T1, "t=%u n=%u m=%u", t, n, m);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRH_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_010101_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010101_0(uint32_t instr, uint32_t pc) {
    // STRB (register) § C2.4.186 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt = GETBITS(instr, 0, 2);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rm = GETBITS(instr, 6, 8);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    auto     shiftT = SRType_LSL;
    uint32_t shiftN = 0;

    TRACEI(STRB_reg, T1, "t=%u n=%u m=%u index=%u add=%u wback=%u shiftT=%u shiftN=%u", t, n, m, index, add, wback, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRB_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_010101_1 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010101_1(uint32_t instr, uint32_t pc) {
    // LDRSB (register) § C2.4.70 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt = GETBITS(instr, 0, 2);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rm = GETBITS(instr, 6, 8);

    CHECK01(BIT(15), 0);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    auto     shiftT = SRType_LSL;
    uint32_t shiftN = 0;

    TRACEI(LDRSB_reg, T1, "t=%u n=%u m=%u index=%u add=%u wback=%u shiftT=%u shiftN=%u", t, n, m, index, add, wback, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSB_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_010110_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010110_0(uint32_t instr, uint32_t pc) {
    // LDR (register) § C2.4.54 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt = GETBITS(instr, 0, 2);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rm = GETBITS(instr, 6, 8);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    auto     shiftT = SRType_LSL;
    uint32_t shiftN = 0;

    TRACEI(LDR_reg, T1, "t=%u n=%u m=%u index=%u add=%u wback=%u shiftT=%u shiftN=%u", t, n, m, index, add, wback, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_010110_1 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010110_1(uint32_t instr, uint32_t pc) {
    // LDRH (register) § C2.4.66 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt = GETBITS(instr, 0, 2);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rm = GETBITS(instr, 6, 8);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    auto     shiftT = SRType_LSL;
    uint32_t shiftN = 0;

    TRACEI(LDRH_reg, T1, "t=%u n=%u m=%u index=%u add=%u wback=%u shiftT=%u shiftN=%u", t, n, m, index, add, wback, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRH_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_010111_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010111_0(uint32_t instr, uint32_t pc) {
    // LDRB (register) § C2.4.57 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt = GETBITS(instr, 0, 2);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rm = GETBITS(instr, 6, 8);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    auto     shiftT = SRType_LSL;
    uint32_t shiftN = 0;

    TRACEI(LDRB_reg, T1, "t=%u n=%u m=%u index=%u add=%u wback=%u shiftT=%u shiftN=%u", t, n, m, index, add, wback, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRB_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_010111_1 {{{4
   * -------------------------
   */
  void _DecodeExecute16_010111_1(uint32_t instr, uint32_t pc) {
    // LDRSH (register) § C2.4.74 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt = GETBITS(instr, 0, 2);
    uint32_t Rn = GETBITS(instr, 3, 5);
    uint32_t Rm = GETBITS(instr, 6, 8);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    auto     shiftT = SRType_LSL;
    uint32_t shiftN = 0;

    TRACEI(LDRSH_reg, T1, "t=%u n=%u m=%u index=%u add=%u wback=%u shiftT=%u shiftN=%u", t, n, m, index, add, wback, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSH_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute16_011xxx {{{4
   * -----------------------
   */
  void _DecodeExecute16_011xxx(uint32_t instr, uint32_t pc) {
    // Load/store word/byte (immediate offset)
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
        _DecodeExecute16_01110x(instr, pc);
        break;

      case 0b11:
        // LDRB (immediate)
        _DecodeExecute16_01111x(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_01110x {{{4
   * -----------------------
   */
  void _DecodeExecute16_01110x(uint32_t instr, uint32_t pc) {
    // STRB (immediate) § C2.4.185 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm5 = GETBITS(instr, 6,10);
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rt   = GETBITS(instr, 0, 2);

    uint32_t t     = Rt;
    uint32_t n     = Rn;
    uint32_t imm32 = _ZeroExtend(imm5, 32);
    bool     index = true, add = true, wback = false;

    TRACEI(STRB_imm, T1, "t=%u n=%u imm32=0x%x index=%u add=%u wback=%u", t, n, imm32, index, add, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_01111x {{{4
   * -----------------------
   */
  void _DecodeExecute16_01111x(uint32_t instr, uint32_t pc) {
    // LDRB (immediate) § C2.4.55 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm5   = GETBITS(instr, 6,10);
    uint32_t Rn     = GETBITS(instr, 3, 5);
    uint32_t Rt     = GETBITS(instr, 0, 2);

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm5, 32);
    bool     index  = true, add = true, wback = false;

    TRACEI(LDRB_imm, T1, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_01100x {{{4
   * -----------------------
   */
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

    TRACEI(STR_imm, T1, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STR_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_01101x {{{4
   * -----------------------
   */
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

    TRACEI(LDR_imm, T1, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_1000xx {{{4
   * -----------------------
   */
  void _DecodeExecute16_1000xx(uint32_t instr, uint32_t pc) {
    // Load/store halfword (immediate offset)
    uint32_t L = GETBITS(instr,11,11);

    if (!L) {
      // STRH (immediate)
      _DecodeExecute16_10000x(instr, pc);
    } else {
      // LDRH (immediate)
      _DecodeExecute16_10001x(instr, pc);
    }
  }

  /* _DecodeExecute16_10000x {{{4
   * -----------------------
   */
  void _DecodeExecute16_10000x(uint32_t instr, uint32_t pc) {
    // STRH (immediate) § C2.4.192 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm5 = GETBITS(instr, 6,10);
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rt   = GETBITS(instr, 0, 2);

    uint32_t  t     = Rt;
    uint32_t  n     = Rn;
    uint32_t  imm32 = _ZeroExtend(imm5<<1, 32);
    bool      index = true, add = true, wback = false;

    TRACEI(STRH_imm, T1, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_10001x {{{4
   * -----------------------
   */
  void _DecodeExecute16_10001x(uint32_t instr, uint32_t pc) {
    // LDRH (immediate) § C2.4.64 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm5 = GETBITS(instr, 6,10);
    uint32_t Rn   = GETBITS(instr, 3, 5);
    uint32_t Rt   = GETBITS(instr, 0, 2);

    uint32_t  t     = Rt;
    uint32_t  n     = Rn;
    uint32_t  imm32 = _ZeroExtend(imm5<<1, 32);
    bool      index = true, add = true, wback = false;

    TRACEI(LDRH_imm, T1, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_1001xx {{{4
   * -----------------------
   */
  void _DecodeExecute16_1001xx(uint32_t instr, uint32_t pc) {
    // Load/store (SP-relative)
    uint32_t L = GETBITS(instr,11,11);

    if (!L) {
      // STR (immediate)
      _DecodeExecute16_10010x(instr, pc);
    } else {
      // LDR (immediate)
      _DecodeExecute16_10011x(instr, pc);
    }
  }

  /* _DecodeExecute16_10010x {{{4
   * -----------------------
   */
  void _DecodeExecute16_10010x(uint32_t instr, uint32_t pc) {
    // STR (immediate) § C2.4.183 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rt   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t t     = Rt;
    uint32_t n     = 13;
    uint32_t imm32 = _ZeroExtend(imm8<<2, 32);
    bool     index = true;
    bool     add   = true;
    bool     wback = false;

    TRACEI(STR_imm, T2, "t=%u n=%u imm32=0x%x index=%u add=%u wback=%u", t, n, imm32, index, add, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STR_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_10011x {{{4
   * -----------------------
   */
  void _DecodeExecute16_10011x(uint32_t instr, uint32_t pc) {
    // LDR (immediate) § C2.4.52 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rt   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t t     = Rt;
    uint32_t n     = 13;
    uint32_t imm32 = _ZeroExtend(imm8<<2, 32);
    bool     index = true;
    bool     add   = true;
    bool     wback = false;

    TRACEI(LDR_imm, T2, "t=%u n=%u imm32=0x%x index=%u add=%u wback=%u", t, n, imm32, index, add, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute16_1010xx {{{4
   * -----------------------
   */
  void _DecodeExecute16_1010xx(uint32_t instr, uint32_t pc) {
    // Add PC/SP (immediate)
    if (!(instr & BIT(11)))
      // ADR
      _DecodeExecute16_1010xx_0(instr, pc);
    else
      // ADD (SP plus immediate)
      _DecodeExecute16_1010xx_1(instr, pc);
  }

  /* _DecodeExecute16_1010xx_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_1010xx_0(uint32_t instr, uint32_t pc) {
    // ADR § C2.4.8 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t d    = Rd, imm32 = _ZeroExtend(imm8<<2, 32);
    bool     add  = true;

    TRACEI(ADR, T1, "d=%u imm32=0x%x", d, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADR(d, imm32, add);
  }

  /* _DecodeExecute16_1010xx_1 {{{4
   * -------------------------
   */
  void _DecodeExecute16_1010xx_1(uint32_t instr, uint32_t pc) {
    // ADD (SP plus immediate) § C2.4.3 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd   = GETBITS(instr, 8,10);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t d        = Rd;
    bool     setflags = false;
    uint32_t imm32    = _ZeroExtend(imm8<<2, 32);

    TRACEI(ADD_SP_plus_imm, T1, "d=%u S=%u imm32=0x%x", d, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_SP_plus_immediate(d, setflags, imm32);
  }

  /* _DecodeExecute16_1011xx {{{4
   * -----------------------
   */
  void _DecodeExecute16_1011xx(uint32_t instr, uint32_t pc) {
    // Miscellaneous 16-bit instructions
    uint32_t op0 = GETBITS(instr, 8,11);
    uint32_t op1 = GETBITS(instr, 6, 7);
    uint32_t op2 = GETBITS(instr, 5, 5);
    uint32_t op3 = GETBITS(instr, 0, 3);

    switch (op0) {
      case 0b0000:
        // Adjust SP (immediate)
        _DecodeExecute16_101100_00(instr, pc);
        break;

      case 0b0010:
        // Extend
        _DecodeExecute16_101100_10(instr, pc);
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
          _DecodeExecute16_101110_10(instr, pc);
        }
        break;

      case 0b1110:
        // BKPT
        _DecodeExecute16_101111_10(instr, pc);
        break;

      case 0b1111:
        if (!op3) {
          // Hints
          _DecodeExecute16_101111_11_0000(instr, pc);
        } else {
          // IT
          _DecodeExecute16_101111_11_xxxx(instr, pc);
        }
        break;

      case 0b0001:
      case 0b0011:
      case 0b1001:
      case 0b1011:
        // CBNZ, CBZ
        _DecodeExecute16_1011x0_xx(instr, pc);
        break;

      case 0b0100:
      case 0b0101:
      case 0b1100:
      case 0b1101:
        // Push and Pop
        _DecodeExecute16_1011x1_0(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_1011x0_xx {{{4
   * --------------------------
   */
  void _DecodeExecute16_1011x0_xx(uint32_t instr, uint32_t pc) {
    // CBNZ, CBZ § C2.4.24 T1
    // ---- DECODE --------------------------------------------------
    uint32_t op   = GETBITS(instr,11,11);
    uint32_t i    = GETBITS(instr, 9, 9);
    uint32_t imm5 = GETBITS(instr, 3, 7);
    uint32_t Rn   = GETBITS(instr, 0, 2);

    uint32_t n        = Rn;
    uint32_t imm32    = _ZeroExtend((i << 6) | (imm5 << 1), 32);
    bool     nonzero  = !!op;

    if (_InITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(CBNZ_CBZ, T1, "n=%u imm32=0x%x nonzero=%u", n, imm32, nonzero);

    // ---- EXECUTE -------------------------------------------------
    _Exec_CBNZ_CBZ(n, imm32, nonzero);
  }

  /* _DecodeExecute16_101111_10 {{{4
   * --------------------------
   */
  void _DecodeExecute16_101111_10(uint32_t instr, uint32_t pc) {
    // BKPT § C2.4.20 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t imm32 = _ZeroExtend(imm8, 32);
    // imm32 is for assembly/disassembly only and is ignored by hardware.

    TRACEI(BKPT, T1, "imm32=0x%x", imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_BKPT();
  }

  /* _DecodeExecute16_101110_10 {{{4
   * --------------------------
   */
  void _DecodeExecute16_101110_10(uint32_t instr, uint32_t pc) {
    // Reverse bytes
    uint32_t op = GETBITS(instr, 6, 7);

    switch (op) {
      case 0b00:
        // REV
        _DecodeExecute16_101110_10_00(instr, pc);
        break;

      case 0b01:
        // REV16
        _DecodeExecute16_101110_10_01(instr, pc);
        break;

      case 0b11:
        // REVSH
        _DecodeExecute16_101110_10_11(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_101110_10_00 {{{4
   * -----------------------------
   */
  void _DecodeExecute16_101110_10_00(uint32_t instr, uint32_t pc) {
    // REV § C2.4.126 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 5);
    uint32_t Rd = GETBITS(instr, 0, 2);

    uint32_t d = Rd;
    uint32_t m = Rm;

    TRACEI(REV, T1, "d=%u m=%u", d, m);

    // ---- EXECUTE -------------------------------------------------
    _Exec_REV(d, m);
  }

  /* _DecodeExecute16_101110_10_01 {{{4
   * -----------------------------
   */
  void _DecodeExecute16_101110_10_01(uint32_t instr, uint32_t pc) {
    // REV16 § C2.4.127 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 5);
    uint32_t Rd = GETBITS(instr, 0, 2);

    uint32_t d = Rd;
    uint32_t m = Rm;

    TRACEI(REV16, T1, "d=%u m=%u", d, m);

    // ---- EXECUTE -------------------------------------------------
    _Exec_REV16(d, m);
  }

  /* _DecodeExecute16_101110_10_11 {{{4
   * -----------------------------
   */
  void _DecodeExecute16_101110_10_11(uint32_t instr, uint32_t pc) {
    // REVSH § C2.4.128 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm = GETBITS(instr, 3, 5);
    uint32_t Rd = GETBITS(instr, 0, 2);

    uint32_t d = Rd;
    uint32_t m = Rm;

    TRACEI(REVSH, T1, "d=%u m=%u", d, m);

    // ---- EXECUTE -------------------------------------------------
    _Exec_REVSH(d, m);
  }

  /* _DecodeExecute16_101100_10 {{{4
   * --------------------------
   */
  void _DecodeExecute16_101100_10(uint32_t instr, uint32_t pc) {
    // Extend
    uint32_t U = GETBITS(instr, 7, 7);
    uint32_t B = GETBITS(instr, 6, 6);
    uint32_t U_B = (U<<1) | B;

    switch (U_B) {
      case 0b0'0:
        // SXTH
        _DecodeExecute16_101100_10_00(instr, pc);
        break;

      case 0b0'1:
        // SXTB
        _DecodeExecute16_101100_10_01(instr, pc);
        break;

      case 0b1'0:
        // UXTH
        _DecodeExecute16_101100_10_10(instr, pc);
        break;

      case 0b1'1:
        // UXTB
        _DecodeExecute16_101100_10_11(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_101100_10_00 {{{4
   * -----------------------------
   */
  void _DecodeExecute16_101100_10_00(uint32_t instr, uint32_t pc) {
    // SXTH § C2.4.207 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd = GETBITS(instr, 0, 2);
    uint32_t Rm = GETBITS(instr, 3, 5);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = 0;

    TRACEI(SXTH, T1, "d=%u m=%u rotation=%u", d, m, rotation);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SXTH(d, m, rotation);
  }

  /* _DecodeExecute16_101100_10_01 {{{4
   * -----------------------------
   */
  void _DecodeExecute16_101100_10_01(uint32_t instr, uint32_t pc) {
    // SXTB § C2.4.205 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd = GETBITS(instr, 0, 2);
    uint32_t Rm = GETBITS(instr, 3, 5);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = 0;

    TRACEI(SXTB, T1, "d=%u m=%u rotation=%u", d, m, rotation);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SXTB(d, m, rotation);
  }

  /* _DecodeExecute16_101100_10_10 {{{4
   * -----------------------------
   */
  void _DecodeExecute16_101100_10_10(uint32_t instr, uint32_t pc) {
    // UXTH § C2.4.247 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd = GETBITS(instr, 0, 2);
    uint32_t Rm = GETBITS(instr, 3, 5);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = 0;

    TRACEI(UXTH, T1, "d=%u m=%u rotation=%u", d, m, rotation);

    // ---- EXECUTE -------------------------------------------------
    _Exec_UXTH(d, m, rotation);
  }

  /* _DecodeExecute16_101100_10_11 {{{4
   * -----------------------------
   */
  void _DecodeExecute16_101100_10_11(uint32_t instr, uint32_t pc) {
    // UXTB § C2.4.245 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd = GETBITS(instr, 0, 2);
    uint32_t Rm = GETBITS(instr, 3, 5);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = 0;

    TRACEI(UXTB, T1, "d=%u m=%u rotation=%u", d, m, rotation);

    // ---- EXECUTE -------------------------------------------------
    _Exec_UXTB(d, m, rotation);
  }

  /* _DecodeExecute16_1011x1_0 {{{4
   * -------------------------
   */
  void _DecodeExecute16_1011x1_0(uint32_t instr, uint32_t pc) {
    uint32_t L = GETBITS(instr,11,11);
    uint32_t P = GETBITS(instr, 8, 8);

    if (!L) {
      // STMDB, STMFD
      _DecodeExecute16_101101(instr, pc);
    } else {
      // LDM, LDMIA, LDMFD
      _DecodeExecute16_101111(instr, pc);
    }
  }

  /* _DecodeExecute16_101100_00 {{{4
   * --------------------------
   */
  void _DecodeExecute16_101100_00(uint32_t instr, uint32_t pc) {
    uint32_t S = GETBITS(instr, 7, 7);

    if (!S) {
      // ADD (SP plus immediate)
      _DecodeExecute16_101100_00_0(instr, pc);
    } else {
      // SUB (SP minus immediate)
      _DecodeExecute16_101100_00_1(instr, pc);
    }
  }

  /* _DecodeExecute16_101100_00_0 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_101100_00_0(uint32_t instr, uint32_t pc) {
    // ADD (SP plus immediate) § C2.4.3 T2
    // ---- DECODE --------------------------------------------------
    uint32_t imm7 = GETBITS(instr, 0, 6);

    uint32_t d        = 13;
    bool     setflags = false;
    uint32_t imm32    = _ZeroExtend(imm7<<2, 32);

    TRACEI(ADD_SP_plus_imm, T2, "d=%u imm32=0x%x oldSP=0x%x", d, imm32, _GetSP());

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_SP_plus_immediate(d, setflags, imm32);
  }

  /* _DecodeExecute16_101100_00_1 {{{4
   * ----------------------------
   */
  void _DecodeExecute16_101100_00_1(uint32_t instr, uint32_t pc) {
    // SUB (SP minus immediate) § C2.4.196 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm7 = GETBITS(instr, 0, 6);

    uint32_t d        = 13;
    bool     setflags = false;
    uint32_t imm32    = _ZeroExtend(imm7<<2, 32);

    TRACEI(SUB_SP_minus_imm, T1, "d=%u S=%u imm32=0x%x prevSP=0x%x", d, setflags, imm32, _GetSP());

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_SP_minus_immediate(d, setflags, imm32);
  }

  /* _DecodeExecute16_101101 {{{4
   * -----------------------
   */
  void _DecodeExecute16_101101(uint32_t instr, uint32_t pc) {
    // STMDB, STMFD § C2.4.182 T2
    // ---- DECODE --------------------------------------------------
    uint32_t M        = GETBITS(instr, 8, 8);
    uint32_t regList  = GETBITS(instr, 0, 7);

    uint32_t n          = 13;
    uint32_t registers  = regList | (M<<14);
    bool     wback      = true;

    if (_BitCount(registers) < 1)
      CUNPREDICTABLE_UNDEFINED();

    TRACEI(STMDB, T2, "n=%u registers=0x%x wback=%u", n, registers, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STMDB(n, registers, wback);
  }

  /* _DecodeExecute16_101101_10_01_1 {{{4
   * -------------------------------
   */
  void _DecodeExecute16_101101_10_01_1(uint32_t instr, uint32_t pc) {
    // CPS § C2.4.32 T1
    // ---- DECODE --------------------------------------------------
    uint32_t im = GETBITS(instr, 4, 4);
    uint32_t I  = GETBITS(instr, 1, 1);
    uint32_t F  = GETBITS(instr, 0, 0);

    CHECK01(BITS(2,3), 0);

    bool enable   = !im;
    bool disable  = !!im;
    if (_InITBlock())
      THROW_UNPREDICTABLE();

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

    TRACEIU(CPS, T1, "enable=%u disable=%u PRI=%u FAULT=%u", enable, disable, affectPRI, affectFAULT);

    // ---- EXECUTE -------------------------------------------------
    _Exec_CPS(enable, disable, affectPRI, affectFAULT);
  }

  /* _DecodeExecute16_101111 {{{4
   * -----------------------
   */
  void _DecodeExecute16_101111(uint32_t instr, uint32_t pc) {
    // LDM, LDMIA, LDMFD § C2.4.50 T3
    // ---- DECODE --------------------------------------------------
    uint32_t P        = GETBITS(instr, 8, 8);
    uint32_t regList  = GETBITS(instr, 0, 7);

    uint32_t n          = 13;
    uint32_t registers  = regList | (P<<15);
    bool     wback      = true;

    if (_BitCount(registers) < 1)
      CUNPREDICTABLE_UNDEFINED();

    if (GETBIT(registers,15) && _InITBlock() && !_LastInITBlock())
      THROW_UNDEFINED();

    TRACEI(LDM, T3, "n=%u wback=%u registers=0x%x", n, wback, registers);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDM(n, registers, wback);
  }

  /* _DecodeExecute16_101111_11_xxxx {{{4
   * -------------------------------
   */
  void _DecodeExecute16_101111_11_xxxx(uint32_t instr, uint32_t pc) {
    // IT § C2.4.41 T1
    // ---- DECODE --------------------------------------------------
    uint32_t firstCond = GETBITS(instr, 4, 7);
    uint32_t mask      = GETBITS(instr, 0, 3);

    ASSERT(mask != 0b0000);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    if (firstCond == 0b1111 || (firstCond == 0b1110 && _BitCount(mask) != 1))
      CUNPREDICTABLE_UNDEFINED();

    if (_InITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(IT, T1, "firstCond=0x%x mask=0x%x", firstCond, mask);

    // ---- EXECUTE -------------------------------------------------
    _Exec_IT(firstCond, mask);
  }

  /* _DecodeExecute16_101111_11_0000 {{{4
   * -------------------------------
   */
  void _DecodeExecute16_101111_11_0000(uint32_t instr, uint32_t pc) {
    uint32_t hint = GETBITS(instr, 4, 7);

    switch (hint) {
      case 0b0000:
        // NOP
        _DecodeExecute16_101111_11_0000_0000(instr, pc);
        break;

      case 0b0001:
        // YIELD
        _DecodeExecute16_101111_11_0000_0001(instr, pc);
        break;

      case 0b0010:
        // WFE
        _DecodeExecute16_101111_11_0000_0010(instr, pc);
        break;

      case 0b0011:
        // WFI
        _DecodeExecute16_101111_11_0000_0011(instr, pc);
        break;

      case 0b0100:
        // SEV
        _DecodeExecute16_101111_11_0000_0100(instr, pc);
        break;

      case 0b0101:
      case 0b0110:
      case 0b0111:
      case 0b1000:
      case 0b1001:
      case 0b1010:
      case 0b1011:
      case 0b1100:
      case 0b1101:
      case 0b1110:
      case 0b1111:
        // Reserved hint, behaves as NOP.
        _DecodeExecute16_101111_11_0000_xxxx(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute16_101111_11_0000_0000 {{{4
   * ------------------------------------
   */
  void _DecodeExecute16_101111_11_0000_0000(uint32_t instr, uint32_t pc) {
    // NOP § C2.4.100 T1
    // ---- DECODE --------------------------------------------------

    TRACEI(NOP, T1, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_NOP();
  }

  /* _DecodeExecute16_101111_11_0000_0001 {{{4
   * ------------------------------------
   */
  void _DecodeExecute16_101111_11_0000_0001(uint32_t instr, uint32_t pc) {
    // YIELD § C2.4.306 T1
    // ---- DECODE --------------------------------------------------

    TRACEI(YIELD, T1, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_YIELD();
  }

  /* _DecodeExecute16_101111_11_0000_0010 {{{4
   * ------------------------------------
   */
  void _DecodeExecute16_101111_11_0000_0010(uint32_t instr, uint32_t pc) {
    // WFE § C2.4.304 T1
    // ---- DECODE --------------------------------------------------

    TRACEI(WFE, T1, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_WFE();
  }

  /* _DecodeExecute16_101111_11_0000_0011 {{{4
   * ------------------------------------
   */
  void _DecodeExecute16_101111_11_0000_0011(uint32_t instr, uint32_t pc) {
    // WFI § C2.4.305 T1
    // ---- DECODE --------------------------------------------------

    TRACEI(WFI, T1, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_WFI();
  }

  /* _DecodeExecute16_101111_11_0000_0100 {{{4
   * ------------------------------------
   */
  void _DecodeExecute16_101111_11_0000_0100(uint32_t instr, uint32_t pc) {
    // SEV § C2.4.145 T1
    // ---- DECODE --------------------------------------------------

    CHECK01(BIT(11) | BIT(13), BITS(16+0,16+3));

    TRACEI(SEV, T1, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_SEV();
  }

  /* _DecodeExecute16_101111_11_0000_xxxx {{{4
   * ------------------------------------
   */
  void _DecodeExecute16_101111_11_0000_xxxx(uint32_t instr, uint32_t pc) {
    // Reserved hint, behaves as NOP.
    // ---- DECODE --------------------------------------------------

    TRACEI(RSVD_HINT, UNK, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_NOP();
  }

  /* _DecodeExecute16_1100xx {{{4
   * -----------------------
   */
  void _DecodeExecute16_1100xx(uint32_t instr, uint32_t pc) {
    uint32_t L = GETBITS(instr,11,11);

    if (!L) {
      // STM, STMIA, STMEA
      _DecodeExecute16_11000x(instr, pc);
    } else {
      // LDM, LDMIA, LDMFD
      _DecodeExecute16_11001x(instr, pc);
    }
  }

  /* _DecodeExecute16_11000x {{{4
   * -----------------------
   */
  void _DecodeExecute16_11000x(uint32_t instr, uint32_t pc) {
    // STM, STMIA, STMEA § C2.4.181 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn       = GETBITS(instr, 8,10);
    uint32_t regList  = GETBITS(instr, 0, 7);

    uint32_t n          = Rn;
    uint32_t registers  = regList;
    bool     wback      = true;

    if (_BitCount(registers) < 1)
      CUNPREDICTABLE_UNDEFINED();

    TRACEI(STM, T1, "n=%u registers=0x%x", n, registers);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STM(n, registers, wback);
  }

  /* _DecodeExecute16_11001x {{{4
   * -----------------------
   */
  void _DecodeExecute16_11001x(uint32_t instr, uint32_t pc) {
    // LDM, LDMIA, LDMFD § C2.4.50 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn       = GETBITS(instr, 8,10);
    uint32_t regList  = GETBITS(instr, 0, 7);

    CHECK01(BIT(13), 0);

    uint32_t n          = Rn;
    uint32_t registers  = regList;
    bool     wback      = !GETBIT(registers, n);

    if (_BitCount(registers) < 1)
      CUNPREDICTABLE_UNDEFINED();

    TRACEI(LDM, T1, "n=%u registers=0x%x wback=%u", n, registers, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDM(n, registers, wback);
  }

  /* _DecodeExecute16_1101xx {{{4
   * -----------------------
   */
  void _DecodeExecute16_1101xx(uint32_t instr, uint32_t pc) {
    // Conditional branch, and Supervisor Call
    uint32_t op0 = GETBITS(instr, 8,11);

    switch (op0) {
      case 0b1110:
      case 0b1111:
        // Exception generation
        _DecodeExecute16_110111_1x(instr, pc);
        break;

      default:
        // B - T1 variant
        _DecodeExecute16_1101xx_xx(instr, pc);
        break;
    }
  }

  /* _DecodeExecute16_110111_1x {{{4
   * --------------------------
   */
  void _DecodeExecute16_110111_1x(uint32_t instr, uint32_t pc) {
    // Exception generation
    uint32_t S = GETBITS(instr, 8, 8);

    if (!S) {
      // UDF
      _DecodeExecute16_110111_10(instr, pc);
    } else {
      // SVC
      _DecodeExecute16_110111_11(instr, pc);
    }
  }

  /* _DecodeExecute16_110111_10 {{{4
   * --------------------------
   */
  void _DecodeExecute16_110111_10(uint32_t instr, uint32_t pc) {
    // UDF § C2.4.218 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t imm32 = _ZeroExtend(imm8, 32);

    // imm32 is for assembly and disassembly only, and is ignored by hardware.

    TRACEI(UDF, T1, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_UDF();
  }

  /* _DecodeExecute32_110111_11 {{{4
   * --------------------------
   */
  void _DecodeExecute16_110111_11(uint32_t instr, uint32_t pc) {
    // SVC § C2.4.201 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm8 = GETBITS(instr, 0, 7);

    uint32_t imm32 = _ZeroExtend(imm8, 32);

    // imm32 is for assembly/disassebly. SVC handlers in some
    // systems interpret imm8 in software, for example to determine
    // the required service.

    TRACEI(SVC, T1, "imm32=0x%x", imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SVC();
  }

  /* _DecodeExecute16_1101xx_xx {{{4
   * --------------------------
   */
  void _DecodeExecute16_1101xx_xx(uint32_t instr, uint32_t pc) {
    // B § C2.4.15 T1
    // ---- DECODE --------------------------------------------------
    uint32_t cond = GETBITS(instr, 8,11);
    uint32_t imm8 = GETBITS(instr, 0, 7);

    ASSERT (cond != 0b1110 && cond != 0b1111);

    uint32_t imm32 = _SignExtend(imm8<<1, 9, 32);

    if (_InITBlock())
      THROW_UNPREDICTABLE();

    _s.curCondOverride = cond;
    TRACEI(B, T1, "imm32=0x%x cond=%u", imm32, cond);

    // ---- EXECUTE -------------------------------------------------
    _Exec_B(imm32);
  }

  /* _DecodeExecute16_11100 {{{4
   * ----------------------
   */
  void _DecodeExecute16_11100(uint32_t instr, uint32_t pc) {
    // B § C2.4.15 T2
    // ---- DECODE --------------------------------------------------
    uint32_t imm11 = GETBITS(instr, 0,10);

    uint32_t imm32 = _SignExtend(imm11<<1, 12, 32);
    if (_InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(B, T2, "imm32=0x%x", imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_B(imm32);
  }

  /* Decode/Execute (32-Bit Instructions) {{{3
   * ====================================
   */

  /* _DecodeExecute32 {{{4
   * ----------------
   */
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
        _DecodeExecute32_x11x(instr, pc);
        break;

      case 0b0100:
        // Load/store (multiple, dual, exclusive, acquire-release), table branch
        _DecodeExecute32_0100(instr, pc);
        break;

      case 0b0101:
        // Data-processing (shifted register)
        _DecodeExecute32_0101(instr, pc);
        break;

      case 0b1000:
      case 0b1001:
      case 0b1010:
      case 0b1011:
        if (op3) {
          // Branches and miscellaneous control
          _DecodeExecute32_10xx(instr, pc);
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
          UNDEFINED_DEC();
        } else {
          // Load/store single
          _DecodeExecute32_1100_xxxxx(instr, pc);
        }
        break;

      case 0b1101:
        if (!(op1 & BIT(4))) {
          // Data processing (register)
          _DecodeExecute32_1101_0xxxx(instr, pc);
        } else if (!(op1 & BIT(3))) {
          // Multiply, multiply accumulate, and absolute difference
          _DecodeExecute32_1101_10xxx(instr, pc);
        } else {
          // Long multiply and divide
          _DecodeExecute32_1101_11xxx(instr, pc);
        }
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_x11x {{{4
   * ---------------------
   */
  void _DecodeExecute32_x11x(uint32_t instr, uint32_t pc) {
    // Coprocessor and floating-point instructions
    uint32_t op0 = GETBITS(instr>>16, 8, 9);
    uint32_t op1 = GETBITS(instr    ,11,11);
    uint32_t op2 = GETBITS(instr    ,10,10);
    uint32_t op3 = GETBITS(instr    , 4, 4);

    if (op0 == 0b11) {
      // Unallocated
      UNDEFINED_DEC();
    } else if (!op1) {
      // Coprocessor
      _DecodeExecute32_x11x_0(instr, pc);
    } else switch (op0) {
      case 0b00:
      case 0b01:
        if (!op2) {
          // Floating-point load/store and 64-bit register moves
          TODO_DEC();
        } else {
          // Unallocated
          UNDEFINED_DEC();
        }
        break;

      case 0b10:
        switch ((op2<<1) | op3) {
          case 0b0'0:
            // Floating-point data processing
            TODO_DEC();
            break;

          case 0b0'1:
            // Floating-point 32-bit register moves
            TODO_DEC();
            break;

          case 0b1'0:
          case 0b1'1:
            // Unallocated
            UNDEFINED_DEC();
            break;

          default:
            abort();
        }
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_x11x_0 {{{4
   * -----------------------
   */
  //                    C  9
  void _DecodeExecute32_x11x_0(uint32_t instr, uint32_t pc) {
    // Coprocessor
    uint32_t op0 = GETBITS(instr>>16, 9, 9);
    uint32_t op1 = GETBITS(instr>>16, 5, 8);
    uint32_t op2 = GETBITS(instr    , 4, 4);

    if (!op0) {
      if ((op1 & 0b1101) == 0) {
        // Coprocessor 64-bit move
        _DecodeExecute32_x110_0_00x0(instr, pc);
      } else {
        // Coprocessor load/store registers
        _DecodeExecute32_x110_0_xxxx(instr, pc);
      }
    } else {
      if (!(op1 & BIT(3))) {
        if (!op2) {
          // CDP, CDP2
          _DecodeExecute32_CDP_CDP2_T1_T2(instr, pc);
        } else {
          // Coprocessor 32-bit move
          _DecodeExecute32_x111_0_0xxx_1(instr, pc);
        }
      } else {
        // ?
        UNDEFINED_DEC();
      }
    }
  }

  /* _DecodeExecute32_x110_0_xxxx {{{4
   * ----------------------------
   */
  void _DecodeExecute32_x110_0_xxxx(uint32_t instr, uint32_t pc) {
    // Coprocessor load/store registers
    uint32_t o0 = GETBITS(instr>>16,12,12);
    uint32_t P  = GETBITS(instr>>16, 8, 8);
    uint32_t U  = GETBITS(instr>>16, 7, 7);
    uint32_t D  = GETBITS(instr>>16, 6, 6);
    uint32_t W  = GETBITS(instr>>16, 5, 5);
    uint32_t L  = GETBITS(instr>>16, 4, 4);
    uint32_t Rn = GETBITS(instr>>16, 0, 3);

    uint32_t P_U_W = (P<<2) | (U<<1) | W;

    if (!L)
      switch (P_U_W) {
        case 0b001:
        case 0b011:
        case 0b010:
        case 0b100:
        case 0b110:
        case 0b101:
        case 0b111:
          // STC, STC2
          _DecodeExecute32_STC_STC2_T1_T2(instr, pc);
          break;

        default:
          // ?
          UNDEFINED_DEC();
          break;
      }
    else
      if (Rn == 0b1111) {
        if (P_U_W) {
          // ?
          UNDEFINED_DEC();
        } else if (!o0) {
          // LDC, LDC2 (literal) — T1
          _DecodeExecute32_LDC_LDC2_literal_T1_T2(instr, pc);
        } else {
          // LDC, LDC2 (literal) — T2
          _DecodeExecute32_LDC_LDC2_literal_T1_T2(instr, pc);
        }
      } else switch (P_U_W) {
        case 0b001:
        case 0b011:
        case 0b010:
        case 0b100:
        case 0b110:
        case 0b101:
        case 0b111:
          // LDC, LDC2 (immediate)
          _DecodeExecute32_LDC_LDC2_immediate_T1_T2(instr, pc);
          break;

        default:
          // ?
          UNDEFINED_DEC();
          break;
      }
  }

  /* _DecodeExecute32_LDC_LDC2_literal_T1_T2 {{{4
   * ---------------------------------------
   */
  void _DecodeExecute32_LDC_LDC2_literal_T1_T2(uint32_t instr, uint32_t pc) {
    // LDC, LDC2 (literal) § C2.4.49 T1/T2
    // ---- DECODE --------------------------------------------------
    uint32_t P      = GETBITS(instr>>16, 8, 8);
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t D      = GETBITS(instr>>16, 6, 6);
    uint32_t W      = GETBITS(instr>>16, 5, 5);
    uint32_t CRd    = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t imm8   = GETBITS(instr    , 0, 7);

    ASSERT(!(!P && !W && D && !W));
    ASSERT((coproc & 0b1110) != 0b1010);

    if (!P && !U && !D && !W)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    bool     index  = !!P; // Always true in the T32 instruction set
    bool     add    = !!U;
    uint32_t cp     = coproc;
    uint32_t imm32  = _ZeroExtend(imm8<<2, 32);

    if (W || !P)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDC_LDC2_literal(index, add, cp, imm32);
  }

  /* _DecodeExecute32_LDC_LDC2_immediate_T1_T2 {{{4
   * -----------------------------------------
   */
  void _DecodeExecute32_LDC_LDC2_immediate_T1_T2(uint32_t instr, uint32_t pc) {
    // LDC, LDC2 (immediate) § C2.4.48 T1/T2
    // ---- DECODE --------------------------------------------------
    uint32_t P      = GETBITS(instr>>16, 8, 8);
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t D      = GETBITS(instr>>16, 6, 6);
    uint32_t W      = GETBITS(instr>>16, 5, 5);
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t CRd    = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t imm8   = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);
    ASSERT(!(!P && !U && D && !W));
    ASSERT((coproc & 0b1110) != 0b1010);

    if (!P && !U && !D && !W)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = Rn;
    uint32_t cp     = coproc;
    uint32_t imm32  = _ZeroExtend(imm8<<2, 32);
    bool     index = !!P, add = !!U, wback = !!W;

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDC_LDC2_immediate(n, cp, imm32, index, add, wback);
  }

  /* _DecodeExecute32_STC_STC2_T1_T2 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_STC_STC2_T1_T2(uint32_t instr, uint32_t pc) {
    // STC, STC2 § C2.4.174 T1/T2
    // ---- DECODE --------------------------------------------------
    uint32_t P      = GETBITS(instr>>16, 8, 8);
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t D      = GETBITS(instr>>16, 6, 6);
    uint32_t W      = GETBITS(instr>>16, 5, 5);
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t CRd    = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t imm8   = GETBITS(instr    , 0, 7);

    ASSERT(!(!P && !U && D && !W));
    ASSERT((coproc & 0b1110) != 0b1010);

    if (!P && !U && !D && !W)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = Rn;
    uint32_t cp     = coproc;
    uint32_t imm32  = _ZeroExtend(imm8<<2, 32);
    bool     index  = !!P, add = !!U, wback = !!W;

    if (n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STC_STC2(n, cp, imm32, index, add, wback);
  }

  /* _DecodeExecute32_CDP_CDP2_T1_T2 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_CDP_CDP2_T1_T2(uint32_t instr, uint32_t pc) {
    // CDP, CDP2 § C2.4.25 T1/T2
    // ---- DECODE --------------------------------------------------
    uint32_t opc1   = GETBITS(instr>>16, 4, 7);
    uint32_t CRn    = GETBITS(instr>>16, 0, 3);
    uint32_t CRd    = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc2   = GETBITS(instr    , 5, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    ASSERT((coproc & 0b1110) != 0b1010);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t cp = coproc;

    // ---- EXECUTE -------------------------------------------------
    _Exec_CDP_CDP2(cp);
  }

  /* _DecodeExecute32_x110_0_00x0 {{{4
   * ----------------------------
   */
  void _DecodeExecute32_x110_0_00x0(uint32_t instr, uint32_t pc) {
    // Coprocessor 64-bit move
    uint32_t o0 = GETBITS(instr>>16,12,12);
    uint32_t D  = GETBITS(instr>>16, 6, 6);
    uint32_t L  = GETBITS(instr>>16, 4, 4);

    switch ((o0<<2) | (D<<1) | L) {
      case 0b0'0'0:
      case 0b0'0'1:
      case 0b1'0'0:
      case 0b1'0'1:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b0'1'0:
        // MCRR, MCRR2 - T1
        _DecodeExecute32_MCRR_MCRR2_T1(instr, pc);
        break;

      case 0b0'1'1:
        // MRRC, MRRC2 - T1
        _DecodeExecute32_MRRC_MRRC2_T1(instr, pc);
        break;

      case 0b1'1'0:
        // MCRR, MCRR2 - T2
        _DecodeExecute32_MCRR_MCRR2_T2(instr, pc);
        break;

      case 0b1'1'1:
        // MRRC, MRRC2 - T2
        _DecodeExecute32_MRRC_MRRC2_T2(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_MRRC_MRRC2_T1 {{{4
   * ------------------------------
   */
  void _DecodeExecute32_MRRC_MRRC2_T1(uint32_t instr, uint32_t pc) {
    // MRRC, MRRC2 § C2.4.94 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt2    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc1   = GETBITS(instr    , 4, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    ASSERT((coproc & 0b1110) != 0b1010);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t t2   = Rt2;
    uint32_t cp   = coproc;

    if (t == 15 || t2 == 15)
      THROW_UNPREDICTABLE();

    if (t == 13 || t2 == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MRRC_MRRC2(t, t2, cp);
  }

  /* _DecodeExecute32_MRRC_MRRC2_T2 {{{4
   * ------------------------------
   */
  void _DecodeExecute32_MRRC_MRRC2_T2(uint32_t instr, uint32_t pc) {
    // MRRC, MRRC2 § C2.4.94 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rt2    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc1   = GETBITS(instr    , 4, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    if ((coproc & 0b1110) == 0b1010)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t t2   = Rt2;
    uint32_t cp   = coproc;

    if (t == 15 || t2 == 15)
      THROW_UNPREDICTABLE();

    if (t == 13 || t2 == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MRRC_MRRC2(t, t2, cp);
  }

  /* _DecodeExecute32_MCRR_MCRR2_T1 {{{4
   * ------------------------------
   */
  void _DecodeExecute32_MCRR_MCRR2_T1(uint32_t instr, uint32_t pc) {
    // MCRR, MCRR2 § C2.4.86 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rt2    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc1   = GETBITS(instr    , 4, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    ASSERT((coproc & 0b1110) != 0b1010);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t t2   = Rt2;
    uint32_t cp   = coproc;

    if (t == 15 || t2 == 15)
      THROW_UNPREDICTABLE();

    if (t == 13 || t2 == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MCRR_MCRR2(t, t2, cp);
  }

  /* _DecodeExecute32_MCRR_MCRR2_T2 {{{4
   * ------------------------------
   */
  void _DecodeExecute32_MCRR_MCRR2_T2(uint32_t instr, uint32_t pc) {
    // MCRR, MCRR2 § C2.4.86 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rt2    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc1   = GETBITS(instr    , 4, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    if ((coproc & 0b1110) == 0b1010)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t t2   = Rt2;
    uint32_t cp   = coproc;

    if (t == 15 || t2 == 15)
      THROW_UNPREDICTABLE();

    if (t == 13 || t2 == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MCRR_MCRR2(t, t2, cp);
  }

  /* _DecodeExecute32_x111_0_0xxx_1 {{{4
   * ------------------------------
   */
  void _DecodeExecute32_x111_0_0xxx_1(uint32_t instr, uint32_t pc) {
    // Coprocessor 32-bit move
    uint32_t o0 = GETBITS(instr>>16,12,12);
    uint32_t L  = GETBITS(instr>>16, 4, 4);

    switch ((o0<<1) | L) {
      case 0b0'0:
        // MCR, MCR2 - T1
        _DecodeExecute32_MCR_MCR2_T1(instr, pc);
        break;

      case 0b0'1:
        // MRC, MRC2 - T1
        _DecodeExecute32_MRC_MRC2_T1(instr, pc);
        break;

      case 0b1'0:
        // MCR, MCR2 - T2
        _DecodeExecute32_MCR_MCR2_T2(instr, pc);
        break;

      case 0b1'1:
        // MRC, MRC2 - T2
        _DecodeExecute32_MRC_MRC2_T2(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_MRC_MRC2_T1 {{{4
   * ----------------------------
   */
  void _DecodeExecute32_MRC_MRC2_T1(uint32_t instr, uint32_t pc) {
    // MRC, MRC2 § C2.4.93 T1
    // ---- DECODE --------------------------------------------------
    uint32_t opc1   = GETBITS(instr>>16, 5, 7);
    uint32_t CRn    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc2   = GETBITS(instr    , 5, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    ASSERT((coproc & 0b1110) != 0b1010);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t cp   = coproc;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MRC_MRC2(t, cp);
  }

  /* _DecodeExecute32_MRC_MRC2_T2 {{{4
   * ----------------------------
   */
  void _DecodeExecute32_MRC_MRC2_T2(uint32_t instr, uint32_t pc) {
    // MRC, MRC2 § C2.4.93 T2
    // ---- DECODE --------------------------------------------------
    uint32_t opc1   = GETBITS(instr>>16, 5, 7);
    uint32_t CRn    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc2   = GETBITS(instr    , 5, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    if ((coproc & 0b1110) == 0b1010)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t cp   = coproc;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MRC_MRC2(t, cp);
  }

  /* _DecodeExecute32_MCR_MCR2_T1 {{{4
   * ----------------------------
   */
  void _DecodeExecute32_MCR_MCR2_T1(uint32_t instr, uint32_t pc) {
    // MCR, MCR2 § C2.4.85 T1
    // ---- DECODE --------------------------------------------------
    uint32_t opc1   = GETBITS(instr>>16, 5, 7);
    uint32_t CRn    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc2   = GETBITS(instr    , 5, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    ASSERT((coproc & 0b1110) != 0b1010);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t cp   = coproc;

    if (t == 15 || t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MCR_MCR2(t, cp);
  }

  /* _DecodeExecute32_MCR_MCR2_T2 {{{4
   * ----------------------------
   */
  void _DecodeExecute32_MCR_MCR2_T2(uint32_t instr, uint32_t pc) {
    // MCR, MCR2 § C2.4.85 T2
    // ---- DECODE --------------------------------------------------
    uint32_t opc1   = GETBITS(instr>>16, 5, 7);
    uint32_t CRn    = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t coproc = GETBITS(instr    , 8,11);
    uint32_t opc2   = GETBITS(instr    , 5, 7);
    uint32_t CRm    = GETBITS(instr    , 0, 3);

    if ((coproc & 0b1110) == 0b1010)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t    = Rt;
    uint32_t cp   = coproc;

    if (t == 15 || t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MCR_MCR2(t, cp);
  }

  /* _DecodeExecute32_1101_11xxx {{{4
   * ---------------------------
   */
  void _DecodeExecute32_1101_11xxx(uint32_t instr, uint32_t pc) {
    // Long multiply and divide
    uint32_t op1 = GETBITS(instr>>16, 4, 6);
    uint32_t op2 = GETBITS(instr    , 4, 7);

    switch (op1) {
      case 0b000:
        if (!op2) {
          // SMULL
          _DecodeExecute32_SMULL_T1(instr, pc);
        } else {
          // Unallocated
          UNDEFINED_DEC();
        }
        break;

      case 0b001:
        if (op2 == 0b1111) {
          // SDIV
          _DecodeExecute32_SDIV_T1(instr, pc);
        } else {
          // Unallocated
          UNDEFINED_DEC();
        }
        break;

      case 0b010:
        if (!op2) {
          // UMULL
          _DecodeExecute32_UMULL_T1(instr, pc);
        } else {
          // Unallocated
          UNDEFINED_DEC();
        }
        break;

      case 0b011:
        if (op2 == 0b1111) {
          // UDIV
          _DecodeExecute32_UDIV_T1(instr, pc);
        } else {
          // Unallocated
          UNDEFINED_DEC();
        }
        break;

      case 0b100:
        switch (op2) {
          case 0b0000:
            // SMLAL
            _DecodeExecute32_SMLAL_T1(instr, pc);
            break;

          case 0b0001:
          case 0b0010:
          case 0b0011:
          case 0b0100:
          case 0b0101:
          case 0b0110:
          case 0b0111:
          case 0b1110:
          case 0b1111:
            // Unallocated
            UNDEFINED_DEC();
            break;

          case 0b1000:
          case 0b1001:
          case 0b1010:
          case 0b1011:
            // SMLALBB, SMLALBT, SMLALTB, SMLALTT
            TODO_DEC(); // DSP
            break;

          case 0b1100:
          case 0b1101:
            // SMLALD, SMLALDX
            TODO_DEC(); // DSP
            break;

          default:
            abort();
        }
        break;

      case 0b101:
        switch (op2) {
          case 0b1100:
          case 0b1101:
            // SMLSLD, SMLSLDX
            TODO_DEC(); // DSP
            break;

          default:
            // Unallocated
            UNDEFINED_DEC();
            break;
        }
        break;

      case 0b110:
        switch (op2) {
          case 0b0000:
            // UMLAL
            _DecodeExecute32_UMLAL_T1(instr, pc);
            break;

          case 0b0110:
            // UMAAL
            TODO_DEC(); // DSP
            break;

          default:
            // Unallocated
            UNDEFINED_DEC();
            break;
        }
        break;

      case 0b111:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_SMLAL_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_SMLAL_T1(uint32_t instr, uint32_t pc) {
    // SMLAL § C2.4.155 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t RdLo   = GETBITS(instr    ,12,15);
    uint32_t RdHi   = GETBITS(instr    , 8,11);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t dLo      = RdLo;
    uint32_t dHi      = RdHi;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = false;

    if ((dLo == 13 || dLo == 15) || (dHi == 13 || dHi == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    if (dHi == dLo)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SMLAL(dLo, dHi, n, m, setflags);
  }

  /* _DecodeExecute32_UMLAL_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_UMLAL_T1(uint32_t instr, uint32_t pc) {
    // UMLAL § C2.4.227 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t RdLo   = GETBITS(instr    ,12,15);
    uint32_t RdHi   = GETBITS(instr    , 8,11);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t dLo      = RdLo;
    uint32_t dHi      = RdHi;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = false;

    if ((dLo == 13 || dLo == 15) || (dHi == 13 || dHi == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    if (dHi == dLo)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_UMLAL(dLo, dHi, n, m, setflags);
  }

  /* _DecodeExecute32_SDIV_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_SDIV_T1(uint32_t instr, uint32_t pc) {
    // SDIV § C2.4.143 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    CHECK01(0, BITS(12,15));

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SDIV(d, n, m);
  }

  /* _DecodeExecute32_UDIV_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_UDIV_T1(uint32_t instr, uint32_t pc) {
    // UDIV § C2.4.219 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    CHECK01(0, BITS(12,15));

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_UDIV(d, n, m);
  }

  /* _DecodeExecute32_SMULL_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_SMULL_T1(uint32_t instr, uint32_t pc) {
    // SMULL § C2.4.166 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t RdLo   = GETBITS(instr    ,12,15);
    uint32_t RdHi   = GETBITS(instr    , 8,11);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t dLo      = RdLo;
    uint32_t dHi      = RdHi;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = false;

    if ((dLo == 13 || dLo == 15) || (dHi == 13 || dHi == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    if (dHi == dLo)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SMULL(dLo, dHi, n, m, setflags);
  }

  /* _DecodeExecute32_UMULL_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_UMULL_T1(uint32_t instr, uint32_t pc) {
    // UMULL § C2.4.228 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t RdLo   = GETBITS(instr    ,12,15);
    uint32_t RdHi   = GETBITS(instr    , 8,11);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t dLo      = RdLo;
    uint32_t dHi      = RdHi;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = false;

    if ((dLo == 13 || dLo == 15) || (dHi == 13 || dHi == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    if (dHi == dLo)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_UMULL(dLo, dHi, n, m, setflags);
  }

  /* _DecodeExecute32_1101_10xxx {{{4
   * ---------------------------
   */
  void _DecodeExecute32_1101_10xxx(uint32_t instr, uint32_t pc) {
    // Multiply, multiply accumulate, and absolute difference
    uint32_t op = GETBITS(instr    , 6, 7);

    switch (op) {
      case 0b00:
        // Multiply and absolute difference
        _DecodeExecute32_1101_10xxx_00(instr, pc);
        break;

      case 0b01:
      case 0b10:
      case 0b11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_1101_10xxx_00 {{{4
   * ------------------------------
   */
  void _DecodeExecute32_1101_10xxx_00(uint32_t instr, uint32_t pc) {
    // Multiply and absolute difference
    uint32_t op1 = GETBITS(instr>>16, 4, 6);
    uint32_t Ra  = GETBITS(instr>>16, 0, 3);
    uint32_t op2 = GETBITS(instr    , 4, 5);

    switch ((op1<<2) | op2) {
      case 0b000'00:
        if (Ra != 0b1111) {
          // MLA
          _DecodeExecute32_MLA_T1(instr, pc);
        } else {
          // MUL
          _DecodeExecute32_MUL_T2(instr, pc);
        }
        break;

      case 0b000'01:
        // MLS
        _DecodeExecute32_MLS_T1(instr, pc);
        break;

      case 0b000'10:
      case 0b000'11:
      case 0b010'10:
      case 0b010'11:
      case 0b011'10:
      case 0b011'11:
      case 0b100'10:
      case 0b100'11:
      case 0b101'10:
      case 0b101'11:
      case 0b110'10:
      case 0b110'11:
      case 0b111'01:
      case 0b111'10:
      case 0b111'11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b001'00:
      case 0b001'01:
      case 0b001'10:
      case 0b001'11:
        if (Ra != 0b1111) {
          // SMLABB, SMLABT, SMLATB, SMLATT
          TODO_DEC();
        } else {
          // SMULBB, SMULBT, SMULTB, SMULTT
          TODO_DEC();
        }
        break;

      case 0b010'00:
      case 0b010'01:
        if (Ra != 0b1111) {
          // SMLAD, SMLADX
          TODO_DEC();
        } else {
          // SMUAD, SMUADX
          TODO_DEC();
        }
        break;

      case 0b011'00:
      case 0b011'01:
        if (Ra != 0b1111) {
          // SMLAWB, SMLAWT
          TODO_DEC();
        } else {
          // SMULWB, SMULWT
          TODO_DEC();
        }
        break;

      case 0b100'00:
      case 0b100'01:
        if (Ra != 0b1111) {
          // SMLSD, SMLSDX
          TODO_DEC();
        } else {
          // SMUSD, SMUSDX
          TODO_DEC();
        }
        break;

      case 0b101'00:
      case 0b101'01:
        if (Ra != 0b1111) {
          // SMMLA, SMMLAR
          TODO_DEC();
        } else {
          // SMMUL, SMMULR
          TODO_DEC();
        }
        break;

      case 0b110'00:
      case 0b110'01:
        // SMMLS, SMMLSR
        TODO_DEC();
        break;

      case 0b111'00:
        if (Ra != 0b1111) {
          // USADA8
          TODO_DEC();
        } else {
          // USAD8
          TODO_DEC();
        }
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_MLA_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_MLA_T1(uint32_t instr, uint32_t pc) {
    // MLA § C2.4.87 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Ra   = GETBITS(instr    ,12,15);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Ra != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    uint32_t a        = Ra;
    bool     setflags = false;

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15) || a == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MLA(d, n, m, a, setflags);
  }

  /* _DecodeExecute32_MLS_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_MLS_T1(uint32_t instr, uint32_t pc) {
    // MLS § C2.4.88 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Ra   = GETBITS(instr    ,12,15);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    uint32_t a        = Ra;

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15) || (a == 13 || a == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MLS(d, n, m, a);
  }

  /* _DecodeExecute32_MUL_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_MUL_T2(uint32_t instr, uint32_t pc) {
    // MUL § C2.4.97 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = false;

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MUL(d, n, m, setflags);
  }

  /* _DecodeExecute32_1101_0xxxx {{{4
   * ---------------------------
   */
  void _DecodeExecute32_1101_0xxxx(uint32_t instr, uint32_t pc) {
    // Data-processing (register)
    uint32_t op0 = GETBITS(instr>>16, 7, 7);
    uint32_t op1 = GETBITS(instr    , 4, 7);

    if (!op0) {
      if (!op1) {
        // MOV, MOVS (register-shifted register) — Flag-setting variant
        _DecodeExecute32_MOV_MOVS_register_shifted_register_T2(instr, pc);
      } else if (op1 & BIT(3)) {
        // Register extends
        _DecodeExecute32_1101_00xxx_1xxx(instr, pc);
      } else {
        // Unallocated
        UNDEFINED_DEC();
      }
    } else {
      if (!(op1 & BIT(3))) {
        // Parallel add-subtract
        TODO_DEC(); // DSP
      } else if ((op1 & 0b1100) == 0b1000) {
        // Data-processing (two source registers)
        _DecodeExecute32_1101_01xxx_10xx(instr, pc);
      } else {
        // Unallocated
        UNDEFINED_DEC();
      }
    }
  }

  /* _DecodeExecute32_1101_01xxx_10xxx {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_1101_01xxx_10xx(uint32_t instr, uint32_t pc) {
    // Data-processing (two source registers)
    uint32_t op1 = GETBITS(instr>>16, 4, 6);
    uint32_t op2 = GETBITS(instr    , 4, 5);

    switch ((op1<<2) | op2) {
      case 0b000'00:
        // QADD
        TODO_DEC(); // DSP
        break;

      case 0b000'01:
        // QDADD
        TODO_DEC(); // DSP
        break;

      case 0b000'10:
        // QSUB
        TODO_DEC(); // DSP
        break;

      case 0b000'11:
        // QDSUB
        TODO_DEC(); // DSP
        break;

      case 0b001'00:
        // REV
        _DecodeExecute32_REV_T2(instr, pc);
        break;

      case 0b001'01:
        // REV16
        _DecodeExecute32_REV16_T2(instr, pc);
        break;

      case 0b001'10:
        // RBIT
        _DecodeExecute32_RBIT_T1(instr, pc);
        break;

      case 0b001'11:
        // REVSH
        _DecodeExecute32_REVSH_T2(instr, pc);
        break;

      case 0b010'00:
        // SEL
        TODO_DEC(); // DSP
        break;

      case 0b010'01:
      case 0b010'10:
      case 0b010'11:
      case 0b011'01:
      case 0b011'10:
      case 0b011'11:
      case 0b100'00:
      case 0b100'01:
      case 0b100'10:
      case 0b100'11:
      case 0b101'00:
      case 0b101'01:
      case 0b101'10:
      case 0b101'11:
      case 0b110'00:
      case 0b110'01:
      case 0b110'10:
      case 0b110'11:
      case 0b111'00:
      case 0b111'01:
      case 0b111'10:
      case 0b111'11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b011'00:
        // CLZ
        _DecodeExecute32_CLZ_T1(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_CLZ_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_CLZ_T1(uint32_t instr, uint32_t pc) {
    // CLZ § C2.4.27 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm     = GETBITS(instr>>16, 0, 3);
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t Rm2    = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    if (Rm != Rm2)
      CUNPREDICTABLE_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t m        = Rm;

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_CLZ(d, m);
  }

  /* _DecodeExecute32_REV_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_REV_T2(uint32_t instr, uint32_t pc) {
    // REV § C2.4.126 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rm     = GETBITS(instr>>16, 0, 3);
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t Rm2    = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    if (Rm != Rm2)
      CUNPREDICTABLE_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t m        = Rm;

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_REV(d, m);
  }

  /* _DecodeExecute32_REV16_T2 {{{4
   * -------------------------
   */
  void _DecodeExecute32_REV16_T2(uint32_t instr, uint32_t pc) {
    // REV16 § C2.4.127 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rm     = GETBITS(instr>>16, 0, 3);
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t Rm2    = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    if (Rm != Rm2)
      CUNPREDICTABLE_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t m        = Rm;

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_REV16(d, m);
  }

  /* _DecodeExecute32_RBIT_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_RBIT_T1(uint32_t instr, uint32_t pc) {
    // RBIT § C2.4.125 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rm     = GETBITS(instr>>16, 0, 3);
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t Rm2    = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    if (Rm != Rm2)
      CUNPREDICTABLE_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t m        = Rm;

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_RBIT(d, m);
  }

  /* _DecodeExecute32_REVSH_T2 {{{4
   * -------------------------
   */
  void _DecodeExecute32_REVSH_T2(uint32_t instr, uint32_t pc) {
    // REVSH § C2.4.128 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rm     = GETBITS(instr>>16, 0, 3);
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t Rm2    = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    if (Rm != Rm2)
      CUNPREDICTABLE_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t m        = Rm;

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_REVSH(d, m);
  }

  /* _DecodeExecute32_1101_00xxx_1xxx {{{4
   * --------------------------------
   */
  void _DecodeExecute32_1101_00xxx_1xxx(uint32_t instr, uint32_t pc) {
    // Register extends
    uint32_t op1  = GETBITS(instr>>16, 5, 6);
    uint32_t U    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr    , 0, 3);

    switch ((op1<<1) | U) {
      case 0b00'0:
        if (Rn != 0b1111) {
          // SXTAH
          TODO_DEC(); // DSP
        } else {
          // SXTH
          _DecodeExecute32_SXTH_T2(instr, pc);
        }
        break;

      case 0b00'1:
        if (Rn != 0b1111) {
          // UXTAH
          TODO_DEC(); // DSP
        } else {
          // UXTH
          _DecodeExecute32_UXTH_T2(instr, pc);
        }
        break;

      case 0b01'0:
        if (Rn != 0b1111) {
          // SXTAB16
          TODO_DEC(); // DSP
        } else {
          // SXTB16
          TODO_DEC(); // DSP
        }
        break;

      case 0b01'1:
        if (Rn != 0b1111) {
          // UXTAB16
          TODO_DEC(); // DSP
        } else {
          // UXTB16
          TODO_DEC(); // DSP
        }
        break;

      case 0b10'0:
        if (Rn != 0b1111) {
          // SXTAB
          TODO_DEC(); // DSP
        } else {
          // SXTB
          _DecodeExecute32_SXTB_T2(instr, pc);
        }
        break;

      case 0b10'1:
        if (Rn != 0b1111) {
          // UXTAB
          TODO_DEC(); // DSP
        } else {
          // UXTB
          _DecodeExecute32_UXTB_T2(instr, pc);
        }
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

  /* _DecodeExecute32_SXTB_T2 {{{4
   * ------------------------
   */
  void _DecodeExecute32_SXTB_T2(uint32_t instr, uint32_t pc) {
    // SXTB § C2.4.205 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t rotate = GETBITS(instr    , 4, 5);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(6), 0);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = (rotate<<3);

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SXTB(d, m, rotation);
  }

  /* _DecodeExecute32_UXTB_T2 {{{4
   * ------------------------
   */
  void _DecodeExecute32_UXTB_T2(uint32_t instr, uint32_t pc) {
    // UXTB § C2.4.245 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t rotate = GETBITS(instr    , 4, 5);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(6), 0);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = (rotate<<3);

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_UXTB(d, m, rotation);
  }

  /* _DecodeExecute32_SXTH_T2 {{{4
   * ------------------------
   */
  void _DecodeExecute32_SXTH_T2(uint32_t instr, uint32_t pc) {
    // SXTH § C2.4.207 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t rotate = GETBITS(instr    , 4, 5);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(6), 0);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = (rotate<<3);

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SXTH(d, m, rotation);
  }

  /* _DecodeExecute32_UXTH_T2 {{{4
   * ------------------------
   */
  void _DecodeExecute32_UXTH_T2(uint32_t instr, uint32_t pc) {
    // UXTH § C2.4.247 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t rotate = GETBITS(instr    , 4, 5);
    uint32_t Rm     = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(6), 0);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t rotation = (rotate<<3);

    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_UXTH(d, m, rotation);
  }

  /* _DecodeExecute32_MOV_MOVS_register_shifted_register_T2 {{{4
   * ------------------------------------------------------
   */
  void _DecodeExecute32_MOV_MOVS_register_shifted_register_T2(uint32_t instr, uint32_t pc) {
    // MOV, MOVS (register-shifted register) § C2.4.91 T2
    // ---- DECODE --------------------------------------------------
    uint32_t type = GETBITS(instr>>16, 5, 6);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rm   = GETBITS(instr>>16, 0, 3);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t Rs   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    uint32_t s        = Rs;
    bool     setflags = !!S;
    SRType   shiftT   = _DecodeRegShift(type);

    if ((d == 13 || d == 15) || (m == 13 || m == 15) || (s == 13 || s == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOV_MOVS_register_shifted_register(d, m, s, setflags, shiftT);
  }

  /* _DecodeExecute32_0100 {{{4
   * ---------------------
   */
  void _DecodeExecute32_0100(uint32_t instr, uint32_t pc) {
    // Load/store (multiple, dual, exclusive, acquire-release), table branch
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
        _DecodeExecute32_0100_010(instr, pc);
        break;

      case 0b0'11:
        // Load/store dual (post-indexed)
        _DecodeExecute32_0100_011(instr, pc);
        break;

      case 0b1'10:
        // Load/store dual (literal and immediate)
        _DecodeExecute32_0100_110(instr, pc);
        break;

      case 0b1'11:
        // Load/store dual (pre-indexed), secure gateway
        _DecodeExecute32_0100_111(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_0100_111 {{{4
   * -------------------------
   */
  void _DecodeExecute32_0100_111(uint32_t instr, uint32_t pc) {
    // Load/store dual (pre-indexed), secure gateway
    uint32_t op0 = GETBITS(instr>>16, 7, 7);
    uint32_t op1 = GETBITS(instr>>16, 4, 4);
    uint32_t op2 = GETBITS(instr>>16, 0, 3);
    uint32_t op3 = GETBITS(instr    , 0,15);

    if (op2 == 0b1111) {
      switch ((op0<<1) | op1) {
        case 0b0'0:
        case 0b1'0:
        case 0b1'1:
          THROW_UNPREDICTABLE();

        case 0b0'1:
          if (op3 == 0b1110'1001'0111'1111) {
            // SG
            _DecodeExecute32_SG_T1(instr, pc);
          } else
            THROW_UNPREDICTABLE();
          break;

        default:
          abort();
      }
    } else {
      // Load/store dual (immediate, pre-indexed)
      _DecodeExecute32_0100_111_xxxx(instr, pc);
    }
  }

  /* _DecodeExecute32_SG_T1 {{{4
   * ----------------------
   */
  void _DecodeExecute32_SG_T1(uint32_t instr, uint32_t pc) {
    // SG § C2.4.146 T1
    // ---- DECODE --------------------------------------------------
    // No encoding specific operations

    CHECKV(8);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SG();
  }

  /* _DecodeExecute32_0100_111_xxxx {{{4
   * ------------------------------
   */
  void _DecodeExecute32_0100_111_xxxx(uint32_t instr, uint32_t pc) {
    // Load/store dual (immediate, pre-indexed)
    uint32_t L = GETBITS(instr>>16, 4, 4);

    // These are also mapped via Load/store dual (immediate, post-indexed) and
    // Load/store dual (immediate).
    if (!L) {
      _DecodeExecute32_STRD_immediate_T1(instr, pc);
    } else {
      _DecodeExecute32_LDRD_immediate_T1(instr, pc);
    }
  }

  /* _DecodeExecute32_0100_110 {{{4
   * -------------------------
   */
  void _DecodeExecute32_0100_110(uint32_t instr, uint32_t pc) {
    // Load/store dual (literal and immediate)
    uint32_t op0 = GETBITS(instr>>16, 0, 3);

    switch (op0) {
      case 0b1111:
        // LDRD (literal)
        _DecodeExecute32_LDRD_literal_T1(instr, pc);
        break;

      default:
        // Load/store dual (immediate)
        _DecodeExecute32_0100_110_xxxx(instr, pc);
        break;
    }
  }

  /* _DecodeExecute32_0100_110_xxxx {{{4
   * ------------------------------
   */
  void _DecodeExecute32_0100_110_xxxx(uint32_t instr, uint32_t pc) {
    // Load/store dual (immediate)
    uint32_t L = GETBITS(instr>>16, 4, 4);

    // These are also mapped via Load/store dual (immediate, pre-indexed) and
    // Load/store dual (immediate, post-indexed).
    if (!L) {
      // STRD (immediate)
      _DecodeExecute32_STRD_immediate_T1(instr, pc);
    } else {
      // LDRD (immediate)
      _DecodeExecute32_LDRD_immediate_T1(instr, pc);
    }
  }

  /* _DecodeExecute32_LDRD_literal_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_LDRD_literal_T1(uint32_t instr, uint32_t pc) {
    // LDRD (literal) § C2.4.60 T1
    // ---- DECODE --------------------------------------------------
    uint32_t P    = GETBITS(instr>>16, 8, 8);
    uint32_t U    = GETBITS(instr>>16, 7, 7);
    uint32_t W    = GETBITS(instr>>16, 5, 5);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t Rt2  = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(P || W);
    ASSERT(!(P && W && !U));

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t t2     = Rt2;
    uint32_t imm32  = _ZeroExtend(imm8<<2, 32);
    bool     add    = !!U;

    if ((t == 13 || t == 15) || (t2 == 13 || t2 == 15))
      THROW_UNPREDICTABLE();

    if (t == t2)
      CUNPREDICTABLE_UNDEFINED();

    if (W)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRD_literal(t, t2, imm32, add);
  }

  /* _DecodeExecute32_0100_010 {{{4
   * -------------------------
   */
  void _DecodeExecute32_0100_010(uint32_t instr, uint32_t pc) {
    // Load/store exclusive, load-acquire/store-release, table branch
    uint32_t op0 = GETBITS(instr>>16, 7, 7);
    uint32_t op1 = GETBITS(instr    ,12,20);
    uint32_t op2 = GETBITS(instr    , 5, 7);

    if (!op0) {
      if ((op1 & 0b1'0000'1111) == 0b0'0000'1111) {
        // TT, TTT, TTA, TTAT
        _DecodeExecute32_TT_TTT_TTA_TTAT_T1(instr, pc);
      } else {
        // Load/store exclusive
        _DecodeExecute32_0100_010_0_xxxxxxxxx(instr, pc);
      }
    } else {
      if (!op2) {
        if (!GETBIT(op1, 8)) {
          // Unallocated
          UNDEFINED_DEC();
        } else {
          // TBB, TBH
          _DecodeExecute32_TBB_TBH_T1(instr, pc);
        }
      } else {
        if (GETBITS(op2, 1, 2) == 0b01) {
          // Load/store exclusive byte/half/dual
          _DecodeExecute32_0100_010_1_01x(instr, pc);
        } else if (GETBIT(op2, 2)) {
          // Load-acquire/store-release
          _DecodeExecute32_0100_010_1_1xx(instr, pc);
        } else {
          abort();
        }
      }
    }
  }

  /* _DecodeExecute32_0100_010_1_1xx {{{4
   * -------------------------------
   */
  void _DecodeExecute32_0100_010_1_1xx(uint32_t instr, uint32_t pc) {
    // Load-acquire/store-release
    uint32_t L  = GETBITS(instr>>16, 4, 4);
    uint32_t op = GETBITS(instr    , 6, 6);
    uint32_t sz = GETBITS(instr    , 4, 5);

    uint32_t L_op_sz = (L<<3) | (op<<2) | sz;

    switch (L_op_sz) {
      case 0b0'0'00:
        // STLB
        _DecodeExecute32_STLB_T1(instr, pc);
        break;

      case 0b0'0'01:
        // STLH
        _DecodeExecute32_STLH_T1(instr, pc);
        break;

      case 0b0'0'10:
        // STL
        _DecodeExecute32_STL_T1(instr, pc);
        break;

      case 0b0'1'00:
        // STLEXB
        _DecodeExecute32_STLEXB_T1(instr, pc);
        break;

      case 0b0'1'01:
        // STLEXH
        _DecodeExecute32_STLEXH_T1(instr, pc);
        break;

      case 0b0'1'10:
        // STLEX
        _DecodeExecute32_STLEX_T1(instr, pc);
        break;

      case 0b1'0'00:
        // LDAB
        _DecodeExecute32_LDAB_T1(instr, pc);
        break;

      case 0b1'0'01:
        // LDAH
        _DecodeExecute32_LDAH_T1(instr, pc);
        break;

      case 0b1'0'10:
        // LDA
        _DecodeExecute32_LDA_T1(instr, pc);
        break;

      case 0b1'1'00:
        // LDAEXB
        _DecodeExecute32_LDAEXB_T1(instr, pc);
        break;

      case 0b1'1'01:
        // LDAEXH
        _DecodeExecute32_LDAEXH_T1(instr, pc);
        break;

      case 0b1'1'10:
        // LDAEX
        _DecodeExecute32_LDAEX_T1(instr, pc);
        break;

      case 0b0'0'11:
      case 0b0'1'11:
      case 0b1'0'11:
      case 0b1'1'11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_STLB_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_STLB_T1(uint32_t instr, uint32_t pc) {
    // STLB § C2.4.176 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;

    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STLB(t, n);
  }

  /* _DecodeExecute32_STLH_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_STLH_T1(uint32_t instr, uint32_t pc) {
    // STLH § C2.4.180 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;

    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STLH(t, n);
  }

  /* _DecodeExecute32_STL_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_STL_T1(uint32_t instr, uint32_t pc) {
    // STL § C2.4.175 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;

    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STL(t, n);
  }

  /* _DecodeExecute32_STLEXB_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_STLEXB_T1(uint32_t instr, uint32_t pc) {
    // STLEXB § C2.4.178 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);
    uint32_t Rd = GETBITS(instr    , 0, 3);

    CHECK01(0, BITS(8,11));
    CHECKV(8);

    uint32_t d  = Rd;
    uint32_t t  = Rt;
    uint32_t n  = Rn;

    if ((d == 13 || d == 15) || (t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    if (d == n || d == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STLEXB(d, t, n);
  }

  /* _DecodeExecute32_STLEXH_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_STLEXH_T1(uint32_t instr, uint32_t pc) {
    // STLEXH § C2.4.179 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);
    uint32_t Rd = GETBITS(instr    , 0, 3);

    CHECK01(0, BITS(8,11));
    CHECKV(8);

    uint32_t d  = Rd;
    uint32_t t  = Rt;
    uint32_t n  = Rn;

    if ((d == 13 || d == 15) || (t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    if (d == n || d == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STLEXH(d, t, n);
  }

  /* _DecodeExecute32_STLEX_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_STLEX_T1(uint32_t instr, uint32_t pc) {
    // STLEX § C2.4.177 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);
    uint32_t Rd = GETBITS(instr    , 0, 3);

    CHECK01(0, BITS(8,11));
    CHECKV(8);

    uint32_t d  = Rd;
    uint32_t t  = Rt;
    uint32_t n  = Rn;

    if ((d == 13 || d == 15) || (t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    if (d == n || d == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STLEX(d, t, n);
  }

  /* _DecodeExecute32_LDAB_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_LDAB_T1(uint32_t instr, uint32_t pc) {
    // LDAB § C2.4.43 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;
    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDAB(t, n);
  }

  /* _DecodeExecute32_LDAH_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_LDAH_T1(uint32_t instr, uint32_t pc) {
    // LDAH § C2.4.47 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;
    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDAH(t, n);
  }

  /* _DecodeExecute32_LDA_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_LDA_T1(uint32_t instr, uint32_t pc) {
    // LDA § C2.4.42 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;
    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDA(t, n);
  }

  /* _DecodeExecute32_LDAEXB_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_LDAEXB_T1(uint32_t instr, uint32_t pc) {
    // LDAEXB § C2.4.45 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;
    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDAEXB(t, n);
  }

  /* _DecodeExecute32_LDAEXH_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_LDAEXH_T1(uint32_t instr, uint32_t pc) {
    // LDAEXH § C2.4.46 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;
    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDAEXH(t, n);
  }

  /* _DecodeExecute32_LDAEX_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_LDAEX_T1(uint32_t instr, uint32_t pc) {
    // LDAEX § C2.4.44 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));
    CHECKV(8);

    uint32_t t  = Rt;
    uint32_t n  = Rn;
    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDAEX(t, n);
  }

  /* _DecodeExecute32_0100_010_1_01x {{{4
   * -------------------------------
   */
  void _DecodeExecute32_0100_010_1_01x(uint32_t instr, uint32_t pc) {
    // Load/store exclusive byte/half/dual
    uint32_t L  = GETBITS(instr>>16, 4, 4);
    uint32_t sz = GETBITS(instr    , 4, 5);

    switch ((L<<2) | sz) {
      case 0b0'00:
        // STREXB
        _DecodeExecute32_STREXB_T1(instr, pc);
        break;

      case 0b0'01:
        // STREXH
        _DecodeExecute32_STREXH_T1(instr, pc);
        break;

      case 0b0'10:
      case 0b0'11:
      case 0b1'10:
      case 0b1'11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b1'00:
        // LDREXB
        _DecodeExecute32_LDREXB_T1(instr, pc);
        break;

      case 0b1'01:
        // LDREXH
        _DecodeExecute32_LDREXH_T1(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_LDREXB_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_LDREXB_T1(uint32_t instr, uint32_t pc) {
    // LDREXB § C2.4.62 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));

    uint32_t t = Rt;
    uint32_t n = Rn;

    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDREXB(t, n);
  }

  /* _DecodeExecute32_LDREXH_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_LDREXH_T1(uint32_t instr, uint32_t pc) {
    // LDREXH § C2.4.63 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);

    CHECK01(0, BITS(0,3) | BITS(8,11));

    uint32_t t = Rt;
    uint32_t n = Rn;

    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDREXH(t, n);
  }

  /* _DecodeExecute32_STREXB_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_STREXB_T1(uint32_t instr, uint32_t pc) {
    // STREXB § C2.4.190 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);
    uint32_t Rd = GETBITS(instr    , 0, 3);

    CHECK01(0, BITS(8,11));

    uint32_t d = Rd;
    uint32_t t = Rt;
    uint32_t n = Rn;

    if ((d == 13 || d == 15) || (t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    if (d == n || d == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STREXB(d, t, n);
  }

  /* _DecodeExecute32_STREXH_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_STREXH_T1(uint32_t instr, uint32_t pc) {
    // STREXH § C2.4.191 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rt = GETBITS(instr    ,12,15);
    uint32_t Rd = GETBITS(instr    , 0, 3);

    CHECK01(0, BITS(8,11));

    uint32_t d = Rd;
    uint32_t t = Rt;
    uint32_t n = Rn;

    if ((d == 13 || d == 15) || (t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    if (d == n || d == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STREXH(d, t, n);
  }

  /* _DecodeExecute32_TBB_TBH_T1 {{{4
   * ---------------------------
   */
  void _DecodeExecute32_TBB_TBH_T1(uint32_t instr, uint32_t pc) {
    // TBB, TBH § C2.4.208 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t H  = GETBITS(instr    , 4, 4);
    uint32_t Rm = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BITS(8,11), BITS(12,15));

    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     isTBH  = !!H;

    if (n == 13 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    if (_InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_TBB(n, m, isTBH);
  }

  /* _DecodeExecute32_0100_010_0_xxxxxxxxx {{{4
   * -------------------------------------
   */
  void _DecodeExecute32_0100_010_0_xxxxxxxxx(uint32_t instr, uint32_t pc) {
    // Load/store exclusive
    uint32_t L    = GETBITS(instr>>16, 4, 4);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t L_Rt = (L<<4) | Rt;

    if (L_Rt != 0b0'1111) {
      // STREX
      _DecodeExecute32_STREX_T1(instr, pc);
    } else if (L) {
      // LDREX
      _DecodeExecute32_LDREX_T1(instr, pc);
    } else
      abort();
  }

  /* _DecodeExecute32_STREX_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_STREX_T1(uint32_t instr, uint32_t pc) {
    // STREX § C2.4.189 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    uint32_t d      = Rd;
    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8<<2, 32);

    ASSERT(t != 15);
    if ((d == 13 || d == 15) || t == 13 || n == 15)
      THROW_UNPREDICTABLE();

    if (d == n || d == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STREX(d, t, n, imm32);
  }

  /* _DecodeExecute32_LDREX_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_LDREX_T1(uint32_t instr, uint32_t pc) {
    // LDREX § C2.4.61 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    CHECK01(0, BITS(8,11));

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8<<2, 32);

    if ((t == 13 || t == 15) || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDREX(t, n, imm32);
  }

  /* _DecodeExecute32_TT_TTT_TTA_TTAT_T1 {{{4
   * -----------------------------------
   */
  void _DecodeExecute32_TT_TTT_TTA_TTAT_T1(uint32_t instr, uint32_t pc) {
    // TT, TTT, TTA, TTAT § C2.4.213 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn = GETBITS(instr>>16, 0, 3);
    uint32_t Rd = GETBITS(instr    , 8,11);
    uint32_t A  = GETBITS(instr    , 7, 7);
    uint32_t T  = GETBITS(instr    , 6, 6);

    CHECK01(BITS(0,5), 0);
    CHECKV(8);

    uint32_t d            = Rd;
    uint32_t n            = Rn;
    bool     alt          = !!A;
    bool     forceUnpriv  = !!T;

    if ((d == 13 || d == 15) || n == 15)
      THROW_UNPREDICTABLE();

    if (alt && !_IsSecure())
      THROW_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_TT(d, n, alt, forceUnpriv);
  }

  /* _DecodeExecute32_0100_x0x {{{4
   * -------------------------
   */
  void _DecodeExecute32_0100_x0x(uint32_t instr, uint32_t pc) {
    // Load/store multiple
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
        _DecodeExecute32_STM_STMIA_STMEA_T2(instr, pc);
        break;

      case 0b01'1:
        // LDM, LDMIA, LDMFD
        _DecodeExecute32_LDM_LDMIA_LDMFD_T2(instr, pc);
        break;

      case 0b10'0:
        // STMDB, STMFD
        _DecodeExecute32_STMDB_STMFD_T1(instr, pc);
        break;

      case 0b10'1:
        // LDMDB, LDMEA
        _DecodeExecute32_LDMDB_LDMEA_T1(instr, pc);
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

  /* _DecodeExecute32_STM_STMIA_STMEA_T2 {{{4
   * -----------------------------------
   */
  void _DecodeExecute32_STM_STMIA_STMEA_T2(uint32_t instr, uint32_t pc) {
    // STM, STMIA, STMEA § C2.4.181 T2
    // ---- DECODE --------------------------------------------------
    uint32_t W        = GETBITS(instr>>16, 5, 5);
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t M        = GETBITS(instr    ,14,14);
    uint32_t regList  = GETBITS(instr    ,0 ,12);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(13) | BIT(15), 0);

    uint32_t n          = Rn;
    uint32_t registers  = (M<<14) | regList;
    bool     wback      = !!W;

    if (n == 15)
      THROW_UNPREDICTABLE();

    if (_BitCount(registers) < 2)
      CUNPREDICTABLE_UNDEFINED();

    if (wback && GETBIT(registers, n))
      CUNPREDICTABLE_UNDEFINED();

    TRACEI(STM, T2, "n=%u registers=0x%x wback=%u", n, registers, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STM(n, registers, wback);
  }

  /* _DecodeExecute32_LDM_LDMIA_LDMFD_T2 {{{4
   * -----------------------------------
   */
  void _DecodeExecute32_LDM_LDMIA_LDMFD_T2(uint32_t instr, uint32_t pc) {
    // LDM, LDMIA, LDMFD § C2.4.50 T2
    // ---- DECODE --------------------------------------------------
    uint32_t W        = GETBITS(instr>>16, 4, 4);
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t P        = GETBITS(instr    ,15,15);
    uint32_t M        = GETBITS(instr    ,14,14);
    uint32_t regList  = GETBITS(instr    , 0,12);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(13), 0);

    uint32_t n          = Rn;
    uint32_t registers  = (P<<15) | (M<<14) | regList;
    bool     wback      = !!W;

    if (n == 15 || _BitCount(registers) < 2 || (P && M))
      CUNPREDICTABLE_UNDEFINED();

    if (GETBIT(registers, 15) && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    if (wback && GETBIT(registers, n))
      THROW_UNPREDICTABLE();

    TRACEI(LDM, T2, "n=%u registers=0x%x wback=%u", n, registers, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDM(n, registers, wback);
  }

  /* _DecodeExecute32_STMDB_STMFD_T1 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_STMDB_STMFD_T1(uint32_t instr, uint32_t pc) {
    // STMDB, STMFD § C2.4.182 T1
    // ---- DECODE --------------------------------------------------
    uint32_t W        = GETBITS(instr>>16, 5, 5);
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t M        = GETBITS(instr    ,14,14);
    uint32_t regList  = GETBITS(instr    , 0,12);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(13) | BIT(15), 0);

    uint32_t  n         = Rn;
    uint32_t  registers = regList | (M<<14);
    bool      wback     = !!W;

    if (n == 15) // XXX
      THROW_UNPREDICTABLE();

    if (_BitCount(registers) < 2)
      CUNPREDICTABLE_UNDEFINED();

    if (wback && GETBIT(registers, n))
      CUNPREDICTABLE_UNDEFINED();

    TRACEI(STMDB, T1, "n=%u registers=0x%x wback=%u", n, registers, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STMDB(n, registers, wback);
  }

  /* _DecodeExecute32_LDMDB_LDMEA_T1 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_LDMDB_LDMEA_T1(uint32_t instr, uint32_t pc) {
    // LDMDB, LDMEA § C2.4.51 T1
    // ---- DECODE --------------------------------------------------
    uint32_t W        = GETBITS(instr>>16, 5, 5);
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t P        = GETBITS(instr    ,15,15);
    uint32_t M        = GETBITS(instr    ,14,14);
    uint32_t regList  = GETBITS(instr    , 0,12);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(13), 0);

    uint32_t n          = Rn;
    uint32_t registers  = (P<<15) | (M<<14) | regList;
    bool     wback      = !!W;

    if (n == 15)
      THROW_UNPREDICTABLE();

    if (_BitCount(registers) < 2)
      CUNPREDICTABLE_UNDEFINED();

    if (P && M)
      CUNPREDICTABLE_UNDEFINED();

    if (GETBIT(registers,15) && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    if (wback && GETBIT(registers, n))
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDMDB(n, registers, wback);
  }

  /* _DecodeExecute32_0100_011 {{{4
   * -------------------------
   */
  void _DecodeExecute32_0100_011(uint32_t instr, uint32_t pc) {
    // Load/store dual (post-indexed)
    uint32_t op0 = GETBITS(instr>>16, 0, 3);
    switch (op0) {
      case 0b1111:
        THROW_UNPREDICTABLE();

      default:
        // Load/store dual (immediate, post-indexed)
        return _DecodeExecute32_0100_011_LS(instr, pc);
    }
  }

  /* _DecodeExecute32_0100_011_LS {{{4
   * ----------------------------
   */
  void _DecodeExecute32_0100_011_LS(uint32_t instr, uint32_t pc) {
    // Load/store dual (immediate, post-indexed)
    uint32_t L = GETBITS(instr>>16, 4, 4);

    // These are also mapped via Load/store dual (immediate, pre-indexed) and
    // Load/store dual (immediate).
    if (!L) {
      // STRD (immediate)
      _DecodeExecute32_STRD_immediate_T1(instr, pc);
    } else {
      // LDRD (immediate)
      _DecodeExecute32_LDRD_immediate_T1(instr, pc);
    }
  }

  /* _DecodeExecute32_STRD_immediate_T1 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_STRD_immediate_T1(uint32_t instr, uint32_t pc) {
    // STRD (immediate) § C2.4.188 T1
    // ---- DECODE --------------------------------------------------
    uint32_t P    = GETBITS(instr>>16, 8, 8);
    uint32_t U    = GETBITS(instr>>16, 7, 7);
    uint32_t W    = GETBITS(instr>>16, 5, 5);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t Rt2  = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(P || W);
    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t = Rt, t2 = Rt2, n = Rn, imm32 = _ZeroExtend(imm8<<2, 32);
    bool index = !!P, add = !!U, wback = !!W;
    if (wback && (n == t || n == t2))
      CUNPREDICTABLE_UNDEFINED();

    if (n == 15 || (t == 13 || t == 15) || (t2 == 13 || t2 == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRD_immediate(t, t2, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDRD_immediate_T1 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_LDRD_immediate_T1(uint32_t instr, uint32_t pc) {
    // LDRD (immediate) § C2.4.59 T1
    // ---- DECODE --------------------------------------------------
    uint32_t P    = GETBITS(instr>>16, 8, 8);
    uint32_t U    = GETBITS(instr>>16, 7, 7);
    uint32_t W    = GETBITS(instr>>16, 5, 5);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t Rt2  = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(P || W);
    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t = Rt, t2 = Rt2, n = Rn, imm32 = _ZeroExtend(imm8<<2, 32);
    bool index = !!P, add = !!U, wback = !!W;
    if (wback && (n == t || n == t2))
      CUNPREDICTABLE_UNDEFINED();

    if ((t == 13 || t == 15) || (t2 == 13 || t2 == 15) || t == t2)
      CUNPREDICTABLE_UNDEFINED();

    TRACEI(LDRD_imm, T1, "t=%u t2=%u n=%u imm32=0x%x index=%u add=%u wback=%u", t, t2, n, imm32, index, add, wback);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRD_immediate(t, t2, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_0101 {{{4
   * ---------------------
   */
  void _DecodeExecute32_0101(uint32_t instr, uint32_t pc) {
    // Data-processing (shifted register)
    uint32_t op1  = GETBITS(instr>>16, 5, 8);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    uint32_t imm3_imm2_type = (imm3 << 4) | (imm2<<2) | type;

    switch (op1) {
      case 0b0000:
        if (!S) {
          // AND (register) — AND, rotate right with extend variant
          _DecodeExecute32_AND_register_T2(instr, pc);
        } else if (imm3_imm2_type != 0b0000011) {
          if (Rd != 0b1111) {
            // AND (register) — ANDS, shift or rotate by value variant
            _DecodeExecute32_AND_register_T2(instr, pc);
          } else {
            // TST (register) — Shift or rotate by value variant
            _DecodeExecute32_TST_register_T2(instr, pc);
          }
        } else {
          if (Rd != 0b1111) {
            // AND (register) — ANDS, rotate right with extend variant
            _DecodeExecute32_AND_register_T2(instr, pc);
          } else {
            // TST (register) — Rotate right with extend variant
            _DecodeExecute32_TST_register_T2(instr, pc);
          }
        }
        break;

      case 0b0001:
        // BIC (register)
        _DecodeExecute32_BIC_register_T2(instr, pc);
        break;

      case 0b0010:
        if (!S) {
          if (Rn != 0b1111) {
            // ORR (register) — ORR, rotate right with extend variant
            _DecodeExecute32_ORR_register_T2(instr, pc);
          } else {
            // MOV (register) — MOV, rotate right with extend variant
            _DecodeExecute32_MOV_register_T3(instr, pc);
          }
        } else {
          if (Rn != 0b1111) {
            // ORR (register) — ORRS, rotate right with extend variant
            _DecodeExecute32_ORR_register_T2(instr, pc);
          } else {
            // MOV (register) — MOVS, rotate right with extend variant
            _DecodeExecute32_MOV_register_T3(instr, pc);
          }
        }
        break;

      case 0b0011:
        if (!S) {
          if (Rn != 0b1111) {
            // ORN (register) — ORN, rotate right with extend variant
            _DecodeExecute32_ORN_register_T1(instr, pc);
          } else {
            // MVN (register) — MVN, rotate right with extend variant
            _DecodeExecute32_MVN_register_T2(instr, pc);
          }
        } else {
          if (Rn != 0b1111) {
            // ORN (register) — ORNS, rotate right with extend variant
            _DecodeExecute32_ORN_register_T1(instr, pc);
          } else {
            // MVN (register) — MVNS, rotate right with extend variant
            _DecodeExecute32_MVN_register_T2(instr, pc);
          }
        }
        break;

      case 0b0100:
        if (!S) {
          // EOR (register) — EOR, rotate right with extend variant
          _DecodeExecute32_EOR_register_T2(instr, pc);
        } else if (imm3_imm2_type != 0b0000011) {
          if (Rd != 0b1111) {
            // EOR (register) — EORS, shift or rotate by value variant
            _DecodeExecute32_EOR_register_T2(instr, pc);
          } else {
            // TEQ (register) — Shift or rotate by value variant
            _DecodeExecute32_TEQ_register_T1(instr, pc);
          }
        } else {
          if (Rd != 0b1111) {
            // EOR (register) — EORS, rotate right with extend variant
            _DecodeExecute32_EOR_register_T2(instr, pc);
          } else {
            // TEQ (register) — Rotate right with extend variant
            _DecodeExecute32_TEQ_register_T1(instr, pc);
          }
        }
        break;

      case 0b0110:
        if (!S) {
          switch (imm3_imm2_type & 0b11) {
            case 0b00:
              // PKHBT, PKHTB - PKHBT variant
              _DecodeExecute32_PKHBT_PKHTB_T1(instr, pc);
              break;

            case 0b10:
              // PKHBT, PKHTB - PKHTB variant
              _DecodeExecute32_PKHBT_PKHTB_T1(instr, pc);
              break;

            case 0b01:
            case 0b11:
              // Unallocated
              UNDEFINED_DEC();
              break;

            default:
              abort();
          }
        } else {
          // ?
          UNDEFINED_DEC();
        }
        break;

      case 0b1000:
        if (!S) {
          if (Rn != 0b1101) {
            // ADD (register) — ADD, rotate right with extend variant
            _DecodeExecute32_ADD_register_T3(instr, pc);
          } else {
            // ADD (SP plus register) — ADD, rotate right with extend variant
            _DecodeExecute32_ADD_SP_plus_register_T3(instr, pc);
          }
        } else {
          if (Rd == 0b1111) {
            // CMN (register)
            _DecodeExecute32_CMN_register_T2(instr, pc);
          } else {
            if (Rn != 0b1101) {
              // ADD (register) — ADDS, rotate right with extend variant
              _DecodeExecute32_ADD_register_T3(instr, pc);
            } else {
              // ADD (SP plus register) — ADDS, rotate right with extend variant
              _DecodeExecute32_ADD_SP_plus_register_T3(instr, pc);
            }
          }
        }
        break;

      case 0b1010:
        // ADC (register)
        _DecodeExecute32_ADC_register_T2(instr, pc);
        break;

      case 0b1011:
        // SBC (register)
        _DecodeExecute32_SBC_register_T2(instr, pc);
        break;

      case 0b1101:
        if (!S) {
          if (Rn != 0b1101) {
            // SUB (register) — SUB, rotate right with extend variant
            _DecodeExecute32_SUB_register_T2(instr, pc);
          } else {
            // SUB (SP minus register) — SUB, rotate right with extend variant
            _DecodeExecute32_SUB_SP_minus_register_T1(instr, pc);
          }
        } else {
          if (Rd == 0b1111) {
            // CMP (register)
            _DecodeExecute32_CMP_register_T3(instr, pc);
          } else {
            if (Rn != 0b1101) {
              // SUB (register) — SUBS, rotate right with extend variant
              _DecodeExecute32_SUB_register_T2(instr, pc);
            } else {
              // SUB (SP minus register) — SUBS, rotate right with extend variant
              _DecodeExecute32_SUB_SP_minus_register_T1(instr, pc);
            }
          }
        }
        break;

      case 0b1110:
        // RSB (register)
        _DecodeExecute32_RSB_register_T1(instr, pc);
        break;

      case 0b0101:
      case 0b0111:
      case 0b1001:
      case 0b1100:
      case 0b1111:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_CMP_register_T3 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_CMP_register_T3(uint32_t instr, uint32_t pc) {
    // CMP (register) § C2.4.31 T3
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t n        = Rn;
    uint32_t m        = Rm;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if (n == 15 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMP_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute32_RSB_register_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_RSB_register_T1(uint32_t instr, uint32_t pc) {
    // RSB (register) § C2.4.136 T1
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_RSB_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_CMP_immediate_T2 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_CMP_immediate_T2(uint32_t instr, uint32_t pc) {
    // CMP (immediate) § C2.4.30 T2
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n        = Rn;
    uint32_t imm32    = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);

    if (n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMP_immediate(n, imm32);
  }

  /* _DecodeExecute32_SUB_SP_minus_register_T1 {{{4
   * -----------------------------------------
   */
  void _DecodeExecute32_SUB_SP_minus_register_T1(uint32_t instr, uint32_t pc) {
    // SUB (SP minus register) § C2.4.197 T1
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(!(Rd == 0b1111 && S));
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if (d == 13 || (shiftT != SRType_LSL || shiftN > 3))
      THROW_UNPREDICTABLE();

    if ((d == 15 && !S) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(SUB_SP_minus_reg, T1, "d=%u m=%u S=%u shiftT=%u shiftN=%u", d, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_SP_minus_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_SUB_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_SUB_register_T2(uint32_t instr, uint32_t pc) {
    // SUB (register) § C2.4.200 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(!(Rd == 0b1111 && S));
    ASSERT(Rn != 0b1101);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if (d == 13 || (d == 15 && !S) || n == 15 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(SUB_reg, T2, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_PKHBT_PKHTB_T1 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_PKHBT_PKHTB_T1(uint32_t instr, uint32_t pc) {
    // PKHBT, PKHTB § C2.4.105 T1
    TODO_DEC(); // DSP
  }

  /* _DecodeExecute32_ADC_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_ADC_register_T2(uint32_t instr, uint32_t pc) {
    // ADC (register) § C2.4.2 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADC_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_SBC_register_T2 {{{4
   * --------------------------
   */
  void _DecodeExecute32_SBC_register_T2(uint32_t instr, uint32_t pc) {
    // SBC (register) § C2.4.141 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SBC_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_CMN_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_CMN_register_T2(uint32_t instr, uint32_t pc) {
    // CMN (register) § C2.4.29 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t n = Rn;
    uint32_t m = Rm;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if (n == 15 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMN_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute32_ADD_SP_plus_register_T3 {{{4
   * ----------------------------------------
   */
  void _DecodeExecute32_ADD_SP_plus_register_T3(uint32_t instr, uint32_t pc) {
    // ADD (SP plus register) § C2.4.4 T3
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(!(Rd == 0b1111 && S));
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if (d == 13 || (shiftT != SRType_LSL || shiftN > 3))
      THROW_UNPREDICTABLE();

    if ((d == 15 && !S) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_SP_plus_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_ADD_register_T3 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_ADD_register_T3(uint32_t instr, uint32_t pc) {
    // ADD (register) § C2.4.7 T3
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(!(Rd == 0b1111 && S));
    ASSERT(Rn != 0b1101);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if (d == 13 || (d == 15 && !S) || n == 15 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_TEQ_register_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_TEQ_register_T1(uint32_t instr, uint32_t pc) {
    // TEQ (register) § C2.4.210 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t  n         = Rn;
    uint32_t  m         = Rm;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2)|imm2);
    if ((n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(TEQ_reg, T1, "n=%u m=%u shiftT=%u shiftN=%u", n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_TEQ_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute32_EOR_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_EOR_register_T2(uint32_t instr, uint32_t pc) {
    // EOR (register) § C2.4.37 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(!(Rn == 0b1111 && S));
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    uint32_t  m         = Rm;
    bool      setflags  = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2)|imm2);
    if (d == 13 || (d == 15 && !S) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(EOR_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_EOR_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_MVN_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_MVN_register_T2(uint32_t instr, uint32_t pc) {
    // MVN (register) § C2.4.99 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t  d         = Rd;
    uint32_t  m         = Rm;
    bool      setflags  = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2)|imm2);
    if ((d == 13 || d == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(MVN_reg, T1, "d=%u m=%u S=%u shiftT=%u shiftN=%u", d, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MVN_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_ORN_register_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_ORN_register_T1(uint32_t instr, uint32_t pc) {
    // ORN (register) § C2.4.102 T1
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    uint32_t  m         = Rm;
    bool      setflags  = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2)|imm2);
    if ((d == 13 || d == 15) || n == 13 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(ORN_reg, T1, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ORN_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_MOV_register_T3 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_MOV_register_T3(uint32_t instr, uint32_t pc) {
    // MOV (register) § C2.4.90 T3
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    CHECK01(BIT(15), 0);

    // ---- EXECUTE -------------------------------------------------
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if (!setflags && ((imm3<<4) | (imm2<<2) | type) == 0b0000000) {
      if (d == 15 || m == 15 || (d == 13 && m == 13))
        THROW_UNPREDICTABLE();
    } else {
      if ((d == 13 || d == 15) || (m == 13 || m == 15))
        THROW_UNPREDICTABLE();
    }

    _Exec_MOV_register(d, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_BIC_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_BIC_register_T2(uint32_t instr, uint32_t pc) {
    // BIC (register) § C2.4.19 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if ((d == 13 || d == 15) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_BIC_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_TST_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_TST_register_T2(uint32_t instr, uint32_t pc) {
    // TST (register) § C2.4.212 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t n = Rn;
    uint32_t m = Rm;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);

    if ((n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_TST_register(n, m, shiftT, shiftN);
  }

  /* _DecodeExecute32_AND_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_AND_register_T2(uint32_t instr, uint32_t pc) {
    // AND (register) § C2.4.10 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(!(Rd == 0b1111 && S));
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    uint32_t m        = Rm;
    bool     setflags = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2) | imm2);
    if (d == 13 || (d == 15 && !S) || (n == 13 || n == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_AND_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_ORR_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_ORR_register_T2(uint32_t instr, uint32_t pc)  {
    // ORR (register) § C2.4.104 T2
    // ---- DECODE --------------------------------------------------
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t type = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(15), 0);

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    uint32_t  m         = Rm;
    bool      setflags  = !!S;
    auto [shiftT, shiftN] = _DecodeImmShift(type, (imm3<<2)|imm2);
    if ((d == 13 || d == 15) || n == 13 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(ORR_reg, T2, "d=%u n=%u m=%u S=%u shiftT=%u shiftN=%u", d, n, m, setflags, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ORR_register(d, n, m, setflags, shiftT, shiftN);
  }

  /* _DecodeExecute32_10xx {{{4
   * ---------------------
   */
  void _DecodeExecute32_10xx(uint32_t instr, uint32_t pc) {
    // Branches and miscellaneous control
    uint32_t op0 = GETBITS(instr>>16,10,10);
    uint32_t op1 = GETBITS(instr>>16, 6, 9);
    uint32_t op2 = GETBITS(instr>>16, 4, 5);
    uint32_t op3 = GETBITS(instr    ,14,14);
    uint32_t op4 = GETBITS(instr    ,12,12);
    uint32_t op5 = GETBITS(instr    , 8,10);

    uint32_t op2_3_4 = (op2<<2) | (op3<<1) | op4;
    uint32_t op3_4   = (op3<<1) | op4;

    switch (op3_4) {
      case 0b1'0:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b1'1:
        // BL
        _DecodeExecute32_BL_T1(instr, pc);
        break;

      case 0b0'1:
        // B — T4 variant
        _DecodeExecute32_B_T4(instr, pc);
        break;

      case 0b0'0:
        if ((op1 & 0b1110) != 0b1110) {
          // B - T3 variant
          _DecodeExecute32_B_T3(instr, pc);
        } else {
          if (!(op1 & BIT(0))) {
            if (!op0) {
              switch (op2) {
                case 0b00:
                case 0b01:
                  // MSR (register)
                  _DecodeExecute32_MSR_register_T1(instr, pc);
                  break;

                case 0b10:
                  if (!op5) {
                    // Hints
                    _DecodeExecute32_100x_00_xxx0_10_000(instr, pc);
                  } else {
                    // Unallocated
                    UNDEFINED_DEC();
                  }
                  break;

                case 0b11:
                  // Miscellaneous system
                  _DecodeExecute32_1001_00_110_11(instr, pc);
                  break;

                default:
                  abort();
              }
            } else {
              // Unallocated
              UNDEFINED_DEC();
            }
          } else {
            if (!op0) {
              switch (op2) {
                case 0b00:
                case 0b01:
                  // Unallocated
                  UNDEFINED_DEC();
                  break;

                case 0b10:
                case 0b11:
                  // MRS
                  _DecodeExecute32_MRS_T1(instr, pc);
                  break;

                default:
                  abort();
              }
            } else {
              switch (op2) {
                case 0b00:
                case 0b01:
                  // Unallocated
                  UNDEFINED_DEC();
                  break;

                case 0b10:
                case 0b11:
                  // Exception generation
                  _DecodeExecute32_101x_00_1_1x(instr, pc);
                  break;

                default:
                  abort();
              }
            }
          }
        }
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_MRS_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_MRS_T1(uint32_t instr, uint32_t pc) {
    // MRS § C2.4.95 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rd     = GETBITS(instr    , 8,11);
    uint32_t SYSm   = GETBITS(instr    , 0, 7);

    CHECK01(BIT(13) | BIT(16+4), BITS(16+0,16+3));

    uint32_t d = Rd;

    if ((d == 13 || d == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MRS(d, SYSm);
  }

  /* _DecodeExecute32_101x_00_1_1x {{{4
   * -----------------------------
   */
  void _DecodeExecute32_101x_00_1_1x(uint32_t instr, uint32_t pc) {
    // Exception generation
    uint32_t o1 = GETBITS(instr>>16, 4, 4);
    uint32_t o2 = GETBITS(instr    ,13,13);

    switch ((o1<<1) | o2) {
      case 0b0'0:
      case 0b0'1:
      case 0b1'0:
        UNDEFINED_DEC();
        break;

      case 0b1'1:
        _DecodeExecute32_UDF_T2(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_UDF_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_UDF_T2(uint32_t instr, uint32_t pc) {
    // UDF § C2.4.218 T2
    // ---- DECODE --------------------------------------------------
    uint32_t imm4   = GETBITS(instr>>16, 0, 3);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t imm32 = _ZeroExtend((imm4<<12) | imm12, 32);

    // imm32 is for assembly and disassebly only, and is ignored by hardware.

    // ---- EXECUTE -------------------------------------------------
    _Exec_UDF();
  }

  /* _DecodeExecute32_100x_00_xxx0_10_000 {{{4
   * ------------------------------------
   */
  void _DecodeExecute32_100x_00_xxx0_10_000(uint32_t instr, uint32_t pc) {
    // Hints
    uint32_t hint   = GETBITS(instr, 4, 7);
    uint32_t option = GETBITS(instr, 0, 3);

    switch (hint) {
      case 0b0000:
        switch (option) {
          case 0b0000:
            // NOP
            _DecodeExecute32_NOP_T2(instr, pc);
            break;
          case 0b0001:
            // YIELD
            _DecodeExecute32_YIELD_T2(instr, pc);
            break;
          case 0b0010:
            // WFE
            _DecodeExecute32_WFE_T2(instr, pc);
            break;
          case 0b0011:
            // WFI
            _DecodeExecute32_WFI_T2(instr, pc);
            break;
          case 0b0100:
            // SEV
            _DecodeExecute32_SEV_T2(instr, pc);
            break;
          default:
            // Reserved hints behave as NOP.
            _DecodeExecute32_ReservedHint(instr, pc);
            break;
        }
        break;

      case 0b1111:
        // DBG
        _DecodeExecute32_DBG_T1(instr, pc);
        break;

      default:
        // Reserved hints behave as NOP.
        _DecodeExecute32_ReservedHint(instr, pc);
        break;
    }
  }

  /* _DecodeExecute32_DBG_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_DBG_T1(uint32_t instr, uint32_t pc) {
    // DBG § C2.4.33 T1
    // ---- DECODE --------------------------------------------------
    uint32_t option   = GETBITS(instr, 0, 3);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(11) | BIT(13), BITS(16+0,16+3));

    // Any decoding of 'option' is specified by the debug system.

    // ---- EXECUTE -------------------------------------------------
    _Exec_DBG(option);
  }

  /* _DecodeExecute32_NOP_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_NOP_T2(uint32_t instr, uint32_t pc) {
    // NOP § C2.4.100 T2
    // ---- DECODE --------------------------------------------------

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(11) | BIT(13), BITS(16+0,16+3));

    // No additional decoding required.

    TRACEI(NOP, T2, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_NOP();
  }

  /* _DecodeExecute32_YIELD_T2 {{{4
   * -------------------------
   */
  void _DecodeExecute32_YIELD_T2(uint32_t instr, uint32_t pc) {
    // YIELD § C2.4.306 T2
    // ---- DECODE --------------------------------------------------

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(11) | BIT(13), BITS(16+0,16+3));

    // No additional decoding required.

    TRACEI(YIELD, T2, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_YIELD();
  }

  /* _DecodeExecute32_WFE_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_WFE_T2(uint32_t instr, uint32_t pc) {
    // WFE § C2.4.304 T2
    // ---- DECODE --------------------------------------------------

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(11) | BIT(13), BITS(16+0,16+3));

    // No additional decoding required.

    TRACEI(WFE, T2, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_WFE();
  }

  /* _DecodeExecute32_WFI_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_WFI_T2(uint32_t instr, uint32_t pc) {
    // WFI § C2.4.305 T2
    // ---- DECODE --------------------------------------------------

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(11) | BIT(13), BITS(16+0,16+3));

    // No additional decoding required.

    TRACEI(WFI, T2, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_WFI();
  }

  /* _DecodeExecute32_SEV_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_SEV_T2(uint32_t instr, uint32_t pc) {
    // SEV § C2.4.145 T2
    // ---- DECODE --------------------------------------------------

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(11) | BIT(13), BITS(16+0,16+3));

    // No additional decoding required.

    TRACEI(SEV, T2, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_SEV();
  }

  /* _DecodeExecute32_MSR_register_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_MSR_register_T1(uint32_t instr, uint32_t pc) {
    // MSR (register) § C2.4.96 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t mask   = GETBITS(instr    ,10,11);
    uint32_t SYSm   = GETBITS(instr    , 0, 7);

    CHECK01(BITS(8,9) | BIT(13) | BIT(16+4), 0);

    uint32_t n      = Rn;

    if (!_HaveMainExt()) {
      if (mask == 0b00 || (mask != 0b10 && !(SYSm <= 3)))
        CUNPREDICTABLE_UNDEFINED();
    } else {
      if (mask != 0b10)
        CUNPREDICTABLE_UNDEFINED();
    }

    if ((n == 13 || n == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MSR_register(n, mask, SYSm);
  }

  /* _DecodeExecute32_B_T3 {{{4
   * ---------------------
   */
  void _DecodeExecute32_B_T3(uint32_t instr, uint32_t pc) {
    // B § C2.4.15 T3
    // ---- DECODE --------------------------------------------------
    uint32_t S      = GETBITS(instr>>16,10,10);
    uint32_t cond   = GETBITS(instr>>16, 6, 9);
    uint32_t imm6   = GETBITS(instr>>16, 0, 5);
    uint32_t J1     = GETBITS(instr    ,13,13);
    uint32_t J2     = GETBITS(instr    ,11,11);
    uint32_t imm11  = GETBITS(instr    , 0,10);

    ASSERT(GETBITS(cond,1,3) != 0b111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t imm32 = _SignExtend((S<<20) | (J2<<19) | (J1<<18) | (imm6<<12) | (imm11<<1), 21, 32);

    if (_InITBlock())
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_B(imm32);
  }

  /* _DecodeExecute32_B_T4 {{{4
   * ---------------------
   */
  void _DecodeExecute32_B_T4(uint32_t instr, uint32_t pc) {
    // B § C2.4.15 T4
    // ---- DECODE --------------------------------------------------
    uint32_t S      = GETBITS(instr>>16,10,10);
    uint32_t imm10  = GETBITS(instr>>16, 0, 9);
    uint32_t J1     = GETBITS(instr    ,13,13);
    uint32_t J2     = GETBITS(instr    ,11,11);
    uint32_t imm11  = GETBITS(instr    , 0,10);

    bool     I1     = !(J1 ^ S);
    bool     I2     = !(J2 ^ S);
    uint32_t imm32  = _SignExtend((S<<24) | (I1<<23) | (I2<<22) | (imm10<<12) | (imm11 << 1), 25, 32);

    if (_InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_B(imm32);
  }

  /* _DecodeExecute32_BL_T1 {{{4
   * ----------------------
   */
  void _DecodeExecute32_BL_T1(uint32_t instr, uint32_t pc) {
    // BL § C2.4.21 T1
    // ---- DECODE --------------------------------------------------
    uint32_t S      = GETBITS(instr>>16,10,10);
    uint32_t imm10  = GETBITS(instr>>16, 0, 9);
    uint32_t J1     = GETBITS(instr    ,13,13);
    uint32_t J2     = GETBITS(instr    ,11,11);
    uint32_t imm11  = GETBITS(instr    , 0,10);

    uint32_t I1     = (J1 ^ S) ^ 1;
    uint32_t I2     = (J2 ^ S) ^ 1;
    uint32_t imm32  = _SignExtend((S<<24) | (I1<<23) | (I2<<22) | (imm10<<12) | (imm11<<1), 25, 32);

    if (_InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_BL(imm32);
  }

  /* _DecodeExecute32_10x0_0 {{{4
   * -----------------------
   */
  void _DecodeExecute32_10x0_0(uint32_t instr, uint32_t pc) {
    // Data processing (modified immediate)
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
          _DecodeExecute32_AND_immediate_T1(instr, pc);
        } else if (Rd != 15) {
          // AND (immediate) — ANDS variant
          _DecodeExecute32_AND_immediate_T1(instr, pc);
        } else {
          // TST (immediate)
          _DecodeExecute32_TST_immediate_T1(instr, pc);
        }
        break;

      case 0b0001:
        // BIC (immediate)
        _DecodeExecute32_BIC_immediate_T1(instr, pc);
        break;

      case 0b0010:
        if (!S) {
          if (Rn != 15) {
            // ORR (immediate) — ORR variant
            _DecodeExecute32_ORR_immediate_T1(instr, pc);
          } else {
            // MOV (immediate) — MOV variant
            _DecodeExecute32_MOV_immediate_T2(instr, pc);
          }
        } else {
          if (Rn != 15) {
            // ORR (immediate) — ORRS variant
            _DecodeExecute32_ORR_immediate_T1(instr, pc);
          } else {
            // MOV (immediate) — MOVS variant
            _DecodeExecute32_MOV_immediate_T2(instr, pc);
          }
        }
        break;

      case 0b0011:
        if (!S) {
          if (Rn != 15) {
            // ORN (immediate) — Non flag setting variant
            _DecodeExecute32_ORN_immediate_T1(instr, pc);
          } else {
            // MVN (immediate) — MVN variant
            _DecodeExecute32_MVN_immediate_T1(instr, pc);
          }
        } else {
          if (Rn != 15) {
            // ORN (immediate) — Flag setting variant
            _DecodeExecute32_ORN_immediate_T1(instr, pc);
          } else {
            // MVN (immediate) — MVNS variant
            _DecodeExecute32_MVN_immediate_T1(instr, pc);
          }
        }
        break;

      case 0b0100:
        if (!S) {
          // EOR (immediate) — EOR variant
          _DecodeExecute32_EOR_immediate_T1(instr, pc);
        } else {
          if (Rd != 15) {
            // EOR (immediate) — EORS variant
            _DecodeExecute32_EOR_immediate_T1(instr, pc);
          } else {
            // TEQ (immediate)
            _DecodeExecute32_TEQ_immediate_T1(instr, pc);
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
            _DecodeExecute32_ADD_immediate_T3(instr, pc);
          } else {
            // ADD (SP plus immediate) — ADD variant
            _DecodeExecute32_ADD_SP_plus_immediate_T3(instr, pc);
          }
        } else if (Rd == 15) {
          // CMN (immediate)
          _DecodeExecute32_CMN_immediate_T1(instr, pc);
        } else {
          if (Rn != 0b1101) {
            // ADD (immediate) — ADDS variant
            _DecodeExecute32_ADD_immediate_T3(instr, pc);
          } else {
            // ADD (SP plus immediate) — ADDS variant
            _DecodeExecute32_ADD_SP_plus_immediate_T3(instr, pc);
          }
        }
        break;

      case 0b1010:
        // ADC (immediate)
        _DecodeExecute32_ADC_immediate_T1(instr, pc);
        break;

      case 0b1011:
        // SBC (immediate)
        _DecodeExecute32_SBC_immediate_T1(instr, pc);
        break;

      case 0b1101:
        if (!S) {
          if (Rn != 0b1101) {
            // SUB (immediate) — SUB variant
            _DecodeExecute32_SUB_immediate_T3(instr, pc);
          } else {
            // SUB (SP minus immediate) — SUB variant
            _DecodeExecute32_SUB_SP_minus_immediate_T2(instr, pc);
          }
        } else if (Rd == 15) {
          // CMP (immediate)
          _DecodeExecute32_CMP_immediate_T2(instr, pc);
        } else {
          if (Rn != 0b1101) {
            // SUB (immediate) — SUBS variant
            _DecodeExecute32_SUB_immediate_T3(instr, pc);
          } else {
            // SUB (SP minus immediate) — SUBS variant
            _DecodeExecute32_SUB_SP_minus_immediate_T2(instr, pc);
          }
        }
        break;

      case 0b1110:
        // RSB (immediate)
        _DecodeExecute32_RSB_immediate_T2(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_RSB_immediate_T2 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_RSB_immediate_T2(uint32_t instr, uint32_t pc) {
    // RSB (immediate) § C2.4.135 T2
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !!S;
    uint32_t imm32    = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);

    if ((d == 13 || d == 15) || (n == 13 || n == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_RSB_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute32_SUB_SP_minus_immediate_T2 {{{4
   * ------------------------------------------
   */
  void _DecodeExecute32_SUB_SP_minus_immediate_T2(uint32_t instr, uint32_t pc) {
    // SUB (SP minus immediate) § C2.4.196 T2
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rd == 0b1111 && S));

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    bool     setflags = !!S;
    uint32_t imm32    = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);

    if (d == 15 && !S)
      THROW_UNPREDICTABLE();

    TRACEI(SUB_SP_minus_imm, T2, "d=%u S=%u imm32=0x%x", d, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_SP_minus_immediate(d, setflags, imm32);
  }

  /* _DecodeExecute32_CMN_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_CMN_immediate_T1(uint32_t instr, uint32_t pc) {
    // CMN (immediate) § C2.4.28 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n        = Rn;
    uint32_t imm32    = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);

    if (n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_CMN_immediate(n, imm32);
  }

  /* _DecodeExecute32_SBC_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_SBC_immediate_T1(uint32_t instr, uint32_t pc) {
    // SBC (immediate) § C2.4.140 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !!S;
    uint32_t imm32    = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);

    if ((d == 13 || d == 15) || (n == 13 || n == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SBC_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute32_ADC_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_ADC_immediate_T1(uint32_t instr, uint32_t pc) {
    // ADC (immediate) § C2.4.1 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !!S;
    uint32_t imm32    = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);

    if ((d == 13 || d == 15) || (n == 13 || n == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADC_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute32_ADD_SP_plus_immediate_T3 {{{4
   * -----------------------------------------
   */
  void _DecodeExecute32_ADD_SP_plus_immediate_T3(uint32_t instr, uint32_t pc) {
    // ADD (SP plus immediate) § C2.4.3 T3
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rd == 0b1111 && S));

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    bool     setflags = !!S;
    uint32_t imm32    = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);

    if (d == 15 && !S)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_SP_plus_immediate(d, setflags, imm32);
  }

  /* _DecodeExecute32_TEQ_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_TEQ_immediate_T1(uint32_t instr, uint32_t pc) {
    // TEQ (immediate) § C2.4.209 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n        = Rn;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));

    if (n == 13 || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_TEQ_immediate(n, imm32, carry);
  }

  /* _DecodeExecute32_EOR_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_EOR_immediate_T1(uint32_t instr, uint32_t pc) {
    // EOR (immediate) § C2.4.36 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rd == 0b1111 && S));

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));

    if (d == 13 || (d == 15 && !S) || (n == 13 || n == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_EOR_immediate(d, n, setflags, imm32, carry);
  }

  /* _DecodeExecute32_MVN_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_MVN_immediate_T1(uint32_t instr, uint32_t pc) {
    // MVN (immediate) § C2.4.98 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    bool     setflags = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));

    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MVN_immediate(d, setflags, imm32, carry);
  }

  /* _DecodeExecute32_ORN_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_ORN_immediate_T1(uint32_t instr, uint32_t pc) {
    // ORN (immediate) § C2.4.101 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));

    if ((d == 13 || d == 15) || n == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ORN_immediate(d, n, setflags, imm32, carry);
  }

  /* _DecodeExecute32_TST_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_TST_immediate_T1(uint32_t instr, uint32_t pc) {
    // TST (immediate) § C2.4.211 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n = Rn;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));
    if (n == 13 || n == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_TST_immediate(n, imm32, carry);
  }

  /* _DecodeExecute32_SUB_immediate_T3 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_SUB_immediate_T3(uint32_t instr, uint32_t pc) {
    // SUB (immediate) § C2.4.198 T3
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rd == 0b1111 && S));
    ASSERT(Rn != 0b1101);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    bool      setflags  = !!S;
    uint32_t  imm32     = _T32ExpandImm((i<<11) | (imm3<<8) | imm8);
    if (d == 13 || (d == 15 && !S) || n == 15)
      THROW_UNPREDICTABLE();

    TRACEI(SUB_imm, T3, "d=%u n=%u[0x%x] S=%u imm32=0x%x", d, n, _GetR(n), setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute32_ADD_immediate_T3 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_ADD_immediate_T3(uint32_t instr, uint32_t pc) {
    // ADD (immediate) § C2.4.5 T3
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rd == 0b1111 && S));
    ASSERT(Rn != 0b1101);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    bool      setflags  = !!S;
    uint32_t  imm32     = _T32ExpandImm((i<<11)|(imm3<<8)|imm8);

    if (d == 13 || (d == 15 && !S) || n == 15)
      THROW_UNPREDICTABLE();

    TRACEI(ADD_imm, T3, "d=%u n=%u S=%u imm32=0x%x", d, n, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute32_BIC_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_BIC_immediate_T1(uint32_t instr, uint32_t pc) {
    // BIC (immediate) § C2.4.18 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11)|(imm3<<8)|imm8, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 13 || d == 15 || n == 13 || n == 15)
      THROW_UNPREDICTABLE();

    TRACEI(BIC_imm, T1, "d=%u n=%u S=%u imm32=0x%x carry=%u", d, n, setflags, imm32, carry);

    // ---- EXECUTE -------------------------------------------------
    _Exec_BIC_immediate(d, n, setflags, imm32, carry);
  }

  /* _DecodeExecute32_ORR_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_ORR_immediate_T1(uint32_t instr, uint32_t pc) {
    // ORR (immediate) § C2.4.103 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    uint32_t n        = Rn;
    bool     setflags = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11)|(imm3<<8)|imm8, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 13 || d == 15 || n == 13)
      THROW_UNPREDICTABLE();

    TRACEI(ORR_imm, T1, "d=%u n=%u S=%u imm32=0x%x carry=%u", d, n, setflags, imm32, carry);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ORR_immediate(d, n, setflags, imm32, carry);
  }

  /* _DecodeExecute32_MOV_immediate_T2 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_MOV_immediate_T2(uint32_t instr, uint32_t pc) {
    // MOV (immediate) § C2.4.89 T2
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    bool     setflags = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    TRACEI(MOV_imm, T2, "d=%u S=%u imm32=0x%x carry=%u", d, setflags, imm32, carry);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOV_immediate(d, setflags, imm32, carry);
  }

  /* _DecodeExecute32_AND_immediate_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_AND_immediate_T1(uint32_t instr, uint32_t pc) {
    // AND (immediate) § C2.4.9 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t S    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rd == 0b1111 && S));
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    bool      setflags  = !!S;
    auto [imm32, carry] = _T32ExpandImm_C((i<<11) | (imm3<<8) | imm8, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 13 || (d == 15 && !S) || (n == 13 || n == 15))
      THROW_UNPREDICTABLE();

    TRACEI(AND_imm, T1, "d=%u n=%u S=%u imm32=0x%x carry=%u", d, n, setflags, imm32, carry);

    // ---- EXECUTE -------------------------------------------------
    _Exec_AND_immediate(d, n, setflags, imm32, carry);
  }

  /* _DecodeExecute32_10x1_0 {{{4
   * -----------------------
   */
  void _DecodeExecute32_10x1_0(uint32_t instr, uint32_t pc) {
    // Data processing (plain binary immediate)
    uint32_t op0 = GETBITS(instr>>16, 8, 8);
    uint32_t op1 = GETBITS(instr>>16, 5, 6);

    switch ((op0<<2)|op1) {
      case 0b0'00:
      case 0b0'01:
        // Data processing (simple immediate)
        _DecodeExecute32_10x1_0_00x(instr, pc);
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

  /* _DecodeExecute32_10x1_0_00x {{{4
   * ---------------------------
   */
  void _DecodeExecute32_10x1_0_00x(uint32_t instr, uint32_t pc) {
    // Data processing (simple immediate)
    uint32_t o1 = GETBITS(instr>>16, 7, 7);
    uint32_t o2 = GETBITS(instr>>16, 5, 5);
    uint32_t Rn = GETBITS(instr>>16, 0, 3);

    switch ((o1<<1) | o2) {
      case 0b0'0:
        if (Rn == 0b1111) {
          // ADR - T3
          _DecodeExecute32_ADR_T3(instr, pc);
        } else if (Rn == 0b1101) {
          // ADD (SP plus immediate) (T4)
          _DecodeExecute32_ADD_SP_plus_immediate_T4(instr, pc);
          break;
        } else {
          // ADD (immediate)
          _DecodeExecute32_ADD_immediate_T4(instr, pc);
          break;
        }
        break;

      case 0b0'1:
      case 0b1'0:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b1'1:
        if (Rn == 0b1111) {
          // ADR - T2
          _DecodeExecute32_ADR_T2(instr, pc);
        } else if (Rn == 0b1101) {
          // SUB (SP minus immediate) (T3)
          _DecodeExecute32_SUB_SP_minus_immediate_T3(instr, pc);
        } else {
          // SUB (immediate)
          _DecodeExecute32_SUB_immediate_T4(instr, pc);
        }
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_SUB_immediate_T4 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_SUB_immediate_T4(uint32_t instr, uint32_t pc) {
    // SUB (immediate) § C2.4.198 T4
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);
    ASSERT(Rn != 0b1101);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    bool      setflags  = false;
    uint32_t  imm32     = _ZeroExtend((i<<11)|(imm3<<8)|imm8, 32);

    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    TRACEI(SUB_imm, T4, "d=%u n=%u S=%u imm32=0x%x", d, n, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute32_ADD_immediate_T4 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_ADD_immediate_T4(uint32_t instr, uint32_t pc) {
    // ADD (immediate) § C2.4.5 T4
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);
    ASSERT(Rn != 0b1101);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  d         = Rd;
    uint32_t  n         = Rn;
    bool      setflags  = false;
    uint32_t  imm32     = _ZeroExtend((i<<11)|(imm3<<8)|imm8, 32);

    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    TRACEI(ADD_imm, T4, "d=%u n=%u S=%u imm32=0x%x", d, n, setflags, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_immediate(d, n, setflags, imm32);
  }

  /* _DecodeExecute32_SUB_SP_minus_immediate_T3 {{{4
   * ------------------------------------------
   */
  void _DecodeExecute32_SUB_SP_minus_immediate_T3(uint32_t instr, uint32_t pc) {
    // SUB (SP minus immediate) § C2.4.196 T3
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    bool     setflags = false;
    uint32_t imm32    = _ZeroExtend((i<<11) | (imm3<<8) | imm8, 32);

    if (d == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SUB_SP_minus_immediate(d, setflags, imm32);
  }

  /* _DecodeExecute32_ADD_SP_plus_immediate_T4 {{{4
   * -----------------------------------------
   */
  void _DecodeExecute32_ADD_SP_plus_immediate_T4(uint32_t instr, uint32_t pc) {
    // ADD (SP plus immediate) § C2.4.3 T4
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d        = Rd;
    bool     setflags = false;
    uint32_t imm32    = _ZeroExtend((i<<11) | (imm3<<8) | imm8, 32);

    if (d == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADD_SP_plus_immediate(d, setflags, imm32);
  }

  /* _DecodeExecute32_ADR_T3 {{{4
   * -----------------------
   */
  void _DecodeExecute32_ADR_T3(uint32_t instr, uint32_t pc) {
    // ADR § C2.4.8 T3
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t d      = Rd;
    uint32_t imm32  = _ZeroExtend((i<<11) | (imm3<<8) | imm8, 32);
    bool     add    = true;
    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADR(d, imm32, add);
  }

  /* _DecodeExecute32_ADR_T2 {{{4
   * -----------------------
   */
  void _DecodeExecute32_ADR_T2(uint32_t instr, uint32_t pc) {
    // ADR § C2.4.8 T2
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    if (!_HaveMainExt())
      THROW_UNPREDICTABLE();

    uint32_t d      = Rd;
    uint32_t imm32  = _ZeroExtend((i<<11) | (imm3<<8) | imm8, 32);
    bool     add    = false;

    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_ADR(d, imm32, add);
  }

  /* _DecodeExecute32_10x1_0_010 {{{4
   * ---------------------------
   */
  void _DecodeExecute32_10x1_0_010(uint32_t instr, uint32_t pc) {
    // Move Wide (16-bit immediate)
    uint32_t o1 = GETBITS(instr>>16, 7, 7);
    if (!o1) {
      // MOV (immediate)
      _DecodeExecute32_MOV_immediate_T3(instr, pc);
    } else {
      // MOVT
      _DecodeExecute32_MOVT_T1(instr, pc);
    }
  }

  /* _DecodeExecute32_MOVT_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_MOVT_T1(uint32_t instr, uint32_t pc) {
    // MOVT § C2.4.92 T1
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t imm4 = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    uint32_t d      = Rd;
    uint32_t imm16  = (imm4<<12) | (i<<11) | (imm3<<8) | imm8;

    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOVT(d, imm16);
  }

  /* _DecodeExecute32_MOV_immediate_T3 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_MOV_immediate_T3(uint32_t instr, uint32_t pc) {
    // MOV (immediate) § C2.4.89 T3
    // ---- DECODE --------------------------------------------------
    uint32_t i    = GETBITS(instr>>16,10,10);
    uint32_t imm4 = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    uint32_t d        = Rd;
    bool     setflags = false;
    uint32_t imm32    = (imm4<<12) | (i<<11) | (imm3<<8) | imm8;
    bool     carry    = UNKNOWN_VAL(false);

    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    TRACEI(MOV_imm, T3, "d=%u imm32=0x%x", d, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_MOV_immediate(d, setflags, imm32, carry);
  }

  /* _DecodeExecute32_10x1_0_1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_10x1_0_1(uint32_t instr, uint32_t pc) {
    // Saturate, Bitfield
    uint32_t op1  = GETBITS(instr>>16, 5, 7);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t imm3_imm2 = (imm3<<2) | imm2;

    switch (op1) {
      case 0b000:
        // SSAT - Logical shift left variant
        _DecodeExecute32_SSAT_T1(instr, pc);
        break;

      case 0b001:
        if (imm3_imm2) {
          // SSAT - Arithmetic shift right variant
          _DecodeExecute32_SSAT_T1(instr, pc);
        } else {
          // SSAT16
          TODO_DEC(); // DSP
        }
        break;

      case 0b010:
        // SBFX
        _DecodeExecute32_SBFX_T1(instr, pc);
        break;

      case 0b011:
        if (Rn != 0b1111) {
          // BFI
          _DecodeExecute32_BFI_T1(instr, pc);
        } else {
          // BFC
          _DecodeExecute32_BFC_T1(instr, pc);
        }
        break;

      case 0b100:
        // USAT - Logical shift left variant
        _DecodeExecute32_USAT_T1(instr, pc);
        break;

      case 0b101:
        if (imm3_imm2) {
          // USAT - Arithmetic shift right variant
          _DecodeExecute32_USAT_T1(instr, pc);
        } else {
          // USAT16
          TODO_DEC(); // DSP
        }
        break;

      case 0b110:
        // UBFX
        _DecodeExecute32_UBFX_T1(instr, pc);
        break;

      case 0b111:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_SSAT_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_SSAT_T1(uint32_t instr, uint32_t pc) {
    // SSAT § C2.4.169 T1
    // ---- DECODE --------------------------------------------------
    uint32_t sh       = GETBITS(instr>>16, 5, 5);
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t imm3     = GETBITS(instr    ,12,14);
    uint32_t Rd       = GETBITS(instr    , 8,11);
    uint32_t imm2     = GETBITS(instr    , 6, 7);
    uint32_t satImm   = GETBITS(instr    , 0, 4);

    ASSERT(!sh || imm3 || imm2);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(5) | BIT(16+10), 0);

    uint32_t d            = Rd;
    uint32_t n            = Rn;
    uint32_t saturateTo   = satImm;
    auto [shiftT, shiftN] = _DecodeImmShift(sh<<1, (imm3<<2) | imm2);

    if ((d == 13 || d == 15) || (n == 13 || n == 15))
      THROW_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SSAT(d, n, saturateTo, shiftT, shiftN);
  }

  /* _DecodeExecute32_USAT_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_USAT_T1(uint32_t instr, uint32_t pc) {
    // USAT § C2.4.237 T1
    // ---- DECODE --------------------------------------------------
    uint32_t sh       = GETBITS(instr>>16, 5, 5);
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t imm3     = GETBITS(instr    ,12,14);
    uint32_t Rd       = GETBITS(instr    , 8,11);
    uint32_t imm2     = GETBITS(instr    , 6, 7);
    uint32_t satImm   = GETBITS(instr    , 0, 4);

    ASSERT(!sh || imm3 || imm2);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(5) | BIT(16+10), 0);

    uint32_t d            = Rd;
    uint32_t n            = Rn;
    uint32_t saturateTo   = satImm;
    auto [shiftT, shiftN] = _DecodeImmShift(sh<<1, (imm3<<2) | imm2);

    if ((d == 13 || d == 15) || (n == 13 || n == 15))
      THROW_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_USAT(d, n, saturateTo, shiftT, shiftN);
  }

  /* _DecodeExecute32_SBFX_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_SBFX_T1(uint32_t instr, uint32_t pc) {
    // SBFX § C2.4.142 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t imm3     = GETBITS(instr    ,12,14);
    uint32_t Rd       = GETBITS(instr    , 8,11);
    uint32_t imm2     = GETBITS(instr    , 6, 7);
    uint32_t widthm1  = GETBITS(instr    , 0, 4);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(5) | BIT(16+10), 0);

    uint32_t d            = Rd;
    uint32_t n            = Rn;
    uint32_t lsbit        = (imm3<<2) | imm2;
    uint32_t widthminus1  = widthm1;
    uint32_t msbit        = lsbit + widthminus1;

    if (msbit > 31)
      CUNPREDICTABLE_UNDEFINED();

    if ((d == 13 || d == 15) || (n == 13 || n == 15))
      THROW_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_SBFX(d, n, lsbit, widthminus1, msbit);
  }

  /* _DecodeExecute32_UBFX_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_UBFX_T1(uint32_t instr, uint32_t pc) {
    // UBFX § C2.4.217 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn       = GETBITS(instr>>16, 0, 3);
    uint32_t imm3     = GETBITS(instr    ,12,14);
    uint32_t Rd       = GETBITS(instr    , 8,11);
    uint32_t imm2     = GETBITS(instr    , 6, 7);
    uint32_t widthm1  = GETBITS(instr    , 0, 4);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(5) | BIT(16+10), 0);

    uint32_t d            = Rd;
    uint32_t n            = Rn;
    uint32_t lsbit        = (imm3<<2) | imm2;
    uint32_t widthminus1  = widthm1;
    uint32_t msbit        = lsbit + widthminus1;

    if (msbit > 31)
      CUNPREDICTABLE_UNDEFINED();

    if ((d == 13 || d == 15) || (n == 13 || n == 15))
      THROW_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_UBFX(d, n, lsbit, widthminus1, msbit);
  }

  /* _DecodeExecute32_1001_00_110_11 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_1001_00_110_11(uint32_t instr, uint32_t pc) {
    // Miscellaneous system
    uint32_t opc = GETBITS(instr, 4, 7);

    switch (opc) {
      case 0b0000:
      case 0b0001:
      case 0b0011:
      case 0b0111:
      case 0b1000:
      case 0b1001:
      case 0b1010:
      case 0b1011:
      case 0b1100:
      case 0b1101:
      case 0b1110:
      case 0b1111:
        // Unallocated
        UNDEFINED_DEC();
        break;

      case 0b0010:
        // CLREX
        _DecodeExecute32_CLREX_T1(instr, pc);
        break;

      case 0b0100:
        // DSB
        _DecodeExecute32_DSB_T1(instr, pc);
        break;

      case 0b0101:
        // DMB
        _DecodeExecute32_DMB_T1(instr, pc);
        break;

      case 0b0110:
        // ISB
        _DecodeExecute32_ISB_T1(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_CLREX_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_CLREX_T1(uint32_t instr, uint32_t pc) {
    // CLREX § C2.4.26 T1
    // ---- DECODE --------------------------------------------------

    CHECK01(BIT(13), BITS(0,3) | BITS(8,11) | BITS(16+0,16+3));
    // No additional decoding required.

    // ---- EXECUTE -------------------------------------------------
    _Exec_CLREX();
  }

  /* _DecodeExecute32_DSB_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_DSB_T1(uint32_t instr, uint32_t pc) {
    // DSB § C2.4.35 T1
    // ---- DECODE --------------------------------------------------
    uint32_t option = GETBITS(instr, 0, 3);

    CHECK01(BIT(13), BITS(8,11) | BITS(16+0,16+3));

    TRACEI(DSB, T1, "option=0x%x", option);

    // ---- EXECUTE -------------------------------------------------
    _Exec_DSB(option);
  }

  /* _DecodeExecute32_DMB_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_DMB_T1(uint32_t instr, uint32_t pc) {
    // DMB § C2.4.34 T1
    // ---- DECODE --------------------------------------------------
    uint32_t option = GETBITS(instr, 0, 3);

    CHECK01(BIT(13), BITS(8,11) | BITS(16+0,16+3));

    TRACEI(DMB, T1, "option=0x%x", option);

    // ---- EXECUTE -------------------------------------------------
    _Exec_DMB(option);
  }

  /* _DecodeExecute32_ISB_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_ISB_T1(uint32_t instr, uint32_t pc) {
    // ISB § C2.4.40 T1
    // ---- DECODE --------------------------------------------------
    uint32_t option = GETBITS(instr, 0, 3);

    CHECK01(BIT(13), BITS(8,11) | BITS(16+0,16+3));

    TRACEI(ISB, T1, "option=0x%x", option);

    // ---- EXECUTE -------------------------------------------------
    _Exec_ISB(option);
  }

  /* _DecodeExecute32_BFI_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_BFI_T1(uint32_t instr, uint32_t pc) {
    // BFI § C2.4.17 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rd   = GETBITS(instr    , 8,11);
    uint32_t imm3 = GETBITS(instr    ,12,14);
    uint32_t imm2 = GETBITS(instr    , 6, 7);
    uint32_t msb  = GETBITS(instr    , 0, 4);

    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(5) | BIT(16+10), 0);

    uint32_t d      = Rd;
    uint32_t n      = Rn;
    uint32_t msbit  = msb;
    uint32_t lsbit  = (imm3<<2) | imm2;
    if (msbit < lsbit)
      CUNPREDICTABLE_UNDEFINED();

    if (d == 13 || d == 15 || n == 13)
      THROW_UNPREDICTABLE();

    TRACEI(BFI, T1, "d=%u n=%u msbit=%u lsbit=%u", d, n, msbit, lsbit);

    // ---- EXECUTE -------------------------------------------------
    _Exec_BFI(d, n, msbit, lsbit);
  }

  /* _DecodeExecute32_BFC_T1 {{{4
   * -----------------------
   */
  void _DecodeExecute32_BFC_T1(uint32_t instr, uint32_t pc) {
    // BFC § C2.4.16 T1
    // ---- DECODE --------------------------------------------------
    uint32_t imm3 = GETBITS(instr,12,14);
    uint32_t Rd   = GETBITS(instr, 8,11);
    uint32_t imm2 = GETBITS(instr, 6, 7);
    uint32_t msb  = GETBITS(instr, 0, 4);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    CHECK01(BIT(5) | BIT(16+10), 0);

    uint32_t d      = Rd;
    uint32_t msbit  = msb;
    uint32_t lsbit  = (imm3<<2)|imm2;

    if (msbit < lsbit)
      CUNPREDICTABLE_UNDEFINED();

    if (d == 13 || d == 15)
      THROW_UNPREDICTABLE();

    TRACEI(BFC, T1, "d=%u msbit=%u lsbit=%u", d, msbit, lsbit);

    // ---- EXECUTE -------------------------------------------------
    _Exec_BFC(d, msbit, lsbit);
  }

  /* _DecodeExecute32_1100_xxxxx {{{4
   * ---------------------------
   */
  //                         87654/BA9876
  void _DecodeExecute32_1100_xxxxx(uint32_t instr, uint32_t pc) {
    // Load/store single
    uint32_t op0 = GETBITS(instr>>16, 7, 8);
    uint32_t op1 = GETBITS(instr>>16, 4, 4);
    uint32_t op2 = GETBITS(instr>>16, 0, 3);
    uint32_t op3 = GETBITS(instr    , 6,11);

    if (op2 == 0b1111) {
      if (!(op0 & BIT(1))) {
        // Load, unsigned (literal)
        _DecodeExecute32_1100_0xxxx_1111(instr, pc);
      } else {
        if (!op1) {
          // ???
          UNDEFINED_DEC();
        } else {
          // Load, signed (literal)
          _DecodeExecute32_1100_1xxx1_1111(instr, pc);
        }
      }
    } else
      switch (op0) {
        case 0b00:
          if (!op3) {
            // Load/store, unsigned (register offset)
            _DecodeExecute32_1100_00xxx_xxxx_000000(instr, pc);
          } else if (  op3 == 0b000001
                    || (op3 & 0b111110) == 0b000010
                    || (op3 & 0b111100) == 0b000100
                    || (op3 & 0b111000) == 0b001000
                    || (op3 & 0b110000) == 0b010000
                    || (op3 & 0b110100) == 0b100000) {
            // Unallocated
            UNDEFINED_DEC();
          } else if ((op3 & 0b110100) == 0b100100) {
            // Load/store, unsigned (immediate, post-indexed)
            _DecodeExecute32_1100_00xxx_10x1xx(instr, pc);
          } else if ((op3 & 0b111100) == 0b110000) {
            // Load/store, unsigned (negative immediate)
            _DecodeExecute32_1100_00xxx_1100xx(instr, pc);
          } else if ((op3 & 0b111100) == 0b111000) {
            // Load/store, unsigned (unprivileged)
            _DecodeExecute32_1100_00xxx_1110xx(instr, pc);
          } else if ((op3 & 0b110100) == 0b110100) {
            // Load/store, unsigned (immediate, pre-indexed)
            _DecodeExecute32_1100_00xxx_11x1xx(instr, pc);
          } else {
            // ?
            UNDEFINED_DEC();
          }
          break;

        case 0b01:
          // Load/store, unsigned (positive immediate)
          _DecodeExecute32_1100_01xxx_xxxx(instr, pc);
          break;

        case 0b10:
          if (!op3) {
            // Load/store, signed (register offset)
            _DecodeExecute32_1100_10xxx_000000(instr, pc);
          } else if (op3 == 0b000001
                  || (op3 & 0b111110) == 0b000010
                  || (op3 & 0b111100) == 0b000100
                  || (op3 & 0b111000) == 0b001000
                  || (op3 & 0b110000) == 0b010000
                  || (op3 & 0b110100) == 0b100000) {
            // Unallocated
            UNDEFINED_DEC();
          } else if ((op3 & 0b110100) == 0b100100) {
            // Load/store, signed (immediate, post-indexed)
            _DecodeExecute32_1100_10xxx_10x100(instr, pc);
          } else if ((op3 & 0b111100) == 0b110000) {
            // Load/store, signed (negative immediate)
            _DecodeExecute32_1100_10xxx_1100xx(instr, pc);
          } else if ((op3 & 0b111100) == 0b111000) {
            // Load/store, signed (unprivileged)
            _DecodeExecute32_1100_10xxx_1110xx(instr, pc);
          } else if ((op3 & 0b110100) == 0b110100) {
            // Load/store, signed (immediate, pre-indexed)
            _DecodeExecute32_1100_10xxx_11x1xx(instr, pc);
          } else {
            // ?
            UNDEFINED_DEC();
          }
          break;

        case 0b11:
          // Load/store, signed (positive immediate)
          _DecodeExecute32_1100_11xxx_xxxxxx(instr, pc);
          break;
      }
  }

  /* _DecodeExecute32_1100_11xxx_xxxxxx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_11xxx_xxxxxx(uint32_t instr, uint32_t pc) {
    // Load/store, signed (positive immediate)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    switch (size) {
      case 0b00:
        if (Rt != 0b1111) {
          // LDRSB (immediate)
          _DecodeExecute32_LDRSB_immediate_T1(instr, pc);
        } else {
          // PLI (immediate, literal)
          _DecodeExecute32_PLI_immediate_literal_T1(instr, pc);
        }
        break;

      case 0b01:
        if (Rt != 0b1111) {
          // LDRSH (immediate)
          _DecodeExecute32_LDRSH_immediate_T1(instr, pc);
        } else {
          // Reserved hint, behaves as NOP
          _DecodeExecute32_ReservedHint(instr, pc);
        }
        break;

      case 0b10:
      case 0b11:
        // ?
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_PLI_immediate_literal_T1 {{{4
   * -----------------------------------------
   */
  void _DecodeExecute32_PLI_immediate_literal_T1(uint32_t instr, uint32_t pc) {
    // PLI (immediate, literal) § C2.4.109 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = true;

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLI_immediate_literal(n, imm32, add);
  }

  /* _DecodeExecute32_LDRSB_immediate_T1 {{{4
   * -----------------------------------
   */
  void _DecodeExecute32_LDRSB_immediate_T1(uint32_t instr, uint32_t pc) {
    // LDRSB (immediate) § C2.4.68 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     index  = true, add = true, wback = false;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDRSH_immediate_T1 {{{4
   * -----------------------------------
   */
  void _DecodeExecute32_LDRSH_immediate_T1(uint32_t instr, uint32_t pc) {
    // LDRSH (immediate) § C2.4.72 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     index  = true, add = true, wback = false;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_1100_10xxx_11x1xx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_10xxx_11x1xx(uint32_t instr, uint32_t pc) {
    // Load/store, signed (immediate, pre-indexed)
    uint32_t size = GETBITS(instr>>16, 5, 6);

    switch (size) {
      case 0b00:
        // LDRSB (immediate)
        _DecodeExecute32_LDRSB_immediate_T2(instr, pc);
        break;

      case 0b01:
        // LDRSH (immediate)
        _DecodeExecute32_LDRSH_immediate_T2(instr, pc);
        break;

      case 0b10:
      case 0b11:
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_1100_10xxx_1110xx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_10xxx_1110xx(uint32_t instr, uint32_t pc) {
    // Load/store, signed (unprivileged)
    uint32_t size = GETBITS(instr>>16, 5, 6);

    switch (size) {
      case 0b00:
        // LDRSBT
        _DecodeExecute32_LDRSBT_T1(instr, pc);
        break;

      case 0b01:
        // LDRSHT
        _DecodeExecute32_LDRSHT_T1(instr, pc);
        break;

      case 0b10:
      case 0b11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_LDRSBT_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_LDRSBT_T1(uint32_t instr, uint32_t pc) {
    // LDRSBT § C2.4.71 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t         = Rt;
    uint32_t n         = Rn;
    bool     postindex = false, add = true, registerForm = false;
    uint32_t imm32     = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSBT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_LDRSHT_T1 {{{4
   * --------------------------
   */
  void _DecodeExecute32_LDRSHT_T1(uint32_t instr, uint32_t pc) {
    // LDRSHT § C2.4.75 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t         = Rt;
    uint32_t n         = Rn;
    bool     postindex = false, add = true, registerForm = false;
    uint32_t imm32     = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSHT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_1100_10xxx_1100xx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_10xxx_1100xx(uint32_t instr, uint32_t pc) {
    // Load/store, signed (negative immediate)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    switch (size) {
      case 0b00:
        if (Rt != 0b1111) {
          // LDRSB (immediate)
          _DecodeExecute32_LDRSB_immediate_T2(instr, pc);
        } else {
          // PLI (immediate, literal)
          _DecodeExecute32_PLI_immediate_literal_T2(instr, pc);
        }
        break;

      case 0b01:
        if (Rt != 0b1111) {
          // LDRSH (immediate)
          _DecodeExecute32_LDRSH_immediate_T2(instr, pc);
        } else {
          // Reserved hint, behaves as NOP
          _DecodeExecute32_ReservedHint(instr, pc);
        }
        break;

      case 0b10:
      case 0b11:
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_1100_10xxx_10x100 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_10xxx_10x100(uint32_t instr, uint32_t pc) {
    // Load/store, signed (immediate, post-indexed)
    uint32_t size = GETBITS(instr>>16, 5, 6);

    switch (size) {
      case 0b00:
        // LDRSB (immediate)
        _DecodeExecute32_LDRSB_immediate_T2(instr, pc);
        break;

      case 0b01:
        // LDRSH (immediate)
        _DecodeExecute32_LDRSH_immediate_T2(instr, pc);
        break;

      case 0b10:
      case 0b11:
        // Unallocated.
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_LDRSB_immediate_T2 {{{4
   * -----------------------------------
   */
  void _DecodeExecute32_LDRSB_immediate_T2(uint32_t instr, uint32_t pc) {
    // LDRSB (immediate) § C2.4.68 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rt == 0b1111 && P && !U && !W));
    ASSERT(Rn != 0b1111);
    ASSERT(!(P && U && !W));
    ASSERT(P || W);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P;
    bool     add    = !!U;
    bool     wback  = !!W;

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    if (t == 13 || (t == 15 && W))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDRSH_immediate_T2 {{{4
   * -----------------------------------
   */
  void _DecodeExecute32_LDRSH_immediate_T2(uint32_t instr, uint32_t pc) {
    // LDRSH (immediate) § C2.4.72 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rt == 0b1111 && P && !U && !W));
    ASSERT(Rn != 0b1111);
    ASSERT(!(P && U && !W));
    ASSERT(P || W);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P;
    bool     add    = !!U;
    bool     wback  = !!W;

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    if (t == 13 || (t == 15 && W))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_1100_10xxx_000000 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_10xxx_000000(uint32_t instr, uint32_t pc) {
    // Load/store, signed (register offset)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    switch (size) {
      case 0b00:
        if (Rt != 0b1111) {
          // LDRSB (register)
          _DecodeExecute32_LDRSB_register_T2(instr, pc);
        } else {
          // PLI (register)
          _DecodeExecute32_PLI_register_T1(instr, pc);
        }
        break;

      case 0b01:
        if (Rt != 0b1111) {
          // LDRSH (register)
          _DecodeExecute32_LDRSH_register_T2(instr, pc);
        } else {
          // Reserved hint, behaves as NOP.
          _DecodeExecute32_ReservedHint(instr, pc);
        }
        break;

      case 0b10:
      case 0b11:
        UNDEFINED_DEC();
        break;
    }
  }

  /* _DecodeExecute32_PLI_register_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_PLI_register_T1(uint32_t instr, uint32_t pc) {
    // PLI (register) § C2.4.110 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     add    = true;
    SRType   shiftT = SRType_LSL;
    uint32_t shiftN = imm2;

    if (m == 13 || m == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLI_register(n, m, add, shiftT, shiftN);
  }

  /* _DecodeExecute32_LDRSB_register_T2 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_LDRSB_register_T2(uint32_t instr, uint32_t pc) {
    // LDRSB (register) § C2.4.70 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    SRType   shiftT = SRType_LSL;
    uint32_t shiftN = imm2;

    if (t == 13 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSB_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_LDRSH_register_T2 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_LDRSH_register_T2(uint32_t instr, uint32_t pc) {
    // LDRSH (register) § C2.4.74 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t m      = Rm;
    bool     index  = true, add = true, wback = false;
    SRType   shiftT = SRType_LSL;
    uint32_t shiftN = imm2;

    if (t == 13 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSH_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_1100_00xxx_1110xx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_00xxx_1110xx(uint32_t instr, uint32_t pc) {
    // Load/store, unsigned (unprivileged)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t L    = GETBITS(instr>>16, 4, 4);

    switch ((size<<1) | L) {
      case 0b00'0:
        // STRBT
        _DecodeExecute32_STRBT_T1(instr, pc);
        break;

      case 0b00'1:
        // LDRBT
        _DecodeExecute32_LDRBT_T1(instr, pc);
        break;

      case 0b01'0:
        // STRHT
        _DecodeExecute32_STRHT_T1(instr, pc);
        break;

      case 0b01'1:
        // LDRHT
        _DecodeExecute32_LDRHT_T1(instr, pc);
        break;

      case 0b10'0:
        // STRT
        _DecodeExecute32_STRT_T1(instr, pc);
        break;

      case 0b10'1:
        // LDRT
        _DecodeExecute32_LDRT_T1(instr, pc);
        break;

      default:
        abort();
        break;
    }
  }

  /* _DecodeExecute32_LDRBT_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_LDRBT_T1(uint32_t instr, uint32_t pc) {
    // LDRBT § C2.4.58 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t            = Rt;
    uint32_t n            = Rn;
    bool     postindex    = false;
    bool     add          = true;
    bool     registerForm = false;
    uint32_t imm32        = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRBT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_LDRHT_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_LDRHT_T1(uint32_t instr, uint32_t pc) {
    // LDRHT § C2.4.67 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t            = Rt;
    uint32_t n            = Rn;
    bool     postindex    = false;
    bool     add          = true;
    bool     registerForm = false;
    uint32_t imm32        = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRHT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_LDRT_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_LDRT_T1(uint32_t instr, uint32_t pc) {
    // LDRT § C2.4.76 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t            = Rt;
    uint32_t n            = Rn;
    bool     postindex    = false;
    bool     add          = true;
    bool     registerForm = false;
    uint32_t imm32        = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_STRBT_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_STRBT_T1(uint32_t instr, uint32_t pc) {
    // STRBT § C2.4.187 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t            = Rt;
    uint32_t n            = Rn;
    bool     postindex    = false;
    bool     add          = true;
    bool     registerForm = false;
    uint32_t imm32        = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRBT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_STRHT_T1 {{{4
   * -------------------------
   */
  void _DecodeExecute32_STRHT_T1(uint32_t instr, uint32_t pc) {
    // STRHT § C2.4.194 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t            = Rt;
    uint32_t n            = Rn;
    bool     postindex    = false;
    bool     add          = true;
    bool     registerForm = false;
    uint32_t imm32        = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRHT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_STRT_T1 {{{4
   * ------------------------
   */
  void _DecodeExecute32_STRT_T1(uint32_t instr, uint32_t pc) {
    // STRT § C2.4.195 T1
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t            = Rt;
    uint32_t n            = Rn;
    bool     postindex    = false;
    bool     add          = true;
    bool     registerForm = false;
    uint32_t imm32        = _ZeroExtend(imm8, 32);

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRT(t, n, postindex, add, registerForm, imm32);
  }

  /* _DecodeExecute32_1100_00xxx_11x1xx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_00xxx_11x1xx(uint32_t instr, uint32_t pc) {
    // Load/store, unsigned (immediate, pre-indexed)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t L    = GETBITS(instr>>16, 4, 4);

    switch ((size<<1) | L) {
      case 0b00'0:
        // STRB (immediate)
        _DecodeExecute32_STRB_immediate_T3(instr, pc);
        break;
      case 0b00'1:
        // LDRB (immediate)
        _DecodeExecute32_LDRB_immediate_T3(instr, pc);
        break;
      case 0b01'0:
        // STRH (immediate)
        _DecodeExecute32_STRH_immediate_T3(instr, pc);
        break;
      case 0b01'1:
        // LDRH (immediate)
        _DecodeExecute32_LDRH_immediate_T3(instr, pc);
        break;
      case 0b10'0:
        // STR (immediate)
        _DecodeExecute32_STR_immediate_T4(instr, pc);
        break;
      case 0b10'1:
        // LDR (immediate)
        _DecodeExecute32_LDR_immediate_T4(instr, pc);
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

  /* _DecodeExecute32_1100_00xxx_1100xx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_00xxx_1100xx(uint32_t instr, uint32_t pc) {
    // Load/store, unsigned (negative immediate)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t L    = GETBITS(instr>>16, 4, 4);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    switch ((size<<1) | L) {
      case 0b00'0:
        // STRB (immediate)
        _DecodeExecute32_STRB_immediate_T3(instr, pc);
        break;
      case 0b00'1:
        if (Rt != 0b1111) {
          // LDRB (immediate)
          _DecodeExecute32_LDRB_immediate_T3(instr, pc);
        } else {
          // PLD, PLDW (immediate) — Preload read variant
          _DecodeExecute32_PLD_PLDW_immediate_T2(instr, pc);
        }
        break;
      case 0b01'0:
        // STRH (immediate)
        _DecodeExecute32_STRH_immediate_T3(instr, pc);
        break;
      case 0b01'1:
        if (Rt != 0b1111) {
          // LDRH (immediate)
          _DecodeExecute32_LDRH_immediate_T3(instr, pc);
        } else {
          // PLD, PLDW (immediate) — Preload write variant
          _DecodeExecute32_PLD_PLDW_immediate_T2(instr, pc);
        }
        break;
      case 0b10'0:
        // STR (immediate)
        _DecodeExecute32_STR_immediate_T4(instr, pc);
        break;
      case 0b10'1:
        // LDR (immediate)
        _DecodeExecute32_LDR_immediate_T4(instr, pc);
        break;
      case 0b11'0:
      case 0b11'1:
        // Unallocated
        UNDEFINED_DEC();
        break;
    }
  }

  /* _DecodeExecute32_PLD_PLDW_immediate_T2 {{{4
   * --------------------------------------
   */
  void _DecodeExecute32_PLD_PLDW_immediate_T2(uint32_t instr, uint32_t pc) {
    // PLD, PLDW (immediate) § C2.4.108 T2
    // ---- DECODE --------------------------------------------------
    uint32_t W    = GETBITS(instr>>16, 5, 5);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     add    = false;
    bool     isPLDW = !!W;

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLD_PLDW_immediate(n, imm32, add, isPLDW);
  }

  /* _DecodeExecute32_1100_00xxx_10x1xx {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_1100_00xxx_10x1xx(uint32_t instr, uint32_t pc) {
    // Load/store, unsigned (immediate, post-indexed)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t L    = GETBITS(instr>>16, 4, 4);

    switch ((size<<1) | L) {
      case 0b00'0:
        // STRB (immediate)
        _DecodeExecute32_STRB_immediate_T3(instr, pc);
        break;

      case 0b00'1:
        // LDRB (immediate)
        _DecodeExecute32_LDRB_immediate_T3(instr, pc);
        break;

      case 0b01'0:
        // STRH (immediate)
        _DecodeExecute32_STRH_immediate_T3(instr, pc);
        break;

      case 0b01'1:
        // LDRH (immediate)
        _DecodeExecute32_LDRH_immediate_T3(instr, pc);
        break;

      case 0b10'0:
        // STR (immediate)
        _DecodeExecute32_STR_immediate_T4(instr, pc);
        break;

      case 0b10'1:
        // LDR (immediate)
        _DecodeExecute32_LDR_immediate_T4(instr, pc);
        break;

      case 0b11'0:
      case 0b11'1:
        // Unallocated
        UNDEFINED_DEC();
        break;
    }
  }

  /* _DecodeExecute32_LDR_immediate_T4 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_LDR_immediate_T4(uint32_t instr, uint32_t pc) {
    // LDR (immediate) § C2.4.52 T4
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);
    ASSERT(!(P && U && !W));

    if (!P && !W)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P, add = !!U, wback = !!W;

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    if (t == 15 && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDRB_immediate_T3 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_LDRB_immediate_T3(uint32_t instr, uint32_t pc) {
    // LDRB (immediate) § C2.4.55 T3
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(Rt == 0b1111 && P && !U && !W));
    ASSERT(Rn != 0b1111);
    ASSERT(!(P && U && !W));

    if (!P && !W)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P, add = !!U, wback = !!W;

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    if (t == 13)
      THROW_UNPREDICTABLE();

    if (t == 15 && W)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDRH_immediate_T3 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_LDRH_immediate_T3(uint32_t instr, uint32_t pc) {
    // LDRH (immediate) § C2.4.64 T3
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);
    ASSERT(!(Rt == 0b1111 && P && !U && !W));
    ASSERT(!(P && U && !W));

    if (!P && !W)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P, add = !!U, wback = !!W;

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    if (t == 13)
      THROW_UNPREDICTABLE();

    if (t == 15 && W)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_STR_immediate_T4 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_STR_immediate_T4(uint32_t instr, uint32_t pc) {
    // STR (immediate) § C2.4.183 T4
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(P && U && !W));
    ASSERT(!(Rn == 0b1111 || (!P && !W)));

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P, add = !!U, wback = !!W;

    if (t == 15)
      THROW_UNPREDICTABLE();

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STR_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_STRB_immediate_T3 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_STRB_immediate_T3(uint32_t instr, uint32_t pc) {
    // STRB (immediate) § C2.4.185 T3
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(P && U && !W));
    ASSERT(!(Rn == 0b1111 || (!P && !W)));

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P, add = !!U, wback = !!W;

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_STRH_immediate_T3 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_STRH_immediate_T3(uint32_t instr, uint32_t pc) {
    // STRH (immediate) § C2.4.192 T3
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t P    = GETBITS(instr    ,10,10);
    uint32_t U    = GETBITS(instr    , 9, 9);
    uint32_t W    = GETBITS(instr    , 8, 8);
    uint32_t imm8 = GETBITS(instr    , 0, 7);

    ASSERT(!(P && U && !W));
    ASSERT(!(Rn == 0b1111 || (!P && !W)));

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     index  = !!P, add = !!U, wback = !!W;

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    if (wback && n == t)
      CUNPREDICTABLE_UNDEFINED();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_1100_1xxx1_1111 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_1100_1xxx1_1111(uint32_t instr, uint32_t pc) {
    // Load, signed (literal)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    switch (size) {
      case 0b00:
        if (Rt != 0b1111) {
          // LDRSB (literal)
          _DecodeExecute32_LDRSB_literal_T1(instr, pc);
        } else {
          // PLI (immediate, literal)
          _DecodeExecute32_PLI_immediate_literal_T3(instr, pc);
        }
        break;

      case 0b01:
        if (Rt != 0b1111) {
          // LDRSH (literal)
          _DecodeExecute32_LDRSH_literal_T1(instr, pc);
        } else {
          // Reserved hint, behaves as NOP
          _DecodeExecute32_ReservedHint(instr, pc);
        }
        break;

      case 0b10:
      case 0b11:
        // Unallocated
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_PLI_immediate_literal_T2 {{{4
   * -----------------------------------------
   */
  void _DecodeExecute32_PLI_immediate_literal_T2(uint32_t instr, uint32_t pc) {
    // PLI (immediate, literal) § C2.4.109 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t imm8   = GETBITS(instr    , 0, 7);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm8, 32);
    bool     add    = false;

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLI_immediate_literal(n, imm32, add);
  }

  /* _DecodeExecute32_PLI_immediate_literal_T3 {{{4
   * -----------------------------------------
   */
  void _DecodeExecute32_PLI_immediate_literal_T3(uint32_t instr, uint32_t pc) {
    // PLI (immediate, literal) § C2.4.109 T3
    // ---- DECODE --------------------------------------------------
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = 15;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = !!U;

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLI_immediate_literal(n, imm32, add);
  }

  /* _DecodeExecute32_LDRSB_literal_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_LDRSB_literal_T1(uint32_t instr, uint32_t pc) {
    // LDRSB (literal) § C2.4.69 T1
    // ---- DECODE --------------------------------------------------
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = !!U;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSB_literal(t, imm32, add);
  }

  /* _DecodeExecute32_LDRSH_literal_T1 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_LDRSH_literal_T1(uint32_t instr, uint32_t pc) {
    // LDRSH (literal) § C2.4.73 T1
    // ---- DECODE --------------------------------------------------
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = !!U;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRSH_literal(t, imm32, add);
  }

  /* _DecodeExecute32_ReservedHint {{{4
   * -----------------------------
   */
  void _DecodeExecute32_ReservedHint(uint32_t instr, uint32_t pc) {
    // Reserved hint, behaves as NOP.
    // ---- DECODE --------------------------------------------------

    TRACEI(RSVD_HINT, UNK32, "");

    // ---- EXECUTE -------------------------------------------------
    _Exec_NOP();
  }

  /* _DecodeExecute32_1100_00xxx_xxxx_000000 {{{4
   * ---------------------------------------
   */
  void _DecodeExecute32_1100_00xxx_xxxx_000000(uint32_t instr, uint32_t pc) {
    // Load/store, unsigned (register offset)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t L    = GETBITS(instr>>16, 4, 4);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    ASSERT(Rn != 0b1111);

    switch ((size<<1)|L) {
      case 0b00'0:
        // STRB (register)
        _DecodeExecute32_STRB_register_T2(instr, pc);
        break;

      case 0b00'1:
        if (Rt != 0b1111) {
          // LDRB (register)
          _DecodeExecute32_LDRB_register_T2(instr, pc);
        } else {
          // PLD (register) — Preload read variant
          _DecodeExecute32_PLD_register_RO(instr, pc);
        }
        break;

      case 0b01'0:
        // STRH (register)
        _DecodeExecute32_STRH_register_T2(instr, pc);
        break;

      case 0b01'1:
        if (Rt != 0b1111) {
          // LDRH (register)
          _DecodeExecute32_LDRH_register_T2(instr, pc);
        } else {
          // PLD (register) — Preload write variant
          _DecodeExecute32_PLD_register_RO(instr, pc);
        }
        break;

      case 0b10'0:
        // STR (register)
        _DecodeExecute32_STR_register_T2(instr, pc);
        break;

      case 0b10'1:
        // LDR (register)
        _DecodeExecute32_LDR_register_T2(instr, pc);
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

  /* _DecodeExecute32_STR_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_STR_register_T2(uint32_t instr, uint32_t pc) {
    // STR (register) § C2.4.184 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    SRType    shiftT = SRType_LSL;
    uint32_t  shiftN = imm2;

    if (t == 15 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(STR_reg, T2, "t=%u n=%u m=%u shiftT=%u shiftN=%u", t, n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STR_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_STRB_register_T2 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_STRB_register_T2(uint32_t instr, uint32_t pc) {
    // STRB (register) § C2.4.186 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    SRType    shiftT = SRType_LSL;
    uint32_t  shiftN = imm2;

    if ((t == 13 || t == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(STRB_reg, T2, "t=%u n=%u m=%u shiftT=%u shiftN=%u", t, n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRB_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_STRH_register_T2 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_STRH_register_T2(uint32_t instr, uint32_t pc) {
    // STRH (register) § C2.4.193 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    SRType    shiftT = SRType_LSL;
    uint32_t  shiftN = imm2;

    if ((t == 13 || t == 15) || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(STRH_reg, T2, "t=%u n=%u m=%u shiftT=%u shiftN=%u", t, n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRH_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_PLD_register_RO {{{4
   * --------------------------------
   */
  void _DecodeExecute32_PLD_register_RO(uint32_t instr, uint32_t pc) {
    // PLD (register) § C2.4.107 Register-offset
    // ---- DECODE --------------------------------------------------
    uint32_t W    = GETBITS(instr>>16, 5, 5);
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  n       = Rn;
    uint32_t  m       = Rm;
    bool      add     = true;
    SRType    shiftT  = SRType_LSL;
    uint32_t  shiftN  = imm2; // XXX error in ISA manual

    if (m == 13 || m == 15)
      THROW_UNPREDICTABLE();

    TRACEI(PLD_reg, RO, "n=%u m=%u shiftT=%u shiftN=%u", n, m, shiftT, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLD_register(n, m, add, shiftT, shiftN);
  }

  /* _DecodeExecute32_LDR_register_T2 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_LDR_register_T2(uint32_t instr, uint32_t pc) {
    // LDR (register) § C2.4.54 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    SRType    shiftT = SRType_LSL;
    uint32_t  shiftN = imm2;

    if (m == 13 || m == 15)
      THROW_UNPREDICTABLE();

    if (t == 15 && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(LDR_reg, T2, "t=%u n=%u m=%u shiftL=%u", t, n, m, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_LDRB_register_T2 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_LDRB_register_T2(uint32_t instr, uint32_t pc) {
    // LDRB (register) § C2.4.57 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    SRType    shiftT = SRType_LSL;
    uint32_t  shiftN = imm2;

    if (t == 13 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(LDRB_reg, T2, "t=%u n=%u m=%u shiftL=%u", t, n, m, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRB_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_LDRH_register_T2 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_LDRH_register_T2(uint32_t instr, uint32_t pc) {
    // LDRH (register) § C2.4.66 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn   = GETBITS(instr>>16, 0, 3);
    uint32_t Rt   = GETBITS(instr    ,12,15);
    uint32_t imm2 = GETBITS(instr    , 4, 5);
    uint32_t Rm   = GETBITS(instr    , 0, 3);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t = Rt;
    uint32_t  n = Rn;
    uint32_t  m = Rm;
    bool      index = true, add = true, wback = false;
    SRType    shiftT = SRType_LSL;
    uint32_t  shiftN = imm2;

    if (t == 13 || (m == 13 || m == 15))
      THROW_UNPREDICTABLE();

    TRACEI(LDRH_reg, T2, "t=%u n=%u m=%u shiftL=%u", t, n, m, shiftN);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRH_register(t, n, m, index, add, wback, shiftT, shiftN);
  }

  /* _DecodeExecute32_1100_0xxxx_1111 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_1100_0xxxx_1111(uint32_t instr, uint32_t pc) {
    // Load, unsigned (literal)
    uint32_t U    = GETBITS(instr>>16, 7, 7);
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t L    = GETBITS(instr>>16, 4, 4);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    switch ((size<<1)|L) {
      case 0b00'1:
        if (Rt != 0b1111) {
          // LDRB (literal)
          _DecodeExecute32_LDRB_literal_T1(instr, pc);
        } else {
          // PLD (literal)
          _DecodeExecute32_PLD_literal_T1(instr, pc);
        }
        break;

      case 0b01'1:
        if (Rt != 0b1111) {
          // LDRH (literal)
          _DecodeExecute32_LDRH_literal_T1(instr, pc);
        } else {
          // ?
          UNDEFINED_DEC();
        }
        break;

      case 0b10'1:
        // LDR (literal)
        _DecodeExecute32_LDR_literal_T2(instr, pc);
        break;

      case 0b11'0:
      case 0b11'1:
        UNDEFINED_DEC();
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_LDRH_literal_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_LDRH_literal_T1(uint32_t instr, uint32_t pc) {
    // LDRH (literal) § C2.4.65 T1
    // ---- DECODE --------------------------------------------------
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = !!U;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRH_literal(t, imm32, add);
  }

  /* _DecodeExecute32_LDRB_literal_T1 {{{4
   * --------------------------------
   */
  void _DecodeExecute32_LDRB_literal_T1(uint32_t instr, uint32_t pc) {
    // LDRB (literal) § C2.4.56 T1
    // ---- DECODE --------------------------------------------------
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = !!U;

    if (t == 13)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRB_literal(t, imm32, add);
  }

  /* _DecodeExecute32_PLD_literal_T1 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_PLD_literal_T1(uint32_t instr, uint32_t pc) {
    // PLD (literal) § C2.4.106 T1
    // ---- DECODE --------------------------------------------------
    uint32_t U      = GETBITS(instr>>16, 7, 7);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = !!U;

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLD_literal(imm32, add);
  }

  /* _DecodeExecute32_LDR_literal_T2 {{{4
   * -------------------------------
   */
  void _DecodeExecute32_LDR_literal_T2(uint32_t instr, uint32_t pc) {
    // LDR (literal) § C2.4.53 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);
    uint32_t U      = GETBITS(instr>>16, 7, 7);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t      = Rt;
    uint32_t imm32  = _ZeroExtend(imm12, 32);
    bool     add    = !!U;

    if (t == 15 && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(LDR_lit, T2, "t=%u imm32=0x%x add=%u", t, imm32, add);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_literal(t, imm32, add);
  }

  /* _DecodeExecute32_1100_01xxx_xxxx {{{4
   * --------------------------------
   */
  void _DecodeExecute32_1100_01xxx_xxxx(uint32_t instr, uint32_t pc) {
    // Load/store, unsigned (positive immediate)
    uint32_t size = GETBITS(instr>>16, 5, 6);
    uint32_t L    = GETBITS(instr>>16, 4, 4);
    uint32_t Rt   = GETBITS(instr    ,12,15);

    uint32_t size_L = (size<<1) | L;
    switch (size_L) {
      case 0b00'0:
        // STRB (immediate)
        _DecodeExecute32_STRB_immediate_T2(instr, pc);
        break;

      case 0b00'1:
        if (Rt != 0b1111) {
          // LDRB (immediate)
          _DecodeExecute32_LDRB_immediate_T2(instr, pc);
        } else {
          // PLD, PLDW (immediate) — Preload read variant
          _DecodeExecute32_PLD_PLDW_immediate_T1(instr, pc);
        }
        break;

      case 0b01'0:
        // STRH (immediate)
        _DecodeExecute32_STRH_immediate_T2(instr, pc);
        break;

      case 0b01'1:
        if (Rt != 0b1111) {
          // LDRH (immediate)
          _DecodeExecute32_LDRH_immediate_T2(instr, pc);
        } else {
          // PLD, PLDW (immediate) — Preload write variant
          _DecodeExecute32_PLD_PLDW_immediate_T1(instr, pc);
        }
        break;

      case 0b10'0:
        // STR (immediate)
        _DecodeExecute32_STR_immediate_T3(instr, pc);
        break;

      case 0b10'1:
        // LDR (immediate)
        _DecodeExecute32_LDR_immediate_T3(instr, pc);
        break;

      default:
        abort();
    }
  }

  /* _DecodeExecute32_PLD_PLDW_immediate_T1 {{{4
   * --------------------------------------
   */
  void _DecodeExecute32_PLD_PLDW_immediate_T1(uint32_t instr, uint32_t pc) {
    // PLD, PLDW (immediate) § C2.4.108 T1
    // ---- DECODE --------------------------------------------------
    uint32_t W      = GETBITS(instr>>16, 5, 5);
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rn != 0b1111);

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t n      = Rn;
    uint32_t imm32  = _ZeroExtend(imm32, 32);
    bool     add    = true;
    bool     isPLDW = !!W;

    // ---- EXECUTE -------------------------------------------------
    _Exec_PLD_PLDW_immediate(n, imm32, add, isPLDW);
  }

  /* _DecodeExecute32_STRB_immediate_T2 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_STRB_immediate_T2(uint32_t instr, uint32_t pc) {
    // STRB (immediate) § C2.4.185 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    if (Rn == 0b1111)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t     = Rt;
    uint32_t n     = Rn;
    uint32_t imm32 = _ZeroExtend(imm12, 32);
    bool     index = true, add = true, wback = false;

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_STRH_immediate_T2 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_STRH_immediate_T2(uint32_t instr, uint32_t pc) {
    // STRH (immediate) § C2.4.192 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    if (Rn == 0b1111)
      THROW_UNDEFINED();

    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t t     = Rt;
    uint32_t n     = Rn;
    uint32_t imm32 = _ZeroExtend(imm12, 32);
    bool     index = true, add = true, wback = false;

    if (t == 13 || t == 15)
      THROW_UNPREDICTABLE();

    // ---- EXECUTE -------------------------------------------------
    _Exec_STRH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_STR_immediate_T3 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_STR_immediate_T3(uint32_t instr, uint32_t pc) {
    // STR (immediate) § C2.4.183 T3
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t     = Rt;
    uint32_t  n     = Rn;
    uint32_t  imm32 = _ZeroExtend(imm12, 32);
    bool      index = true, add = true, wback = false;

    if (t == 15)
      THROW_UNPREDICTABLE();

    TRACEI(STR_imm, T3, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_STR_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDRB_immediate_T2 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_LDRB_immediate_T2(uint32_t instr, uint32_t pc) {
    // LDRB (immediate) § C2.4.55 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t     = Rt;
    uint32_t  n     = Rn;
    uint32_t  imm32 = _ZeroExtend(imm12, 32);
    bool      index = true, add = true, wback = false;

    if (t == 13)
      THROW_UNPREDICTABLE();

    TRACEI(LDRB_imm, T2, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRB_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDRH_immediate_T2 {{{4
   * ----------------------------------
   */
  void _DecodeExecute32_LDRH_immediate_T2(uint32_t instr, uint32_t pc) {
    // LDRH (immediate) § C2.4.64 T2
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rt != 0b1111);
    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t     = Rt;
    uint32_t  n     = Rn;
    uint32_t  imm32 = _ZeroExtend(imm12, 32);
    bool      index = true, add = true, wback = false;

    if (t == 13)
      THROW_UNPREDICTABLE();

    TRACEI(LDRH_imm, T2, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDRH_immediate(t, n, imm32, index, add, wback);
  }

  /* _DecodeExecute32_LDR_immediate_T3 {{{4
   * ---------------------------------
   */
  void _DecodeExecute32_LDR_immediate_T3(uint32_t instr, uint32_t pc) {
    // LDR (immediate) § C2.4.52 T3
    // ---- DECODE --------------------------------------------------
    uint32_t Rn     = GETBITS(instr>>16, 0, 3);
    uint32_t Rt     = GETBITS(instr    ,12,15);
    uint32_t imm12  = GETBITS(instr    , 0,11);

    ASSERT(Rn != 0b1111);
    if (!_HaveMainExt())
      THROW_UNDEFINED();

    uint32_t  t     = Rt;
    uint32_t  n     = Rn;
    uint32_t  imm32 = _ZeroExtend(imm12, 32);
    bool      index = true, add = true, wback = false;

    if (t == 15 && _InITBlock() && !_LastInITBlock())
      THROW_UNPREDICTABLE();

    TRACEI(LDR_imm, T3, "t=%u n=%u imm32=0x%x", t, n, imm32);

    // ---- EXECUTE -------------------------------------------------
    _Exec_LDR_immediate(t, n, imm32, index, add, wback);
  }

  /* Instruction Execution {{{3
   * =====================
   */

  /* _Exec_ADC_immediate {{{4
   * -------------------
   */
  void _Exec_ADC_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32) {
    // ADC (immediate) § C2.4.1
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), imm32, GETBITSM(_s.xpsr, XPSR__C));
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_ADC_register {{{4
   * ------------------
   */
  void _Exec_ADC_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // ADC (register) § C2.4.2
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), shifted, GETBITSM(_s.xpsr, XPSR__C));
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_ADD_SP_plus_immediate {{{4
   * ---------------------------
   */
  void _Exec_ADD_SP_plus_immediate(uint32_t d, bool setflags, uint32_t imm32) {
    // ADD (SP plus immediate) § C2.4.3
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetSP(), imm32, false);
    _SetRSPCheck(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_ADD_SP_plus_register {{{4
   * --------------------------
   */
  void _Exec_ADD_SP_plus_register(uint32_t d, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // ADD (SP plus register) § C2.4.4
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetSP(), shifted, false);
    if (d == 15)
      _ALUWritePC(result); // setflags is always false here
    else {
      _SetRSPCheck(d, result);
      if (setflags) {
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
      }
    }
  }

  /* _Exec_ADD_immediate {{{4
   * -------------------
   */
  void _Exec_ADD_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32) {
    // ADD (immediate) § C2.4.5
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), imm32, false);
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_ADD_register {{{4
   * ------------------
   */
  void _Exec_ADD_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // ADD (register) § C2.4.7
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), shifted, false);
    if (d == 15)
      _ALUWritePC(result);
    else {
      _SetR(d, result);
      if (setflags) {
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
      }
    }
  }

  /* _Exec_ADR {{{4
   * ---------
   */
  void _Exec_ADR(uint32_t d, uint32_t imm32, bool add) {
    // ADR § C2.4.8
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = add ? _Align(_GetPC(),4) + imm32 : _Align(_GetPC(),4) - imm32;
    _SetR(d, result);
  }

  /* _Exec_AND_immediate {{{4
   * -------------------
   */
  void _Exec_AND_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32, bool carry) {
    // AND (immediate) § C2.4.9
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

  /* _Exec_AND_register {{{4
   * ------------------
   */
  void _Exec_AND_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // AND (register) § C2.4.10
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = _GetR(n) & shifted;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_B {{{4
   * -------
   */
  void _Exec_B(uint32_t imm32) {
    // B § C2.4.15
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _BranchWritePC(_GetPC() + imm32);
  }

  /* _Exec_BFC {{{4
   * ---------
   */
  void _Exec_BFC(uint32_t d, uint32_t msbit, uint32_t lsbit) {
    // BFC § C2.4.16
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    if (msbit >= lsbit) {
      _SetR(d, CHGBITS(_GetR(d), lsbit, msbit, 0));
    } else
      _SetR(d, UNKNOWN_VAL(0));
  }

  /* _Exec_BFI {{{4
   * ---------
   */
  void _Exec_BFI(uint32_t d, uint32_t n, uint32_t msbit, uint32_t lsbit) {
    // BFI § C2.4.17
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    if (msbit >= lsbit) {
      _SetR(d, CHGBITS(_GetR(d), lsbit, msbit, GETBITS(_GetR(n), 0, msbit-lsbit)));
    } else
      _SetR(d, UNKNOWN_VAL(0));
  }

  /* _Exec_BIC_immediate {{{4
   * -------------------
   */
  void _Exec_BIC_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32, bool carry) {
    // BIC (immediate) § C2.4.18
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _GetR(n) & ~imm32;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_BIC_register {{{4
   * ------------------
   */
  void _Exec_BIC_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // BIC (register) § C2.4.19
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = _GetR(n) & ~shifted;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_BKPT {{{4
   * ----------
   */
  void _Exec_BKPT() {
    // BKPT § C2.4.20
    //EncodingSpecificOperations
    _BKPTInstrDebugEvent();
  }

  /* _Exec_BL {{{4
   * --------
   */
  void _Exec_BL(uint32_t imm32) {
    // BL § C2.4.21
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t nextInstrAddr = _GetPC();
    _SetLR(nextInstrAddr | 1);
    _BranchWritePC(_GetPC() + imm32);
  }

  /* _Exec_BLX {{{4
   * ---------
   */
  void _Exec_BLX(uint32_t m, bool allowNonSecure) {
    // BLX, BLXNS § C2.4.22
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t target         = _GetR(m);
    uint32_t nextInstrAddr  = (_GetPC()-2) | 1;

    if (allowNonSecure && !(target & BIT(0))) {
      if (!_IsAligned(_GetSP(), 8))
        THROW_UNPREDICTABLE();

      uint32_t addr = _GetSP() - 8;

      uint32_t savedPSR = 0;
      savedPSR = CHGBITSM(savedPSR, RETPSR__EXCEPTION, GETBITSM(_s.xpsr,      XPSR__EXCEPTION));
      savedPSR = CHGBITSM(savedPSR, RETPSR__SFPA,      GETBITSM(_s.controlS,  CONTROL__SFPA  ));

      // Only the stack locations, not the store order, are architected.
      auto spName = _LookUpSP();
      auto mode   = _CurrentMode();

      auto                      exc = _Stack(addr, 0, spName, mode, nextInstrAddr);
      if (exc.fault == NoFault) exc = _Stack(addr, 4, spName, mode, savedPSR);

      _HandleException(exc);

      // Stack pointer update will raise a fault if limit violated.
      _SetSP(addr);
      _SetLR(0xFEFF'FFFF);

      // If in handler mode, IPSR must be non-zero. To prevent revealing which
      // secure handler is calling non-secure code, IPSR is set to an invalid
      // but non-zero value (namely the reset exception number).
      if (mode == PEMode_Handler)
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__EXCEPTION, 1);
    } else
      _SetLR(nextInstrAddr);

    _BLXWritePC(target, allowNonSecure);
  }

  /* _Exec_BX {{{4
   * --------
   */
  void _Exec_BX(uint32_t m, bool allowNonSecure) {
    // BX, BXNS § C2.4.23
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto exc = _BXWritePC(_GetR(m), allowNonSecure);
    _HandleException(exc);
  }

  /* _Exec_CBNZ_CBZ {{{4
   * --------------
   */
  void _Exec_CBNZ_CBZ(uint32_t n, uint32_t imm32, bool nonzero) {
    // CBNZ, CBZ § C2.4.24
    //EncodingSpecificOperations

    if (nonzero != _IsZero(_GetR(n)))
      _BranchWritePC(_GetPC() + imm32);
  }

  /* _Exec_CDP_CDP2 {{{4
   * --------------
   */
  void _Exec_CDP_CDP2(uint32_t cp) {
    // CDP, CDP2 § C2.4.25
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr()))
      _GenerateCoprocessorException();
    else
      _Coproc_InternalOperation(cp, _ThisInstr());
  }

  /* _Exec_CLREX {{{4
   * -----------
   */
  void _Exec_CLREX() {
    // CLREX § C2.4.26
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ClearExclusiveLocal(_ProcessorID());
  }

  /* _Exec_CLZ {{{4
   * ---------
   */
  void _Exec_CLZ(uint32_t d, uint32_t m) {
    // CLZ § C2.4.27 T1
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _CountLeadingZeroBits(_GetR(m));
    _SetR(d, result);
  }

  /* _Exec_CMP_immediate {{{4
   * -------------------
   */
  void _Exec_CMP_immediate(uint32_t n, uint32_t imm32) {
    // CMP (immediate) § C2.4.30
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), ~imm32, true);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
  }

  /* _Exec_CMP_register {{{4
   * ------------------
   */
  void _Exec_CMP_register(uint32_t n, uint32_t m, SRType shiftT, uint32_t shiftN) {
    // CMP (register) § C2.4.31
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), ~shifted, true);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
  }

  /* _Exec_CMN_immediate {{{4
   * -------------------
   */
  void _Exec_CMN_immediate(uint32_t n, uint32_t imm32) {
    // CMN (immediate) § C2.4.28
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), imm32, false);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
  }

  /* _Exec_CMN_register {{{4
   * ------------------
   */
  void _Exec_CMN_register(uint32_t n, uint32_t m, SRType shiftT, uint32_t shiftN) {
    // CMN (register) § C2.4.29
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), shifted, false);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
  }

  /* _Exec_CPS {{{4
   * ---------
   */
  void _Exec_CPS(bool enable, bool disable, bool affectPRI, bool affectFAULT) {
    // CPS § C2.4.32
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

  /* _Exec_DBG {{{4
   * ---------
   */
  void _Exec_DBG(uint32_t option) {
    // DBG § C2.4.33
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _Hint_Debug(option);
  }

  /* _Exec_DMB {{{4
   * ---------
   */
  void _Exec_DMB(uint32_t option) {
    // DMB § C2.4.34
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _DataMemoryBarrier(option);
  }

  /* _Exec_DSB {{{4
   * ---------
   */
  void _Exec_DSB(uint32_t option) {
    // DSB § C2.4.35
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _DataSynchronizationBarrier(option);
  }

  /* _Exec_EOR_immediate {{{4
   * -------------------
   */
  void _Exec_EOR_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32, bool carry) {
    // EOR (immediate) § C2.4.36
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _GetR(n) ^ imm32;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_EOR_register {{{4
   * ------------------
   */
  void _Exec_EOR_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = _GetR(n) ^ shifted;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_ISB {{{4
   * ---------
   */
  void _Exec_ISB(uint32_t option) {
    // ISB § C2.4.40
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _InstructionSynchronizationBarrier(option);
  }

  /* _Exec_IT {{{4
   * --------
   */
  void _Exec_IT(uint32_t firstCond, uint32_t mask) {
    // IT § C2.4.41
    //EncodingSpecificOperations
    _SetITSTATE((firstCond<<4) | mask);
  }

  /* _Exec_LDAB {{{4
   * ----------
   */
  void _Exec_LDAB(uint32_t t, uint32_t n) {
    // LDAB § C2.4.43
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetR(t, _ZeroExtend(_MemO(addr, 1), 32));
  }

  /* _Exec_LDAH {{{4
   * ----------
   */
  void _Exec_LDAH(uint32_t t, uint32_t n) {
    // LDAH § C2.4.47
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetR(t, _ZeroExtend(_MemO(addr, 2), 32));
  }

  /* _Exec_LDA {{{4
   * ---------
   */
  void _Exec_LDA(uint32_t t, uint32_t n) {
    // LDA § C2.4.42
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetR(t, _ZeroExtend(_MemO(addr, 4), 32));
  }

  /* _Exec_LDAEXB {{{4
   * ------------
   */
  void _Exec_LDAEXB(uint32_t t, uint32_t n) {
    // LDAEXB § C2.4.45
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetExclusiveMonitors(addr, 1);
    _SetR(t, _ZeroExtend(_MemO(addr, 1), 32));
  }

  /* _Exec_LDAEXH {{{4
   * ------------
   */
  void _Exec_LDAEXH(uint32_t t, uint32_t n) {
    // LDAEXH § C2.4.46
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetExclusiveMonitors(addr, 2);
    _SetR(t, _ZeroExtend(_MemO(addr, 2), 32));
  }

  /* _Exec_LDAEX {{{4
   * -----------
   */
  void _Exec_LDAEX(uint32_t t, uint32_t n) {
    // LDAEX § C2.4.44
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetExclusiveMonitors(addr, 4);
    _SetR(t, _ZeroExtend(_MemO(addr, 4), 32));
  }

  /* _Exec_LDC_LDC2_immediate {{{4
   * ------------------------
   */
  void _Exec_LDC_LDC2_immediate(uint32_t n, uint32_t cp, uint32_t imm32, bool index, bool add, bool wback) {
    // LDC, LDC2 (immediate) § C2.4.48
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr())) {
      _GenerateCoprocessorException();
      return;
    }

    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit check should be performed.
    uint32_t limit;
    bool     applyLimit;
    if (wback && n == 13)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit) {
      do {
        _Coproc_SendLoadedWord(_MemA(addr, 4), cp, _ThisInstr());
        addr += 4;
      } while (!_Coproc_DoneLoading(cp, _ThisInstr()));
    }

    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_LDC_LDC2_literal {{{4
   * ----------------------
   */
  void _Exec_LDC_LDC2_literal(uint32_t index, bool add, uint32_t cp, uint32_t imm32) {
    // LDC, LDC2 (literal) § C2.4.49
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr())) {
      _GenerateCoprocessorException();
      return;
    }

    uint32_t offsetAddr = add ? _Align(_GetPC(), 4) + imm32 : _Align(_GetPC(), 4) - imm32;
    uint32_t addr       = index ? offsetAddr : _Align(_GetPC(), 4);

    do {
      _Coproc_SendLoadedWord(_MemA(addr, 4), cp, _ThisInstr());
      addr += 4;
    } while (!_Coproc_DoneLoading(cp, _ThisInstr()));
  }

  /* _Exec_LDM {{{4
   * ---------
   */
  void _Exec_LDM(uint32_t n, uint32_t registers, bool wback) {
    // LDM, LDMIA, LDMFD § C2.4.50
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations

    uint32_t addr = _GetR(n);
    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback) {
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
      // If the memory operation is not performed as a result of a stack limit
      // violation, and the write-back of the SP itself does not raise a stack
      // limit violation, it is IMPLEMENTATION DEFINED whether a SPLIM
      // exception is raised. It is recommended that any instruction which
      // discards a memory access as a result of a stack limit violation, and
      // where the write-back of the SP itself does not raise a stack limit
      // violation, generates a SPLIM exception.
      if (IMPL_DEF_SPLIM_EXCEPTION_ON_INVAL_MEM_ACCESS) {
        if (applyLimit && addr < limit) {
          if (_HaveMainExt())
            InternalOr32(REG_CFSR, REG_CFSR__UFSR__STKOF);
          // If Main Extension is not implemented the fault always escalates to
          // HardFault.
          auto excInfo = _CreateException(UsageFault, false, UNKNOWN_VAL(false));
          _HandleException(excInfo);
        }
      }
    } else
      applyLimit = false;

    uint32_t newBaseVal = 0; // ???
    for (int i=0; i<15; ++i) {
      // If R[n] is the SP, memory operation only performed if limit not violated.
      if (GETBIT(registers, i) && (!applyLimit || addr >= limit)) {
        if (i != n)
          _SetR(i, _MemA(addr, 4));
        else
          newBaseVal = _MemA(addr, 4);

        addr += 4;
      }
    }

    uint32_t newPCVal = 0; // ???
    if (GETBIT(registers, 15) && (!applyLimit || addr >= limit))
      newPCVal = _MemA(addr, 4);

    // If the register list contains the register that holds the base address
    // it must be updated after all memory reads have been performed. This
    // prevents the base address being overwritten if one of the memory reads
    // generates a fault.
    if (GETBIT(registers, n))
      wback = true;
    else
      newBaseVal = _GetR(n) + 4*_BitCount(registers);

    // If the PC is in the register list update that now, which may raise a
    // fault. Likewise, if R[n] is the SP, writing back may raise a fault due
    // to SP limit violation.
    if (GETBIT(registers, 15))
      _LoadWritePC(newPCVal, n, newBaseVal, wback, false);
    else if (wback)
      _SetRSPCheck(n, newBaseVal);
  }

  /* _Exec_LDMDB {{{4
   * -----------
   */
  void _Exec_LDMDB(uint32_t n, uint32_t registers, bool wback) {
    // LDMDB, LDMEA § C2.4.51
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n) - 4*_BitCount(registers);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool     applyLimit;
    bool     doOperation;
    if (n == 13 && wback && !GETBIT(registers, n)) {
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
      doOperation                 = (!applyLimit || addr >= limit);
    } else
      doOperation = true;

    uint32_t data = 0, newPCVal = 0, newBaseVal = 0;
    for (int i=0; i<16; ++i) {
      // Memory operation only performed if limit not violated
      if (GETBIT(registers, i) && doOperation) {
        data   = _MemA(addr, 4);
        addr  += 4;
        if (i == 15)
          newPCVal = data;
        else if (i == n)
          newBaseVal = data;
        else
          _SetR(i, data);
      }
    }

    // If the register list contains the register that holds the base
    // address it must be updated after all memory reads have been performed.
    // This prevents the base address being overwritten if one of the memory
    // reads generates a fault.
    if (GETBIT(registers, n))
      wback = true;
    else
      newBaseVal = _GetR(n) - 4*_BitCount(registers);

    // If the PC is in the register list, update that now, which may raise a fault.
    if (GETBIT(registers,15))
      _LoadWritePC(newPCVal, n, newBaseVal, wback, true);
    else if (wback)
      _SetRSPCheck(n, newBaseVal);
  }

  /* _Exec_LDR_immediate {{{4
   * -------------------
   */
  void _Exec_LDR_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // LDR (immediate) § C2.4.52
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

  /* _Exec_LDR_literal {{{4
   * -----------------
   */
  void _Exec_LDR_literal(uint32_t t, uint32_t imm32, bool add) {
    // LDR (literal) § C2.4.53
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
        CUNPREDICTABLE_UNALIGNED();
    } else
      _SetR(t, data);
  }

  /* _Exec_LDR_register {{{4
   * ------------------
   */
  void _Exec_LDR_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // LDR (register) § C2.4.54
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset     = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t offsetAddr = add ? _GetR(n) + offset : _GetR(n) - offset;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    uint32_t data = 0;

    // Memory operation only performed if limit not violated.
    if (!applyLimit && offsetAddr >= limit)
      data = _MemU(addr, 4);

    // If the stack pointer is being updated a fault will be raised
    // if the limit is violated.
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

  /* _Exec_LDRB_immediate {{{4
   * --------------------
   */
  void _Exec_LDRB_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // LDRB (immediate) § C2.4.55
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

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit)
      _SetR(t, _ZeroExtend(_MemU(addr, 1), 32));

    // If the stack pointer is benig updated a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_LDRSB_immediate {{{4
   * ---------------------
   */
  void _Exec_LDRSB_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // LDRSB (immediate) § C2.4.68
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

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit)
      _SetR(t, _SignExtend(_MemU(addr, 1), 8, 32));

    // If the stack pointer is benig updated a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_LDRB_literal {{{4
   * ------------------
   */
  void _Exec_LDRB_literal(uint32_t t, uint32_t imm32, bool add) {
    // LDRB (literal) § C2.4.56
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t base = _Align(_GetPC(), 4);
    uint32_t addr = add ? base + imm32 : base - imm32;
    _SetR(t, _ZeroExtend(_MemU(addr, 1), 32));
  }

  /* _Exec_LDRB_register {{{4
   * -------------------
   */
  void _Exec_LDRB_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // LDRB (register) § C2.4.57
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset     = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t offsetAddr = add ? _GetR(n) + offset : _GetR(n) - offset;
    uint32_t addr       = index ? offsetAddr : _GetR(n);
    _SetR(t, _ZeroExtend(_MemU(addr, 1), 32));
  }

  /* _Exec_LDRBT {{{4
   * -----------
   */
  void _Exec_LDRBT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // LDRBT § C2.4.58
    if (!_ConditionPassed())
      return;

    uint32_t addr = _GetR(n) + imm32;
    _SetR(t, _ZeroExtend(_MemU_unpriv(addr, 1), 32));
  }

  /* _Exec_LDRHT {{{4
   * -----------
   */
  void _Exec_LDRHT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // LDRHT § C2.4.67
    if (!_ConditionPassed())
      return;

    uint32_t addr = _GetR(n) + imm32;
    uint32_t data = _MemU_unpriv(addr, 2);
    _SetR(t, _ZeroExtend(data, 32));
  }

  /* _Exec_LDRSBT {{{4
   * ------------
   */
  void _Exec_LDRSBT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // LDRSBT § C2.4.71
    if (!_ConditionPassed())
      return;

    uint32_t addr = _GetR(n) + imm32;
    _SetR(t, _SignExtend(_MemU_unpriv(addr, 1), 8, 32));
  }

  /* _Exec_LDRSHT {{{4
   * ------------
   */
  void _Exec_LDRSHT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // LDRSHT § C2.4.75
    if (!_ConditionPassed())
      return;

    uint32_t addr = _GetR(n) + imm32;
    uint32_t data = _MemU_unpriv(addr, 2);
    _SetR(t, _SignExtend(data, 16, 32));
  }

  /* _Exec_LDRT {{{4
   * -----------
   */
  void _Exec_LDRT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // LDRT § C2.4.76
    if (!_ConditionPassed())
      return;

    uint32_t addr = _GetR(n) + imm32;
    uint32_t data = _MemU_unpriv(addr, 4);
    _SetR(t, data);
  }

  /* _Exec_LDRH_literal {{{4
   * ------------------
   */
  void _Exec_LDRH_literal(uint32_t t, uint32_t imm32, bool add) {
    // LDRH (literal) § C2.4.65
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t base = _Align(_GetPC(), 4);
    uint32_t addr = add ? base + imm32 : base - imm32;
    uint32_t data = _MemU(addr, 2);
    _SetR(t, _ZeroExtend(data, 32));
  }

  /* _Exec_LDRD_immediate {{{4
   * --------------------
   */
  void _Exec_LDRD_immediate(uint32_t t, uint32_t t2, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // LDRD (immediate) § C2.4.59
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

  /* _Exec_LDRD_literal {{{4
   * ------------------
   */
  void _Exec_LDRD_literal(uint32_t t, uint32_t t2, uint32_t imm32, bool add) {
    // LDRD (literal) § C2.4.60
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    if (GETBITS(_GetPC(), 0, 1) != 0b00)
      CUNPREDICTABLE_UNALIGNED();

    uint32_t addr = add ? _GetPC() + imm32 : _GetPC() - imm32;
    _SetR(t,  _MemA(addr  , 4));
    _SetR(t2, _MemA(addr+4, 4));
  }

  /* _Exec_LDREX {{{4
   * -----------
   */
  void _Exec_LDREX(uint32_t t, uint32_t n, uint32_t imm32) {
    // LDREX § C2.4.61
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n) + imm32;
    _SetExclusiveMonitors(addr, 4);
    _SetR(t, _MemA(addr, 4));
  }

  /* _Exec_LDREXB {{{4
   * ------------
   */
  void _Exec_LDREXB(uint32_t t, uint32_t n) {
    // LDREXB § C2.4.62
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetExclusiveMonitors(addr, 1);
    _SetR(t, _ZeroExtend(_MemA(addr, 1), 32));
  }

  /* _Exec_LDREXH {{{4
   * ------------
   */
  void _Exec_LDREXH(uint32_t t, uint32_t n) {
    // LDREXH § C2.4.63
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetExclusiveMonitors(addr, 2);
    _SetR(t, _ZeroExtend(_MemA(addr, 2), 32));
  }

  /* _Exec_LDRH_immediate {{{4
   * --------------------
   */
  void _Exec_LDRH_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // LDRH (immediate) § C2.4.64
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

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit)
      _SetR(t, _ZeroExtend(_MemU(addr, 2), 32));

    // If the stack pointer is benig updated a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_LDRSH_immediate {{{4
   * ---------------------
   */
  void _Exec_LDRSH_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // LDRSH (immediate) § C2.4.74
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

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit)
      _SetR(t, _SignExtend(_MemU(addr, 2), 16, 32));

    // If the stack pointer is benig updated a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_LDRH_register {{{4
   * -------------------
   */
  void _Exec_LDRH_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // LDRH (register) § C2.4.66
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset     = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t offsetAddr = add ? _GetR(n) + offset : _GetR(n) - offset;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    uint32_t data = _MemU(addr, 2);
    if (wback)
      _SetR(n, offsetAddr);

    _SetR(t, _ZeroExtend(data, 32));
  }

  /* _Exec_LDRSB_literal {{{4
   * -------------------
   */
  void _Exec_LDRSB_literal(uint32_t t, uint32_t imm32, bool add) {
    // LDRSB (literal) § C2.4.69
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t base = _Align(_GetPC(), 4);
    uint32_t addr = add ? base + imm32 : base - imm32;
    uint32_t data = _MemU(addr, 1);
    _SetR(t, _SignExtend(data, 8, 32));
  }

  /* _Exec_LDRSB_register {{{4
   * --------------------
   */
  void _Exec_LDRSB_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // LDRSB (register) § C2.4.70
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset     = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t offsetAddr = add ? _GetR(n) + offset : _GetR(n) - offset;
    uint32_t addr       = index ? offsetAddr : _GetR(n);
    _SetR(t, _SignExtend(_MemU(addr, 1), 8, 32));
  }

  /* _Exec_LDRSH_literal {{{4
   * -------------------
   */
  void _Exec_LDRSH_literal(uint32_t t, uint32_t imm32, bool add) {
    // LDRSH (literal) § C2.4.73
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t base = _Align(_GetPC(), 4);
    uint32_t addr = add ? base + imm32 : base - imm32;
    uint32_t data = _MemU(addr, 2);
    _SetR(t, _SignExtend(data, 16, 32));
  }

  /* _Exec_LDRSH_register {{{4
   * --------------------
   */
  void _Exec_LDRSH_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // LDRSH (register) § C2.4.74
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset     = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t offsetAddr = add ? _GetR(n) + offset : _GetR(n) - offset;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    uint32_t data = _MemU(addr, 2);
    if (wback)
      _SetR(n, offsetAddr);

    _SetR(t, _SignExtend(data, 16, 32));
  }

  /* _Exec_MCR_MCR2 {{{4
   * --------------
   */
  void _Exec_MCR_MCR2(uint32_t t, uint32_t cp) {
    // MCR, MCR2 § C2.4.85
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr()))
      _GenerateCoprocessorException();
    else
      _Coproc_SendOneWord(_GetR(t), cp, _ThisInstr());
  }

  /* _Exec_MCRR_MCRR2 {{{4
   * ----------------
   */
  void _Exec_MCRR_MCRR2(uint32_t t, uint32_t t2, uint32_t cp) {
    // MCRR, MCRR2 § C2.4.86
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr()))
      _GenerateCoprocessorException();
    else
      _Coproc_SendTwoWords(_GetR(t2), _GetR(t), cp, _ThisInstr());
  }

  /* _Exec_MRRC_MRRC2 {{{4
   * ----------------
   */
  void _Exec_MRRC_MRRC2(uint32_t t, uint32_t t2, uint32_t cp) {
    // MRRC, MRRC2 § C2.4.94
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr()))
      _GenerateCoprocessorException();
    else {
      auto [a, b] = _Coproc_GetTwoWords(cp, _ThisInstr());
      _SetR(t2, a);
      _SetR(t,  b);
    }
  }

  /* _Exec_MRC_MRC2 {{{4
   * --------------
   */
  void _Exec_MRC_MRC2(uint32_t t, uint32_t cp) {
    // MRC, MRC2 § C2.4.93
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr()))
      _GenerateCoprocessorException();
    else {
      uint32_t value = _Coproc_GetOneWord(cp, _ThisInstr());
      if (t != 15)
        _SetR(t, value);
      else {
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(value, 31));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, GETBIT(value, 30));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, GETBIT(value, 29));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, GETBIT(value, 28));
        // Bits 0 through 27 inclusive are not used.
      }
    }
  }

  /* _Exec_MOV_immediate {{{4
   * -------------------
   */
  void _Exec_MOV_immediate(uint32_t d, bool setflags, uint32_t imm32, bool carry) {
    // MOV (immediate) § C2.4.89
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = imm32;
    _SetR(d, result);

    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_MOV_register {{{4
   * ------------------
   */
  void _Exec_MOV_register(uint32_t d, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // MOV (register) § C2.4.90
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    if (d == 15)
      _ALUWritePC(result); // setflags is always false here
    else {
      _SetRSPCheck(d, result);
      if (setflags) {
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
        // APSR.V unchanged
      }
    }
  }

  /* _Exec_MOV_MOVS_register_shifted_register {{{4
   * ----------------------------------------
   */
  void _Exec_MOV_MOVS_register_shifted_register(uint32_t d, uint32_t m, uint32_t s, bool setflags, SRType shiftT) {
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shiftN = GETBITS(_GetR(s), 0, 7);
    auto [result, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_MOVT {{{4
   * ----------
   */
  void _Exec_MOVT(uint32_t d, uint32_t imm16) {
    // MOVT § C2.4.92
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _SetR(d, CHGBITS(_GetR(d),16,31, imm16));
    // R[d]<15:0> unchanged
  }

  /* _Exec_MRS {{{4
   * ---------
   */
  void _Exec_MRS(uint32_t d, uint32_t SYSm) {
    // MRS § C2.4.95
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _SetR(d, 0);

    auto &control   = _IsSecure() ? _s.controlS   : _s.controlNS;
    auto &primask   = _IsSecure() ? _s.primaskS   : _s.primaskNS;
    auto &faultmask = _IsSecure() ? _s.faultmaskS : _s.faultmaskNS;
    auto &basepri   = _IsSecure() ? _s.basepriS   : _s.basepriNS;

    // NOTE: the MSB of SYSm is used to select between either the current
    // domain's view of the registers and other domains view of the register.
    // This is required so that the Secure state can access the Non-secure
    // versions of banked registers. For security reasons the Secure versions
    // of the registers are not accessible from the Non-secure state.
    switch (GETBITS(SYSm, 3, 7)) {
      case 0b00000: // XPSR accesses
        if (SYSm == 4)
          CUNPREDICTABLE_UNDEFINED();

        if (_CurrentModeIsPrivileged() && GETBIT(SYSm, 0))
          _SetR(d, CHGBITS(_GetR(d), 0, 8, GETBITSM(_s.xpsr, XPSR__EXCEPTION)));
        if (GETBIT(SYSm, 1)) {
          _SetR(d, CHGBITS(_GetR(d),24,26, 0b000));    // EPSR reads as zero
          _SetR(d, CHGBITS(_GetR(d),10,15, 0b000000));
        }
        if (!GETBIT(SYSm, 2)) {
          _SetR(d, CHGBITS(_GetR(d),27,31, GETBITS(_s.xpsr,27,31)));
          if (_HaveDSPExt())
            _SetR(d, CHGBITS(_GetR(d),16,19, GETBITS(_s.xpsr,16,19)));
        }
        break;

      case 0b00001: // SP accesses
        if (_CurrentModeIsPrivileged()) {
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000:
              _SetR(d, _GetSP_Main());
              break;
            case 0b001:
              _SetR(d, _GetSP_Process());
              break;
            case 0b010:
              if (_IsSecure())
                _SetR(d, GETBITSM(_s.msplimS, MSPLIM__LIMIT)<<3);
              else if (_HaveMainExt())
                _SetR(d, GETBITSM(_s.msplimNS, MSPLIM__LIMIT)<<3);
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b011:
              if (_IsSecure())
                _SetR(d, GETBITSM(_s.psplimS, PSPLIM__LIMIT)<<3);
              else if (_HaveMainExt())
                _SetR(d, GETBITSM(_s.psplimNS, PSPLIM__LIMIT)<<3);
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }
        }
        break;

      case 0b10001: // SP access — alt domain
        if (!_HaveSecurityExt())
          CUNPREDICTABLE_UNDEFINED();

        if (_CurrentModeIsPrivileged() && _s.curState == SecurityState_Secure)
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000:
              _SetR(d, _GetSP_Main_NonSecure());
              break;
            case 0b001:
              _SetR(d, _GetSP_Process_NonSecure());
              break;
            case 0b010:
              if (_HaveMainExt())
                _SetR(d, GETBITSM(_s.msplimNS, MSPLIM__LIMIT)<<3);
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b011:
              if (_HaveMainExt())
                _SetR(d, GETBITSM(_s.psplimNS, PSPLIM__LIMIT)<<3);
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }

        break;

      case 0b00010: // Priority mask or CONTROL access
        switch (GETBITS(SYSm, 0, 2)) {
          case 0b000:
            if (_CurrentModeIsPrivileged())
              _SetR(d, CHGBITS(_GetR(d), 0, 0, GETBITSM(primask, PRIMASK__PM)));
            break;
          case 0b001:
            if (_HaveMainExt()) {
              if (_CurrentModeIsPrivileged())
                _SetR(d, CHGBITS(_GetR(d), 0, 7, GETBITS(basepri, 0, 7)));
            } else
              CUNPREDICTABLE_UNDEFINED();
            break;
          case 0b010:
            if (_HaveMainExt()) {
              if (_CurrentModeIsPrivileged())
                _SetR(d, CHGBITS(_GetR(d), 0, 7, GETBITS(basepri, 0, 7)));
            } else
              CUNPREDICTABLE_UNDEFINED();
            break;
          case 0b011:
            if (_HaveMainExt()) {
              if (_CurrentModeIsPrivileged())
                _SetR(d, CHGBITS(_GetR(d), 0, 0, GETBITSM(faultmask, FAULTMASK__FM)));
            } else
              CUNPREDICTABLE_UNDEFINED();
            break;
          case 0b100:
            if (_HaveFPExt() && _IsSecure())
              _SetR(d, CHGBITS(_GetR(d), 0, 3, GETBITS(control, 0, 3)));
            else if (_HaveFPExt())
              _SetR(d, CHGBITS(_GetR(d), 0, 2, GETBITS(control, 0, 2)));
            else
              _SetR(d, CHGBITS(_GetR(d), 0, 1, GETBITS(control, 0, 1)));
            break;
          default:
            CUNPREDICTABLE_UNDEFINED();
            break;
        }
        break;

      case 0b10010: // Priority mask or CONTROL access — alt domain
        if (!_HaveSecurityExt())
          CUNPREDICTABLE_UNDEFINED();
        if (_s.curState == SecurityState_Secure)
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000:
              if (_CurrentModeIsPrivileged())
                _SetR(d, CHGBITS(_GetR(d), 0, 0, GETBITSM(_s.primaskNS, PRIMASK__PM)));
              break;
            case 0b001:
              if (_HaveMainExt()) {
                if (_CurrentModeIsPrivileged())
                  _SetR(d, CHGBITS(_GetR(d), 0, 7, GETBITS(_s.basepriNS, 0, 7)));
              } else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b011:
              if (_HaveMainExt()) {
                if (_CurrentModeIsPrivileged())
                  _SetR(d, CHGBITS(_GetR(d), 0, 0, GETBITSM(_s.faultmaskNS, FAULTMASK__FM)));
              } else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b100:
              if (_HaveFPExt())
                _SetR(d, CHGBITS(_GetR(d), 0, 2, GETBITS(_s.controlNS, 0, 2)));
              else
                _SetR(d, CHGBITS(_GetR(d), 0, 1, GETBITS(_s.controlNS, 0, 1)));
              break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }
        break;

      case 0b10011: // SP_NS — Non-secure stack pointer
        if (!_HaveSecurityExt())
          CUNPREDICTABLE_UNDEFINED();

        if (_s.curState == SecurityState_Secure)
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000:
              _SetR(d, _GetSP(_LookUpSP_with_security_mode(false, _CurrentMode())));
              break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }

        break;

      default:
        CUNPREDICTABLE_UNDEFINED();
        break;
    }
  }

  /* _Exec_MSR_register {{{4
   * ------------------
   */
  void _Exec_MSR_register(uint32_t n, uint32_t mask, uint32_t SYSm) {
    // MSR (register) § C2.4.96
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations

    auto &control   = _IsSecure() ? _s.controlS   : _s.controlNS;
    auto &primask   = _IsSecure() ? _s.primaskS   : _s.primaskNS;
    auto &faultmask = _IsSecure() ? _s.faultmaskS : _s.faultmaskNS;
    auto &basepri   = _IsSecure() ? _s.basepriS   : _s.basepriNS;

    // NOTE: the MSB of SYSm is used to select between either the current
    // domain's view of the registers and other domains view of the register.
    // This is required so that the Secure state can access the Non-secure
    // versions of banked registers. For security reasons the Secure versions
    // of the registers are not accessible from the Non-secure state.
    switch (GETBITS(SYSm, 3, 7)) {
      case 0b00000: // XPSR accesses
        if (SYSm == 4)
          CUNPREDICTABLE_UNDEFINED();

        if (!GETBIT(SYSm, 2)) { // Include APSR
          if (GETBIT(mask, 0)) { // GE bits
            if (!_HaveDSPExt())
              CUNPREDICTABLE_UNDEFINED();
            else
              _s.xpsr = CHGBITS(_s.xpsr,16,19,GETBITS(_GetR(n),16,19));
          }
          if (GETBIT(mask, 1)) // N, Z, C, V, Q bits
            _s.xpsr = CHGBITS(_s.xpsr,27,31,GETBITS(_GetR(n),27,31));
        }
        break;
      case 0b00001: // SP access
        if (_CurrentModeIsPrivileged()) {
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000:
              // MSR not subject to SP limit, write directly to register
              if (_IsSecure())
                _s.r[RNameSP_Main_Secure] = _GetR(n) & ~BITS(0,1);
              else
                _s.r[RNameSP_Main_NonSecure] = _GetR(n) & ~BITS(0,1);
              break;
            case 0b001:
              // MSR not subject to SP limit, write directly to register
              if (_IsSecure())
                _s.r[RNameSP_Process_Secure] = _GetR(n) & ~BITS(0,1);
              else
                _s.r[RNameSP_Process_NonSecure] = _GetR(n) & ~BITS(0,1);
              break;
            case 0b010:
              if (_IsSecure())
                _s.msplimS = CHGBITSM(_s.msplimS, MSPLIM__LIMIT, GETBITS(_GetR(n), 3,31));
              else if (_HaveMainExt())
                _s.msplimNS = CHGBITSM(_s.msplimNS, MSPLIM__LIMIT, GETBITS(_GetR(n), 3,31));
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b011:
              if (_IsSecure())
                _s.psplimS = CHGBITSM(_s.psplimS, PSPLIM__LIMIT, GETBITS(_GetR(n), 3,31));
              else if (_HaveMainExt())
                _s.psplimNS = CHGBITSM(_s.psplimNS, PSPLIM__LIMIT, GETBITS(_GetR(n), 3,31));
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }
        }
        break;
      case 0b10001: // SP access — alt domain
        if (!_HaveSecurityExt())
          CUNPREDICTABLE_UNDEFINED();
        if (_CurrentModeIsPrivileged() && _s.curState == SecurityState_Secure)
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000:
              // MSR not subject to SP limit, write directly to register.
              _s.r[RNameSP_Main_NonSecure] = _GetR(n) & ~BITS(0,1);
              break;
            case 0b001:
              // MSR not subject to SP limit, write directly to register.
              _s.r[RNameSP_Process_NonSecure] = _GetR(n) & ~BITS(0,1);
              break;
            case 0b010:
              if (_HaveMainExt())
                _s.msplimNS = CHGBITSM(_s.msplimNS, MSPLIM__LIMIT, GETBITS(_GetR(n), 3,31));
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b011:
              if (_HaveMainExt())
                _s.psplimNS = CHGBITSM(_s.psplimNS, PSPLIM__LIMIT, GETBITS(_GetR(n), 3,31));
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }
        break;
      case 0b00010: // Priority mask or CONTROL access
        switch (GETBITS(SYSm, 0, 2)) {
          case 0b000:
            if (_CurrentModeIsPrivileged())
              primask = CHGBITSM(primask, PRIMASK__PM, GETBIT(_GetR(n), 0));
            break;
          case 0b001:
            if (_CurrentModeIsPrivileged()) {
              if (_HaveMainExt())
                basepri = CHGBITS(basepri, 0, 7, GETBITS(_GetR(n), 0, 7));
              else
                CUNPREDICTABLE_UNDEFINED();
            }
            break;
          case 0b010:
            if (_CurrentModeIsPrivileged()) {
              if (_HaveMainExt()) {
                if (   GETBITS(_GetR(n), 0, 7) != 0
                    && (   GETBITS(_GetR(n), 0, 7) < GETBITS(basepri, 0, 7)
                        || GETBITS(basepri, 0, 7) == 0)) {
                  basepri = CHGBITS(basepri, 0, 7, GETBITS(_GetR(n), 0, 7));
                }
              } else
                CUNPREDICTABLE_UNDEFINED();
            }
            break;
          case 0b011:
            if (_CurrentModeIsPrivileged()) {
              if (_HaveMainExt()) {
                if (_ExecutionPriority() > -1 || !GETBIT(_GetR(n), 0))
                  faultmask = CHGBITSM(faultmask, FAULTMASK__FM, GETBIT(_GetR(n), 0));
              } else
                CUNPREDICTABLE_UNDEFINED();
            }
            break;
          case 0b100:
            if (_CurrentModeIsPrivileged()) {
              control = CHGBITSM(control, CONTROL__NPRIV, GETBIT(_GetR(n), 0));
              control = CHGBITSM(control, CONTROL__SPSEL, GETBIT(_GetR(n), 1));
              if (_HaveFPExt() && (_IsSecure() || GETBITSM(InternalLoad32(REG_NSACR), REG_NSACR__CP10)))
                control = CHGBITSM(control, CONTROL__FPCA, GETBIT(_GetR(n), 2));
            }
            if (_HaveFPExt() && _IsSecure())
              _s.controlS = CHGBITSM(_s.controlS, CONTROL__SFPA, GETBIT(_GetR(n), 3));
            break;
          default:
            CUNPREDICTABLE_UNDEFINED();
            break;
        }
        break;
      case 0b10010: // Priority mask or CONTROL access — alt domain
        if (!_HaveSecurityExt())
          CUNPREDICTABLE_UNDEFINED();
        if (_CurrentModeIsPrivileged() && _s.curState == SecurityState_Secure)
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000:
              _s.primaskNS = CHGBITSM(_s.primaskNS, PRIMASK__PM, GETBIT(_GetR(n), 0));
              break;
            case 0b001:
              if (_HaveMainExt())
                _s.basepriNS = CHGBITS(_s.basepriNS, 0, 7, GETBITS(_GetR(n), 0, 7));
              else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b011:
              if (_HaveMainExt()) {
                if (_ExecutionPriority() > -1 || !GETBIT(_GetR(n), 0))
                  _s.faultmaskNS = CHGBITSM(_s.faultmaskNS, FAULTMASK__FM, GETBIT(_GetR(n), 0));
              } else
                CUNPREDICTABLE_UNDEFINED();
              break;
            case 0b100:
              _s.controlNS = CHGBITSM(_s.controlNS, CONTROL__NPRIV, GETBIT(_GetR(n), 0));
              _s.controlNS = CHGBITSM(_s.controlNS, CONTROL__SPSEL, GETBIT(_GetR(n), 1));
              if (_HaveFPExt())
                _s.controlNS = CHGBITSM(_s.controlNS, CONTROL__FPCA, GETBIT(_GetR(n), 2));
              break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }
        break;
      case 0b10011: // SP_NS - Non-secure stack pointer
        if (!_HaveSecurityExt())
          CUNPREDICTABLE_UNDEFINED();
        if (_s.curState == SecurityState_Secure) {
          switch (GETBITS(SYSm, 0, 2)) {
            case 0b000: {
              auto spName = _LookUpSP_with_security_mode(false, _CurrentMode());
              // MSR SP_NS is subject to SP limit check.
              (void)_SetSP(spName, false, _GetR(n));
            } break;
            default:
              CUNPREDICTABLE_UNDEFINED();
              break;
          }
        }
        break;
      default:
        CUNPREDICTABLE_UNDEFINED();
        break;
    }
  }

  /* _Exec_MLA {{{4
   * ---------
   */
  void _Exec_MLA(uint32_t d, uint32_t n, uint32_t m, uint32_t a, bool setflags) {
    // MLA § C2.4.87
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t operand1 = (uint32_t)_GetR(n); // signed/unsigned produces the same final results
    uint32_t operand2 = (uint32_t)_GetR(m); //
    uint32_t addend   = (uint32_t)_GetR(a); //
    uint32_t result   = operand1 * operand2 + addend;
    _SetR(d, GETBITS(result, 0,31));
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(GETBITS(result, 0,31)));
      // APSR.C unchanged
      // APSR.V unchanged
    }
  }

  /* _Exec_MLS {{{4
   * ---------
   */
  void _Exec_MLS(uint32_t d, uint32_t n, uint32_t m, uint32_t a) {
    // MLS § C2.4.88
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t operand1 = (uint32_t)_GetR(n); // signed/unsigned produces the same final results
    uint32_t operand2 = (uint32_t)_GetR(m); //
    uint32_t addend   = (uint32_t)_GetR(a); //
    uint32_t result   = addend - operand1 * operand2;
    _SetR(d, GETBITS(result, 0,31));
  }

  /* _Exec_MUL {{{4
   * ---------
   */
  void _Exec_MUL(uint32_t d, uint32_t n, uint32_t m, bool setflags) {
    // MUL § C2.4.97
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t operand1 = (uint32_t)_GetR(n); // signed/unsigned produces the same final results
    uint32_t operand2 = (uint32_t)_GetR(m); //
    uint32_t result   = operand1 * operand2;
    _SetR(d, GETBITS(result, 0,31));
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(GETBITS(result, 0,31)));
      // APSR.C unchanged
      // APSR.V unchanged
    }
  }

  /* _Exec_MVN_immediate {{{4
   * -------------------
   */
  void _Exec_MVN_immediate(uint32_t d, bool setflags, uint32_t imm32, bool carry) {
    // MVN (immediate) § C2.4.98
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = ~imm32;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_MVN_register {{{4
   * ------------------
   */
  void _Exec_MVN_register(uint32_t d, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = ~shifted;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_NOP {{{4
   * ---------
   */
  void _Exec_NOP() {
    // NOP § C2.4.100
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations

    // Do nothing
  }

  /* _Exec_ORR_immediate {{{4
   * -------------------
   */
  void _Exec_ORR_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32, bool carry) {
    // ORR (immediate) § C2.4.103
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _GetR(n) | imm32;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_ORN_immediate {{{4
   * -------------------
   */
  void _Exec_ORN_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32, bool carry) {
    // ORN (immediate) § C2.4.101
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _GetR(n) | ~imm32;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_ORN_register {{{4
   * ------------------
   */
  void _Exec_ORN_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // ORN (register) § C2.4.102
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = _GetR(n) | ~shifted;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_ORR_register {{{4
   * ------------------
   */
  void _Exec_ORR_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // ORR (register) § C2.4.104
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = _GetR(n) | shifted;
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      // APSR.V unchanged
    }
  }

  /* _Exec_PLD_literal {{{4
   * -----------------
   */
  void _Exec_PLD_literal(uint32_t imm32, bool add) {
    // PLD (literal) § C2.4.106
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = add ? _Align(_GetPC(), 4) + imm32 : _Align(_GetPC(), 4) - imm32;
    _Hint_PreloadData(addr);
  }

  /* _Exec_PLD_register {{{4
   * ------------------
   */
  void _Exec_PLD_register(uint32_t n, uint32_t m, bool add, SRType shiftT, uint32_t shiftN) {
    // PLD (register) § C2.4.107
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t addr   = add ? _GetR(n) + offset : _GetR(n) - offset;
    _Hint_PreloadData(addr);
  }

  /* _Exec_PLD_PLDW_immediate {{{4
   * ------------------------
   */
  void _Exec_PLD_PLDW_immediate(uint32_t n, uint32_t imm32, bool add, bool isPLDW) {
    // PLD, PLDW (immediate) § C2.4.108
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    if (isPLDW)
      _Hint_PreloadDataForWrite(addr);
    else
      _Hint_PreloadData(addr);
  }

  /* _Exec_PLI_immediate_literal {{{4
   * ---------------------------
   */
  void _Exec_PLI_immediate_literal(uint32_t n, uint32_t imm32, bool add) {
    // PLI (immediate, literal) § C2.4.109
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t base = (n == 15) ? _Align(_GetPC(), 4) : _GetR(n);
    uint32_t addr = add ? base + imm32 : base - imm32;
    _Hint_PreloadInstr(addr);
  }

  /* _Exec_PLI_register {{{4
   * ------------------
   */
  void _Exec_PLI_register(uint32_t n, uint32_t m, bool add, SRType shiftT, uint32_t shiftN) {
    // PLI (register) § C2.4.110
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t addr   = add ? _GetR(n) + offset : _GetR(n) - offset;
    _Hint_PreloadInstr(addr);
  }
  /* _Exec_RBIT {{{4
   * ----------
   */
  void _Exec_RBIT(uint32_t d, uint32_t m) {
    // RBIT § C2.4.125
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = 0;
    for (int i=0; i<32; ++i)
      result |= (GETBIT(_GetR(m), i)<<(31-i));

    _SetR(d, result);
  }

  /* _Exec_REV {{{4
   * ---------
   */
  void _Exec_REV(uint32_t d, uint32_t m) {
    // REV § C2.4.126
    if (!_ConditionPassed())
      return;

    uint32_t result = 0;
    result = CHGBITS(result,24,31,GETBITS(_GetR(m), 0, 7));
    result = CHGBITS(result,16,23,GETBITS(_GetR(m), 8,15));
    result = CHGBITS(result, 8,15,GETBITS(_GetR(m),16,23));
    result = CHGBITS(result, 0, 7,GETBITS(_GetR(m),24,31));
    _SetR(d, result);
  }

  /* _Exec_REV16 {{{4
   * -----------
   */
  void _Exec_REV16(uint32_t d, uint32_t m) {
    // REV16 § C2.4.127
    if (!_ConditionPassed())
      return;

    uint32_t result = 0;
    result = CHGBITS(result,24,31,GETBITS(_GetR(m),16,23));
    result = CHGBITS(result,16,23,GETBITS(_GetR(m),24,31));
    result = CHGBITS(result, 8,15,GETBITS(_GetR(m), 0, 7));
    result = CHGBITS(result, 0, 7,GETBITS(_GetR(m), 8,15));
    _SetR(d, result);
  }

  /* _Exec_REVSH {{{4
   * -----------
   */
  void _Exec_REVSH(uint32_t d, uint32_t m) {
    // REVSH § C2.4.128
    if (!_ConditionPassed())
      return;

    uint32_t result = 0;
    result = CHGBITS(result, 8,31,_SignExtend(GETBITS(_GetR(m), 0, 7), 8, 24));
    result = CHGBITS(result, 0, 7,GETBITS(_GetR(m), 8,15));
    _SetR(d, result);
  }

  /* _Exec_RSB_immediate {{{4
   * -------------------
   */
  void _Exec_RSB_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32) {
    // RSB (immediate) § C2.4.135
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(~_GetR(n), imm32, true);
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_RSB_register {{{4
   * ------------------
   */
  void _Exec_RSB_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // RSB (register) § C2.4.136
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(~_GetR(n), shifted, true);
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_SBC_immediate {{{4
   * -------------------
   */
  void _Exec_SBC_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32) {
    // SBC (immediate) § C2.4.140
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), ~imm32, GETBITSM(_s.xpsr, XPSR__C));
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_SBC_register {{{4
   * ------------------
   */
  void _Exec_SBC_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // SBC (register) § C2.4.141
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), ~shifted, GETBITSM(_s.xpsr, XPSR__C));
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_SBFX {{{4
   * ----------
   */
  void _Exec_SBFX(uint32_t d, uint32_t n, uint32_t lsbit, uint32_t widthminus1, uint32_t msbit) {
    // SBFX § C2.4.142
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    if (msbit <= 31)
      _SetR(d, _SignExtend(GETBITS(_GetR(n), lsbit, msbit), msbit-lsbit+1, 32));
    else
      _SetR(d, UNKNOWN_VAL(0));
  }

  /* _Exec_SDIV {{{4
   * ----------
   */
  void _Exec_SDIV(uint32_t d, uint32_t n, uint32_t m) {
    // SDIV § C2.4.143
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    int32_t result;
    if (!_GetR(m)) {
      if (_IntegerZeroDivideTrappingEnabled())
        _GenerateIntegerZeroDivide();
      else
        result = 0;
    } else
      result = int32_t(_GetR(n)) / int32_t(_GetR(m));   // RoundTowardsZero(Real(SInt(R[n])) / Real(UInt(R[m])))

    _SetR(d, (uint32_t)result);
  }

  /* _Exec_SEV {{{4
   * ---------
   */
  void _Exec_SEV() {
    // SEV § C2.4.145
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _SendEvent();
  }

  /* _Exec_SG {{{4
   * --------
   */
  void _Exec_SG() {
    // SG § C2.4.146
    //EncodingSpecificOperations

    if (!_HaveSecurityExt())
      return;

    auto sAttributes = _SecurityCheck(_ThisInstrAddr(), true, _IsSecure());
    if (!sAttributes.ns) {
      if (!_IsSecure()) {
        _SetLR(_GetLR() & ~1);
        if (_HaveFPExt())
          _s.controlS = CHGBITSM(_s.controlS, CONTROL__SFPA, 0);

        _s.curState = SecurityState_Secure;

        // IT/ICI bits cleared to prevent Non-secure code interfering with
        // Secure execution.
        if (_HaveMainExt())
          _SetITSTATE(0);
      }
    }
  }

  /* _Exec_SMLAL {{{4
   * -----------
   */
  void _Exec_SMLAL(uint32_t dLo, uint32_t dHi, uint32_t n, uint32_t m, bool setflags) {
    // SMLAL § C2.4.155
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    int64_t result = int64_t(_GetR(n)) * int64_t(_GetR(m)) + int64_t((((uint64_t)_GetR(dHi))<<32) | (uint64_t)_GetR(dLo));
    _SetR(dHi, (((uint64_t)result) >> 32) & 0xFFFF'FFFF);
    _SetR(dLo, ( (uint64_t)result)        & 0xFFFF'FFFF);
  }

  /* _Exec_SMULL {{{4
   * -----------
   */
  void _Exec_SMULL(uint32_t dLo, uint32_t dHi, uint32_t n, uint32_t m, bool setflags) {
    // SMULL § C2.4.166
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    int64_t result = int64_t(_GetR(n)) * int64_t(_GetR(m));
    _SetR(dHi, (((uint64_t)result) >> 32) & 0xFFFF'FFFF);
    _SetR(dLo, ( (uint64_t)result)        & 0xFFFF'FFFF);
  }

  /* _Exec_SSAT {{{4
   * ----------
   */
  void _Exec_SSAT(uint32_t d, uint32_t n, uint32_t saturateTo, SRType shiftT, uint32_t shiftN) {
    // SSAT § C2.4.169
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t operand = _Shift(_GetR(n), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C)); // APSR.C ignored
    auto [result, sat] = _SignedSatQ(operand, saturateTo);
    _SetR(d, _SignExtend(result, saturateTo, 32));
    if (sat)
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Q, 1);
  }

  /* _Exec_STC_STC2 {{{4
   * --------------
   */
  void _Exec_STC_STC2(uint32_t n, uint32_t cp, uint32_t imm32, bool index, bool add, bool wback) {
    // STC, STC2 § C2.4.174
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _ExecuteCPCheck(cp);
    if (!_Coproc_Accepted(cp, _ThisInstr())) {
      _GenerateCoprocessorException();
      return;
    }

    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit check should be performed.
    uint32_t limit;
    bool     applyLimit;
    if (wback && n == 13)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit) {
      do {
        _MemA(addr, 4, _Coproc_GetWordToStore(cp, _ThisInstr()));
        addr += 4;
      } while (!_Coproc_DoneStoring(cp, _ThisInstr()));
    }

    // If the stack pointer is being updated a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_STLB {{{4
   * ----------
   */
  void _Exec_STLB(uint32_t t, uint32_t n) {
    // STLB § C2.4.176
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _MemO(addr, 1, GETBITS(_GetR(t), 0, 7));
  }

  /* _Exec_STLH {{{4
   * ----------
   */
  void _Exec_STLH(uint32_t t, uint32_t n) {
    // STLH § C2.4.180
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _MemO(addr, 2, GETBITS(_GetR(t), 0,15));
  }

  /* _Exec_STL {{{4
   * ---------
   */
  void _Exec_STL(uint32_t t, uint32_t n) {
    // STL § C2.4.175
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _MemO(addr, 4, _GetR(t));
  }

  /* _Exec_STLEXB {{{4
   * ------------
   */
  void _Exec_STLEXB(uint32_t d, uint32_t t, uint32_t n) {
    // STLEXB § C2.4.178
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);

    // IMPLEMENTATION SPECIFIC: Must hold GM lock for duration of
    // ExclusiveMonitorsPass and _MemO, which must occur atomically. _MemO
    // might also acquire a lock, but the mutex is recursive, so this is OK.
    auto lk = _gm.Lock();

    if (_ExclusiveMonitorsPass(addr, 1)) {
      _MemO(addr, 1, GETBITS(_GetR(t), 0, 7));
      _SetR(d, _ZeroExtend(0, 32));
    } else
      _SetR(d, _ZeroExtend(1, 32));
  }

  /* _Exec_STLEXH {{{4
   * ------------
   */
  void _Exec_STLEXH(uint32_t d, uint32_t t, uint32_t n) {
    // STLEXH § C2.4.179
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);

    // IMPLEMENTATION SPECIFIC: Must hold GM lock for duration of
    // ExclusiveMonitorsPass and _MemO, which must occur atomically. _MemO
    // might also acquire a lock, but the mutex is recursive, so this is OK.
    auto lk = _gm.Lock();

    if (_ExclusiveMonitorsPass(addr, 2)) {
      _MemO(addr, 2, GETBITS(_GetR(t), 0,15));
      _SetR(d, _ZeroExtend(0, 32));
    } else
      _SetR(d, _ZeroExtend(1, 32));
  }

  /* _Exec_STLEX {{{4
   * -----------
   */
  void _Exec_STLEX(uint32_t d, uint32_t t, uint32_t n) {
    // STLEX § C2.4.177
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);

    // IMPLEMENTATION SPECIFIC: Must hold GM lock for duration of
    // ExclusiveMonitorsPass and _MemO, which must occur atomically. _MemO
    // might also acquire a lock, but the mutex is recursive, so this is OK.
    auto lk = _gm.Lock();

    if (_ExclusiveMonitorsPass(addr, 4)) {
      _MemO(addr, 4, _GetR(t));
      _SetR(d, _ZeroExtend(0, 32));
    } else
      _SetR(d, _ZeroExtend(1, 32));
  }

  /* _Exec_STM {{{4
   * ---------
   */
  void _Exec_STM(uint32_t n, uint32_t registers, bool wback) {
    // STM, STMIA, STMEA § C2.4.181
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr     = _GetR(n);
    uint32_t endAddr  = _GetR(n) + 4*_BitCount(registers);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool     applyLimit;
    bool     doOperation;
    if (n == 13 && wback && !GETBIT(registers, n)) {
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
      doOperation                 = (!applyLimit || endAddr >= limit);
    } else
      doOperation = true;

    for (int i=0; i<15; ++i) {
      // Memory operation only performed if limit not violated.
      if (GETBIT(registers, i) && doOperation) {
        if (i == n && wback && i != _LowestSetBit(registers))
          _MemA(addr, 4, UNKNOWN_VAL(0)); // encoding T1 only
        else
          _MemA(addr, 4, _GetR(i));

        addr += 4;
      }
    }

    // If the stack pointer is being updated a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, endAddr);
  }

  /* _Exec_STMDB {{{4
   * -----------
   */
  void _Exec_STMDB(uint32_t n, uint32_t registers, bool wback) {
    // STMDB, STMFD § C2.4.182
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n) - 4*_BitCount(registers);

    uint32_t limit;
    bool applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    for (int i=0; i<15; ++i)
      // If R[n] is the SP, memory operation only performed if limit not violated
      if (GETBIT(registers, i) && (!applyLimit || addr >= limit)) {
        _MemA(addr, 4, _GetR(i));
        addr += 4;
      }

    if (wback)
      _SetRSPCheck(n, _GetR(n) - 4*_BitCount(registers));
  }

  /* _Exec_STR_immediate {{{4
   * -------------------
   */
  void _Exec_STR_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // STR (immediate) § C2.4.183
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit)
      _MemU(addr, 4, _GetR(t));

    // If the stack pointer is being updated, a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_STR_register {{{4
   * ------------------
   */
  void _Exec_STR_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // STR (register) § C2.4.184
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t addr   = _GetR(n) + offset;
    _MemU(addr, 4, _GetR(t));
  }

  /* _Exec_STRBT {{{4
   * -----------
   */
  void _Exec_STRBT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // STRBT § C2.4.187
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n) + imm32;
    _MemU_unpriv(addr, 1, GETBITS(_GetR(t), 0, 7));
  }

  /* _Exec_STRHT {{{4
   * -----------
   */
  void _Exec_STRHT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // STRHT § C2.4.194
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n) + imm32;
    _MemU_unpriv(addr, 2, GETBITS(_GetR(t), 0,15));
  }

  /* _Exec_STRT {{{4
   * ----------
   */
  void _Exec_STRT(uint32_t t, uint32_t n, bool postindex, bool add, bool registerForm, uint32_t imm32) {
    // STRT § C2.4.195
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n) + imm32;
    uint32_t data = _GetR(t);
    _MemU_unpriv(addr, 4, data);
  }

  /* _Exec_STRD_immediate {{{4
   * --------------------
   */
  void _Exec_STRD_immediate(uint32_t t, uint32_t t2, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // STRD (immediate) § C2.4.188
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit) {
      _MemA(addr  , 4, _GetR(t ));
      _MemA(addr+4, 4, _GetR(t2));
    }

    // If the stack pointer is being updated a fault will be raised if
    // the limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_STREX {{{4
   * -----------
   */
  void _Exec_STREX(uint32_t d, uint32_t t, uint32_t n, uint32_t imm32) {
    // STREX § C2.4.189
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n) + imm32;

    // IMPLEMENTATION SPECIFIC: Must hold GM lock for duration of
    // ExclusiveMonitorsPass and _MemO, which must occur atomically. _MemO
    // might also acquire a lock, but the mutex is recursive, so this is OK.
    auto lk = _gm.Lock();

    if (_ExclusiveMonitorsPass(addr, 4)) {
      _MemA(addr, 4, _GetR(t));
      _SetR(d, _ZeroExtend(0, 32));
    } else
      _SetR(d, _ZeroExtend(1, 32));
  }

  /* _Exec_STREXB {{{4
   * ------------
   */
  void _Exec_STREXB(uint32_t d, uint32_t t, uint32_t n) {
    // STREXB § C2.4.190
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);

    // IMPLEMENTATION SPECIFIC: Must hold GM lock for duration of
    // ExclusiveMonitorsPass and _MemO, which must occur atomically. _MemO
    // might also acquire a lock, but the mutex is recursive, so this is OK.
    auto lk = _gm.Lock();

    if (_ExclusiveMonitorsPass(addr, 1)) {
      _MemA(addr, 1, GETBITS(_GetR(t), 0, 7));
      _SetR(d, _ZeroExtend(0, 32));
    } else
      _SetR(d, _ZeroExtend(1, 32));
  }

  /* _Exec_STREXH {{{4
   * ------------
   */
  void _Exec_STREXH(uint32_t d, uint32_t t, uint32_t n) {
    // STREXH § C2.4.191
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);

    // IMPLEMENTATION SPECIFIC: Must hold GM lock for duration of
    // ExclusiveMonitorsPass and _MemO, which must occur atomically. _MemO
    // might also acquire a lock, but the mutex is recursive, so this is OK.
    auto lk = _gm.Lock();

    if (_ExclusiveMonitorsPass(addr, 2)) {
      _MemA(addr, 2, GETBITS(_GetR(t), 0,15));
      _SetR(d, _ZeroExtend(0, 32));
    } else
      _SetR(d, _ZeroExtend(1, 32));
  }

  /* _Exec_STRB_immediate {{{4
   * --------------------
   */
  void _Exec_STRB_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // STRB (immediate) § C2.4.185
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit)
      _MemU(addr, 1, GETBITS(_GetR(t), 0, 7));

    // If the stack pointer is being updated a fault will be raised
    // if the limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_STRB_register {{{4
   * -------------------
   */
  void _Exec_STRB_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // STRB (register) § C2.4.186
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t addr   = _GetR(n) + offset;
    _MemU(addr, 1, GETBITS(_GetR(t), 0, 7));
  }

  /* _Exec_STRH_immediate {{{4
   * --------------------
   */
  void _Exec_STRH_immediate(uint32_t t, uint32_t n, uint32_t imm32, bool index, bool add, bool wback) {
    // STRH (immediate) § C2.4.192
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offsetAddr = add ? _GetR(n) + imm32 : _GetR(n) - imm32;
    uint32_t addr       = index ? offsetAddr : _GetR(n);

    // Determine if the stack pointer limit should be checked.
    uint32_t limit;
    bool     applyLimit;
    if (n == 13 && wback)
      std::tie(limit, applyLimit) = _LookUpSPLim(_LookUpSP());
    else
      applyLimit = false;

    // Memory operation only performed if limit not violated.
    if (!applyLimit || offsetAddr >= limit)
      _MemU(addr, 2, GETBITS(_GetR(t), 0,15));

    // If the stack pointer is being updated a fault will be raised if the
    // limit is violated.
    if (wback)
      _SetRSPCheck(n, offsetAddr);
  }

  /* _Exec_STRH_register {{{4
   * -------------------
   */
  void _Exec_STRH_register(uint32_t t, uint32_t n, uint32_t m, bool index, bool add, bool wback, SRType shiftT, uint32_t shiftN) {
    // STRH (register) § C2.4.193
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t offset = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t addr   = _GetR(n) + offset;
    _MemU(addr, 2, GETBITS(_GetR(t), 0,15));
  }

  /* _Exec_SUB_SP_minus_immediate {{{4
   * ----------------------------
   */
  void _Exec_SUB_SP_minus_immediate(uint32_t d, bool setflags, uint32_t imm32) {
    // SUB (SP minus immediate) § C2.4.196
    if (!_ConditionPassed())
      return;

    auto [result, carry, overflow] = _AddWithCarry(_GetSP(), ~imm32, true);
    _SetRSPCheck(d, result);
    TRACE("  newSP=0x%x\n", result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_SUB_SP_minus_register {{{4
   * ---------------------------
   */
  void _Exec_SUB_SP_minus_register(uint32_t d, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // SUB (SP minus register) § C2.4.197
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetSP(), ~shifted, true);
    _SetRSPCheck(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_SUB_immediate {{{4
   * -------------------
   */
  void _Exec_SUB_immediate(uint32_t d, uint32_t n, bool setflags, uint32_t imm32) {
    // SUB (immediate) § C2.4.198
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), ~imm32, true);
    _SetR(d, result);
    if (setflags) {
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
    }
  }

  /* _Exec_SUB_register {{{4
   * ------------------
   */
  void _Exec_SUB_register(uint32_t d, uint32_t n, uint32_t m, bool setflags, SRType shiftT, uint32_t shiftN) {
    // SUB (register) § C2.4.200
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t shifted = _Shift(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    auto [result, carry, overflow] = _AddWithCarry(_GetR(n), ~shifted, true);
    if (d == 15)
      _ALUWritePC(result);
    else {
      _SetR(d, result);
      if (setflags) {
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
        _s.xpsr = CHGBITSM(_s.xpsr, XPSR__V, overflow);
      }
    }
  }

  /* _Exec_SVC {{{4
   * ---------
   */
  void _Exec_SVC() {
    // SVC § C2.4.201
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _CallSupervisor();
  }

  /* _Exec_SXTB {{{4
   * ----------
   */
  void _Exec_SXTB(uint32_t d, uint32_t m, uint32_t rotation) {
    // SXTB § C2.4.205
    if (!_ConditionPassed())
      return;

    uint32_t rotated = _ROR(_GetR(m), rotation);
    _SetR(d, _SignExtend(GETBITS(rotated, 0, 7), 8, 32));
  }

  /* _Exec_SXTH {{{4
   * ----------
   */
  void _Exec_SXTH(uint32_t d, uint32_t m, uint32_t rotation) {
    // SXTH § C2.4.207
    if (!_ConditionPassed())
      return;

    uint32_t rotated = _ROR(_GetR(m), rotation);
    _SetR(d, _SignExtend(GETBITS(rotated, 0,15), 16, 32));
  }

  /* _Exec_TBB {{{4
   * ---------
   */
  void _Exec_TBB(uint32_t n, uint32_t m, bool isTBH) {
    // TBB, TBH § C2.4.208
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t halfwords;
    if (isTBH)
      halfwords = _MemU(_GetR(n) + _LSL(_GetR(m), 1), 2);
    else
      halfwords = _MemU(_GetR(n) + _GetR(m), 1);

    _BranchWritePC(_GetPC() + 2*halfwords);
  }

  /* _Exec_TEQ_immediate {{{4
   * -------------------
   */
  void _Exec_TEQ_immediate(uint32_t n, uint32_t imm32, bool carry) {
    // TEQ (immediate) § C2.4.209
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _GetR(n) ^ imm32;
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    // APSR.V unchanged
  }

  /* _Exec_TEQ_register {{{4
   * ------------------
   */
  void _Exec_TEQ_register(uint32_t n, uint32_t m, SRType shiftT, uint32_t shiftN) {
    // TEQ (register) § C2.4.210
    if (!_ConditionPassed())
      return;

    //EncodingSpecfificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = _GetR(n) ^ shifted;
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    // APSR.v unchanged
  }

  /* _Exec_TST_immediate {{{4
   * -------------------
   */
  void _Exec_TST_immediate(uint32_t n, uint32_t imm32, bool carry) {
    // TST (immediate) § C2.4.211
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result = _GetR(n) & imm32;

    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result, 31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    // APSR.V unchanged
  }

  /* _Exec_TST_register {{{4
   * ------------------
   */
  void _Exec_TST_register(uint32_t n, uint32_t m, SRType shiftT, uint32_t shiftN) {
    // TST (register) § C2.4.212
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    auto [shifted, carry] = _Shift_C(_GetR(m), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C));
    uint32_t result = _GetR(n) & shifted;
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__N, GETBIT(result,31));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Z, _IsZeroBit(result));
    _s.xpsr = CHGBITSM(_s.xpsr, XPSR__C, carry);
    // APSR.V unchanged
  }

  /* _Exec_TT {{{4
   * --------
   */
  void _Exec_TT(uint32_t d, uint32_t n, bool alt, bool forceUnpriv) {
    // TT, TTT, TTA, TTAT § C2.4.213
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t addr = _GetR(n);
    _SetR(d, _TTResp(addr, alt, forceUnpriv));
  }

  /* _Exec_UBFX {{{4
   * ----------
   */
  void _Exec_UBFX(uint32_t d, uint32_t n, uint32_t lsbit, uint32_t widthminus1, uint32_t msbit) {
    // UBFX § C2.4.217
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    if (msbit <= 31)
      _SetR(d, _ZeroExtend(GETBITS(_GetR(n), lsbit, msbit), 32));
    else
      _SetR(d, UNKNOWN_VAL(0));
  }

  /* _Exec_UDF {{{4
   * ---------
   */
  void _Exec_UDF() {
    // UDF § C2.4.218
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    THROW_UNDEFINED();
  }

  /* _Exec_UDIV {{{4
   * ----------
   */
  void _Exec_UDIV(uint32_t d, uint32_t n, uint32_t m) {
    // UDIV § C2.4.219
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t result;
    if (!_GetR(m)) {
      if (_IntegerZeroDivideTrappingEnabled())
        _GenerateIntegerZeroDivide();
      else
        result = 0;
    } else
      result = _GetR(n) / _GetR(m);   // RoundTowardsZero(Real(UInt(R[n])) / Real(UInt(R[m])))

    _SetR(d, result);
  }

  /* _Exec_UMLAL {{{4
   * -----------
   */
  void _Exec_UMLAL(uint32_t dLo, uint32_t dHi, uint32_t n, uint32_t m, bool setflags) {
    // UMLAL § C2.4.227
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint64_t result = uint64_t(_GetR(n)) * uint64_t(_GetR(m)) + ((((uint64_t)_GetR(dHi))<<32) | (uint64_t)_GetR(dLo));
    _SetR(dHi, (((uint64_t)result) >> 32) & 0xFFFF'FFFF);
    _SetR(dLo, ( (uint64_t)result)        & 0xFFFF'FFFF);
  }

  /* _Exec_UMULL {{{4
   * -----------
   */
  void _Exec_UMULL(uint32_t dLo, uint32_t dHi, uint32_t n, uint32_t m, bool setflags) {
    // UMULL § C2.4.228
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint64_t result = uint64_t(_GetR(n)) * uint64_t(_GetR(m));
    _SetR(dHi, (((uint64_t)result) >> 32) & 0xFFFF'FFFF);
    _SetR(dLo, ( (uint64_t)result)        & 0xFFFF'FFFF);
  }

  /* _Exec_USAT {{{4
   * ----------
   */
  void _Exec_USAT(uint32_t d, uint32_t n, uint32_t saturateTo, SRType shiftT, uint32_t shiftN) {
    // USAT § C2.4.237
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    uint32_t operand = _Shift(_GetR(n), shiftT, shiftN, GETBITSM(_s.xpsr, XPSR__C)); // APSR.C ignored
    auto [result, sat] = _UnsignedSatQ(operand, saturateTo);
    _SetR(d, _ZeroExtend(result, 32));
    if (sat)
      _s.xpsr = CHGBITSM(_s.xpsr, XPSR__Q, 1);
  }

  /* _Exec_UXTB {{{4
   * ----------
   */
  void _Exec_UXTB(uint32_t d, uint32_t m, uint32_t rotation) {
    // UXTB § C2.4.245
    if (!_ConditionPassed())
      return;

    uint32_t rotated = _ROR(_GetR(m), rotation);
    _SetR(d, _ZeroExtend(GETBITS(rotated, 0, 7), 32));
  }

  /* _Exec_UXTH {{{4
   * ----------
   */
  void _Exec_UXTH(uint32_t d, uint32_t m, uint32_t rotation) {
    // UXTH § C2.4.247
    if (!_ConditionPassed())
      return;

    uint32_t rotated = _ROR(_GetR(m), rotation);
    _SetR(d, _ZeroExtend(GETBITS(rotated, 0,15), 32));
  }

  /* _Exec_WFE {{{4
   * ---------
   */
  void _Exec_WFE() {
    // WFE § C2.4.304
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    if (_EventRegistered())
      _ClearEventRegister();
    else
      _WaitForEvent();
  }

  /* _Exec_WFI {{{4
   * ---------
   */
  void _Exec_WFI() {
    // WFI § C2.4.305
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _WaitForInterrupt();
  }

  /* _Exec_YIELD {{{4
   * -----------
   */
  void _Exec_YIELD() {
    // YIELD § C2.4.306
    if (!_ConditionPassed())
      return;

    //EncodingSpecificOperations
    _Hint_Yield();
  }

  // }}}3

private:
  CpuState        _s{};
  CpuNest         _n{};
  Device         &_dev;
  SimulatorConfig _cfg;
  SysTickDevice   _sysTickS, _sysTickNS;
  int             _procID;
  LocalMonitor    _lm;
  GlobalMonitor  &_gm;
};
