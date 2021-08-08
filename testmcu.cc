#include "emu2.cc"
#include <string>
#include <signal.h>
#include <histedit.h>
#include <stdio.h>

/* RouterDevice {{{2
 * ============
 * An IDevice which routes to other devices.
 */
template<typename T>
struct RouterDevice :IDevice {
  int Load(phys_t addr, int size, uint32_t flags, uint32_t &v) override {
    IDevice *dev = _Resolve(addr);
    if (!dev) {
      printf("  B:L%2d %x -> BusFault\n", size, addr);
      return -1;
    }

    return dev->Load(addr, size, flags, v);
  }

  int Store(phys_t addr, int size, uint32_t flags, uint32_t v) override {
    IDevice *dev = _Resolve(addr);
    if (!dev) {
      printf("  B:S%2d %x <- 0x%x BusFault", size, addr, v);
      return -1;
    }

    return dev->Store(addr, size, flags, v);
  }

private:
  IDevice *_Resolve(phys_t addr) {
    return static_cast<T*>(this)->Resolve(addr);
  }
};

/* RangeDevice {{{2
 * ===========
 */
struct RangeDevice :IDevice {
  RangeDevice(phys_t base, size_t len) :_base(base), _len(len) {}

  phys_t GetBase() const { return _base; }
  size_t GetLen() const { return _len; }

  bool Decodes(phys_t addr) const {
    return addr >= _base && addr < _base + _len;
  }

protected:
  phys_t _base;
  size_t _len;
};

/* RamDevice {{{2
 * =========
 */
struct RamDevice final :RangeDevice {
  RamDevice(phys_t base, size_t len) :RangeDevice(base, len) {
    _buf = (uint8_t*)new uint32_t[(len+3)/4];
    memset(_buf, 0, len);
  }

  ~RamDevice() {
    delete[] (uint32_t*)_buf;
    _buf = nullptr;
  }

  uint8_t *GetBuf() { return _buf; }

  int Load(phys_t addr, int size, uint32_t flags, uint32_t &v) override {
    if (addr < _base || addr + size - 1 >= _base + _len)
      return -1;

    addr -= _base;
    switch (size) {
      case 4: v = *(uint32_t*)(_buf + addr); break;
      case 2: v = *(uint16_t*)(_buf + addr); break;
      case 1: v = *(uint8_t *)(_buf + addr); break;
      default: ASSERT(false);
    }

    TRACE("L%2d 0x%08x -> 0x%x\n", size, addr+_base, v);
    return 0;
  }

  int Store(phys_t addr, int size, uint32_t flags, uint32_t v) override {
    if (addr < _base || addr + size - 1 >= _base + _len)
      return -1;

    addr -= _base;
    switch (size) {
      case 4: *(uint32_t*)(_buf + addr) = v; break;
      case 2: *(uint16_t*)(_buf + addr) = v; break;
      case 1: *(uint8_t *)(_buf + addr) = v; break;
      default: ASSERT(false);
    }

    TRACE("S%2d 0x%08x <- 0x%x\n", size, addr+_base, v);
    return 0;
  }

private:
  uint8_t *_buf{};
};

/* UartDevice {{{2
 * ==========
 */
struct UartDevice final :RangeDevice {
  UartDevice(phys_t base) :RangeDevice(base, 0x1000) {}

  int Load(phys_t addr, int size, uint32_t flags, uint32_t &v) override {
    return 0;
  }

  int Store(phys_t addr, int size, uint32_t flags, uint32_t v) override {
    for (int i=0; i<4; ++i, v >>= 8)
      if (auto vv = v & 0xFF) {
        if (vv == '\n')
          _Flush();
        else
          _s.push_back(vv);
      }

    return 0;
  }

  void _Flush() {
    if (!_s.size())
      return;

    printf("MSG: %s\n", _s.c_str());
    _s.clear();
  }

private:
  std::string _s;
};

/* TestDevice {{{2
 * ==========
 * Imaginary MCU used for testing purposes.
 *
 * Memory map:
 *   0000_0000    Unused
 *   2000_0000    RAM (1MiB)
 *   4000_0000    UART
 *
 */
struct TestDevice final :RouterDevice<TestDevice> {
  IDevice *Resolve(phys_t addr) {
    if (_ram.Decodes(addr))
      return &_ram;

    if (_uart.Decodes(addr))
      return &_uart;

    return nullptr;
  }

  RamDevice &GetRam() { return _ram; }

private:
  RamDevice  _ram{0x2000'0000, 1*1024*1024};
  UartDevice _uart{0x4000'0000};
};

bool g_sigint = false;
bool g_inDebugPrompt = false;
EditLine *g_el;

static void _OnSigInt(int) {
  g_sigint = true;
  if (g_inDebugPrompt) {
    el_end(g_el);
    exit(1);
  }
}

static int _DebugPrompt(Simulator<TestDevice> &sim) {
  int rc = 0;
  g_inDebugPrompt = true;

  for (;;) {
    int c;
    const char *p = el_gets(g_el, &c);
    if (!p)
      return 1;

    std::string s{p};
    s.pop_back();
    if (s == "c")
      break;

    if (s == "q") {
      rc = 1;
      break;
    }

    if (s == "dump" || s == "d") {
      auto &st = sim.GetCpuState();
      auto &n  = sim.GetCpuNest();
      std::string activeS, pendingS;
      for (int i=0; i<ARRAYLEN(st.excActive); ++i) {
        if (st.excActive[i]) {
          if (activeS.size())
            activeS.push_back(' ');
          char s2[64];
          sprintf(s2, "%d(%x)", i, st.excActive[i]);
          activeS += s2;
        }
        if (st.excPending[i]) {
          if (pendingS.size())
            pendingS.push_back(' ');
          char s2[64];
          sprintf(s2, "%d(%x)", i, st.excPending[i]);
          pendingS += s2;
        }
      }
      printf("  R0  %08x  PM %2x %2x\n", st.r0, st.primaskS, st.primaskNS);
      printf("  R1  %08x  FM %2x %2x\n", st.r1, st.faultmaskS, st.faultmaskNS);
      printf("  R2  %08x  BP %02x %02x\n", st.r2, st.basepriS, st.basepriNS);
      printf("  R3  %08x  CTRL %x %x\n", st.r3, st.controlS, st.controlNS);
      printf("  R4  %08x  %s %s %s\n", st.r4, st.curState == SecurityState_Secure ? "S " : "NS", st.event ? "EV" : "  ", st.pendingReturnOperation ? "PR" : "");
      printf("  R5  %08x  A:%s\n", st.r5, activeS.c_str());
      printf("  R6  %08x  P:%s\n", st.r6, pendingS.c_str());
      printf("  R7  %08x  FPSCR  %08x\n", st.r7, st.fpscr);
      printf("  R8  %08x  PSPLIM %08x %08x\n", st.r8, st.psplimS, st.psplimNS);
      printf("  R9  %08x  MSPLIM %08x %08x\n", st.r9, st.msplimS, st.msplimNS);
      printf("  R10 %08x  MPU    %x %x\n", st.r10, n.mpuCtrlS & 7, n.mpuCtrlNS & 7);
      printf("  R11 %08x  VTOR   %08x %08x\n", st.r11, n.vtorS, n.vtorNS);
      printf("  R12 %08x\n", st.r12);
      printf("  MSP %08x %08x\n", st.spMainS,    st.spMainNS);
      printf("  PSP %08x %08x\n", st.spProcessS, st.spProcessNS);
      printf("  LR  %08x\n", st.lr);
      printf("  PC  %08x\n", st.pc);
      printf("  PSR %08x\n", st.xpsr);
      continue;
    }

    if (s == "dumpmpu" || s == "dm") {
      auto &n  = sim.GetCpuNest();
      printf("  S   MPU %8s %s %s\n", n.mpuCtrlS & 1 ? "ENABLED" : "disabled", n.mpuCtrlS & 2 ? "HFNMIENA" : "", n.mpuCtrlS & 4 ? "PRIVDEFENA" : "");
      for (int i=0; i<NUM_MPU_REGION_S; ++i) {
        printf("  S   %02u) B=%08lx L=%08lx %s AI=%x %s SH=%x\n", i, n.mpuRbarS[i] & ~BITS(0,4), n.mpuRlarS[i] & ~BITS(0,3), n.mpuRlarS[i] & 1 ? "EN" : "  ", (n.mpuRlarS[i]>>1) & 7, n.mpuRbarS[i] & 1 ? "XN" : "  ", (n.mpuRbarS[i]>>3) & 3);
      }
      printf("  NS  MPU %8s %s %s\n", n.mpuCtrlNS & 1 ? "ENABLED" : "disabled", n.mpuCtrlNS & 2 ? "HFNMIENA" : "", n.mpuCtrlNS & 4 ? "PRIVDEFENA" : "");
      for (int i=0; i<NUM_MPU_REGION_NS; ++i) {
        printf("  NS  %02u) B=%08lx L=%08lx %s AI=%x %s SH=%x\n", i, n.mpuRbarNS[i] & ~BITS(0,4), n.mpuRlarNS[i] & ~BITS(0,3), n.mpuRlarNS[i] & 1 ? "EN" : "  ", (n.mpuRlarNS[i]>>1) & 7, n.mpuRbarNS[i] & 1 ? "XN" : "  ", (n.mpuRbarNS[i]>>3) & 3);
      }
      printf("  SAU %s %s\n", n.sauCtrl & 1 ? "EN" : "  ", n.sauCtrl & 2 ? "ALLNS" : "     ");
      for (int i=0; i<NUM_SAU_REGION; ++i) {
        printf("  SAU %02u) B=%08x L=%08lx %s %s\n", i, n.sauRbar[i], n.sauRlar[i] & ~BITS(0,1), n.sauRlar[i] & 1 ? "EN" : "  ", n.sauRlar[i] & 2 ? "NSC" : "   ");
      }
      continue;
    }

    if (s == "s") {
      g_sigint = true;
      break;
    }

    printf("unknown command: \"%s\"\n", s.c_str());
  }

  g_inDebugPrompt = false;
  return rc;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <program.bin>\n", argv[0]);
    return 2;
  }

  TestDevice dev;
  SimpleSimulatorConfig cfg;

  FILE *f = fopen(argv[1], "rb");
  fseek(f, 0, SEEK_END);
  size_t flen = ftell(f);
  fseek(f, 0, SEEK_SET);

  if (flen > dev.GetRam().GetLen())
    return 1;

  if (fread(dev.GetRam().GetBuf(), 1, flen, f) != flen) {
    fclose(f);
    return 1;
  }

  fclose(f);

  cfg.initialVtor = 0x2000'0000;

  g_el = el_init(argv[0], stdin, stdout, stderr);

  struct sigaction sa = {};
  sa.sa_handler = _OnSigInt;
  sa.sa_flags   = SA_RESTART;
  sigaction(SIGINT, &sa, NULL);

  GlobalMonitor gm;
  Simulator sim(dev, gm, cfg);
  IntrBox   intrBox{sim};
  uint32_t  i = 0;
  for (;;) {
    if unlikely (g_sigint) {
      g_sigint = false;
      if (_DebugPrompt(sim) > 0)
        break;
    }
    sim.TopLevel();

    auto exitCause = sim.GetExitCause();
    if (exitCause & EXIT_CAUSE__WFI) {
      printf("=> waiting for interrupt i=%u\n", i++);
      intrBox.WaitForInterrupt();
    }
    if (exitCause & EXIT_CAUSE__SLEEP_ON_EXIT) {
      printf("=> sleeping on exit i=%u\n", i++);
      intrBox.WaitForInterrupt();
    }
  }

  return 0;
}
