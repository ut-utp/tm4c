
# We have to step at least once before quitting or else we get into a bad state:
stepi

set print pretty on

# print demangled symbols
set print asm-demangle on

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
break UserHardFault
break rust_begin_unwind

break main

# We have to step at least once before quitting or else we get into a bad state:
c # continue to main

py import duel

# ptype <var | type>

## Move the following to TLT (TODO!):
define reset
    monitor reset halt
end

define current_exec
    python print(gdb.objfiles()[0].filename)
end

# Uses descriptions/information sourced from here: https://interrupt.memfault.com/blog/cortex-m-fault-debug
define hardfault
    python

def get_mem(addr):
    return int(gdb.execute(f"x(0x{addr:X})", to_string = True).split("\t")[1].split(" ")[0].rstrip(), 0)

CFSR_ADDR = 0xE000ED28

HFSR_ADDR = 0xE000ED2C

# Configurable Fault Status Register
CFSR = get_mem(CFSR_ADDR)
# Usage Fault Status Register
UFSR = CFSR >> 16
# BusFault Status Register
BFSR = CFSR >> 8 & 0b11111111
# MemManage Status Register
MMFSR = CFSR & 0b11111111

# HardFault Status Register
HFSR = get_mem(HFSR_ADDR)

ufsr_faults = {
    0: "UNDEFINSTR: Indicates an undefined instruction was executed. This can happen on exception exit if the stack got corrupted. A compiler may emit undefined instructions as well for code paths that should be unreachable.",
    1: "INVSTATE: Indicates the processor has tried to execute an instruction with an invalid Execution Program Status Register (EPSR) value. Among other things the ESPR tracks whether or not the processor is in thumb mode state.",
    2: "INVPC: Indicates an integrity check failure on EXC_RETURN. EXC_RETURN is the value branched to upon return from an exception. If this fault flag is set, it means a reserved EXC_RETURN value was used on exception exit.",
    3: "NOCP: Indicates that a Cortex-M coprocessor instruction was issued but the coprocessor was disabled or not present.",
    8: "UNALIGNED: Indicates an unaligned access operation occurred.",
    9: "DIVBYZERO: Indicates a divide instruction was executed where the denominator was zero.",
}

bfsr_faults = {
    0: "IBUSERR: ",
    1: "PRECISERRR: Indicates that the instruction which was executing prior to exception entry triggered the fault.",
    2: "IMPRECISERR: This flag is very important. It tells us whether or not the hardware was able to determine the exact location of the fault.",
    3: "UNSTKERR: Indicates that a fault occurred trying to return from an exception.",
    4: "STKERR: Indicates that a fault occurred during exception entry. Both are situations where the hardware is automatically saving state on the stack. One way this error may occur is if the stack in use overflows off the valid RAM address range while trying to service an exception.",
    5: "LSPERR: Indicates that a fault occurred during lazy state preservation. Both are situations where the hardware is automatically saving state on the stack. One way this error may occur is if the stack in use overflows off the valid RAM address range while trying to service an exception.",
    # 6: "Reserved",
    7: "BFARVALID: Indicates that the Bus Fault Address Register (BFAR), a 32 bit register located at 0xE000ED38, holds the address which triggered the fault.",
}

mmfsr_faults = {
    0: "IACCVIOL: Indicates that an attempt to execute an instruction triggered an MPU or Execute Never (XN) fault.",
    1: "DACCVIOL: Indicates that a data access triggered the MemManage fault.",
    # 2: "",
    3: "MUNSTKERR: Indicates that a fault occurred while returning from an exception.",
    4: "MSTKERR: Indicates that a MemManage fault occurred during exception entry.",
    5: "MLSPERR: Indicates that a MemManage fault occurred during lazy state preservation.",
    # 6: "",
    7: "MMARVALID: Indicates that the MemManage Fault Address Register (MMFAR), a 32 bit register located at 0xE000ED34, holds the address which triggered the MemManage fault.",
}

hfsr_faults = {
    1: "VECTTBL: Indicates a fault occurred because of an issue reading from an address in the vector table. This is pretty atypical but could happen if there is a bad address in the vector table and an unexpected interrupt fires.",
    30: "FORCED: This means a configurable fault was escalated to a HardFault, either because the configurable fault handler was not enabled or a fault occurred within the handler.",
    31: "DEBUGEVT: Indicates that a debug event (i.e executing a breakpoint instruction) occurred while the debug subsystem was not enabled.",
}

def print_errs(val, prefix, dict):
    for bit, msg in dict.items():
        if (val & (1 << bit)) != 0:
            print(f"[{bit:2}] {prefix}: {msg}")

print_errs(UFSR, "UFSR", ufsr_faults)
print_errs(BFSR, "BFSR", bfsr_faults)
print_errs(MMFSR, "MMFSR", mmfsr_faults)

print_errs(HFSR, "HFSR", hfsr_faults)

print("--")

    end
end

define force_precise
set *(0xE000E008 as *mut u32)=(*(0xE000E008 as *mut u32) | 1<<1)
end

define print_bfar
p/x *(0xE000ED38 as *mut u32)
end

define reflash
    python
import subprocess

prog = gdb.objfiles()[0].filename

# TODO: detect debug or release, etc.
gdb.execute("shell cargo b")

# gdb.execute('monitor program "{}"'.format(prog))
gdb.execute('load')
gdb.execute('reset')
end
