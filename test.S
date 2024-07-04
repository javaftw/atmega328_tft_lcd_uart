#include <avr/io.h>
/*
Data types: char is 8 bits, int is 16 bits, long is 32 bits, long long is 64 bits, float and double are 32 bits

Call-used registers (r18-r27, r30-r31): 
- May be allocated by gcc for local data. 
- Use them freely in assembler subroutines.
- Calling C subroutines can clobber any of them - the caller is responsible for saving and restoring.

Call-saved registers (r2-r17, r28-r29): 
- May be allocated by gcc for local data. Calling C subroutines leaves them unchanged.
- Assembler subroutines are responsible for saving and restoring these registers, if changed.
- r29:r28 (Y pointer) is used as a frame pointer (points to local data on stack) if necessary.
- The requirement for the callee to save/preserve the contents of these registers even applies in situations where the compiler assigns them for argument passing.

Fixed registers (r0, r1):
- Never allocated by gcc for local data, but often used for fixed purposes:
----r0 - temporary register, can be clobbered by any C code (except interrupt handlers which save it), may be used to remember something for a while within one piece of assembler code
----r1 - assumed to be always zero in any C code, may be used to remember something for a while within one piece of assembler code, but must then be cleared after use (clr r1). This includes any use of the [f]mul[s[u]] instructions, which return their result in r1:r0. Interrupt handlers save and clear r1 on entry, and restore r1 on exit (in case it was non-zero).

Function call conventions: 
- Arguments - allocated left to right, r25 to r8. 
- All arguments are aligned to start in even-numbered registers (odd-sized arguments, including char, have one free register above them). This allows making better use of the MOVW instruction on the enhanced core.

Return values: 
- 8-bit in r24 (not r25!), 
- 16-bit in r25:r24, 
- up to 32 bits in r22-r25,
- up to 64 bits in r18-r25. 
- 8-bit return values are zero/sign-extended to 16 bits by the called function (unsigned char is more efficient than signed char - just clr r25). 
- Arguments to functions with variable argument lists (printf etc.) are all passed on stack, and 
- char is extended to int.
*/


;----internal designations

;----internal variables (declared here, used here)

;----external variables (declared outside, used here)

;----global variables (declared here, used outside)

.global add_8bit

add_8bit:
    ldi r18,0x08     ;r18 now contains 0xnn
    ldi r19,0x08    ;r19 now caontains 0xmm
    add r18,r19     ;add r19 to r18
    clr r24         ;clear register 24 (return vaue register)
                    ;so r24 now contains 0x00
    add r24,r18     ;add r18 to r24 si that it now contains the sum of r18 and 19
    ret

.global add_16bit

add_16bit:
    ldi r18,hi8(0x01ff)
    ldi r19,lo8(0x01ff)
    ldi r20,hi8(0x01ff)
    ldi r21,lo8(0x01ff)
    ret



