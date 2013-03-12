// Copyright 2013 Thai-Duong Nguyen. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//     * Neither the name of Thai-Duong Nguyen nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERPRETER_H
#define INTERPRETER_H
namespace v8 {
namespace internal {

#define USE_REDIRECTION

#define MEM_GET_UINT32(X) (*(reinterpret_cast<unsigned int*>(X)))
#define MEM_GET_UINT8(X) (*(reinterpret_cast<unsigned char*>(X)))
#define MEM_GET_UINT16(X) (*(reinterpret_cast<unsigned short*>(X)))

#define MEM_GET_INT32(X) (*(reinterpret_cast<int*>(X)))
#define MEM_GET_INT8(X) (*(reinterpret_cast<char*>(X)))
#define MEM_GET_INT16(X) (*(reinterpret_cast<short*>(X)))

#define DEFAULT_STACK_SIZE 1024*1024
#define X87_STACK_SIZE 8
#define NUMBER_OF_XMM_REGISTERS 8

typedef unsigned int* register32_ptr;
typedef unsigned short* register16_ptr;
typedef unsigned char* register8_ptr;

#define REG_EAX 0
#define REG_ECX 1
#define REG_EDX 2
#define REG_EBX 3
#define REG_ESP 4
#define REG_EBP 5
#define REG_ESI 6
#define REG_EDI 7
#define MAX_REGS 8
class Interpreter
{
public:
	enum OPCODES{
		OP_ADD_EAX_32 = 0x05,
		OP_ADD_RM_8	= 0x83,//	0
		OP_ADD_RM_32 = 0x81,//	0
		OP_ADD_REG_RM = 0x03,
		OP_ADDSD_XMMREG_XMMREG = 0xF20F58,
		OP_AND_RM_32 = 0x81, //	4
		OP_AND_RM_8 = 0x83,//	4
		OP_AND_RM_REG = 0x21,
		OP_AND_REG_RM = 0x23,
		OP_AND_EAX_32 = 0x25,
		OP_BT_OPRD_REG = 0x0F, //	0xA3
		OP_BTS_OPRD_REG = 0x0F, //	0xAB
		OP_CALL_32 = 0xE8,	
		OP_CALL_OPRD = 0xFF	
	};
	enum Register{
		EAX = 0,
		ECX = 1,
		EDX = 2,
		EBX = 3,
		ESP = 4,
		EBP = 5,
		ESI = 6,
		EDI = 7,
		NUMBER_OF_REGISTERS
	};
	enum Flags {
#ifdef _ARM_DEVICE_
        _CF = 1 << 29,
        _PF = 1 << 25, //not available
        _AC = 1 << 24, //not available
        _ZF = 1 << 30,
        _SF = 1 << 31, //nagative
        _DF = 1 << 23, //not available
        _OF = 1 << 28,
#else
		_CF = 1,
		_PF = 1 << 2,
		_AC = 1 << 4,
		_ZF = 1 << 6,
		_SF = 1 << 7,
		_DF = 1 << 10,
		_OF = 1 << 11,						
#endif
		/*
		F_CARRY = 1,
		F_PARITY = 1 << 1,
		F_AUXILIARY = 1 << 2,
		F_ZERO = 1 << 3,
		F_SIGN = 1 << 4,
		F_TRAP = 1 << 5,
		F_INTERRUPT_ENABLE = 1 << 6,
		F_DIRECT_FLAG = 1 << 7,
		F_OVERFLOW = 1 << 8,
		F_IO_PRIVILEGE_LEVEL = 1 << 9,
		F_NESTED_TASK = 1 << 10,
		F_RESUME = 1 << 11,*/

		F_OVER_FLOW	 = _OF,						
		//F_NO_OVER_FLOW = 0x01,					//
		F_BELOW		= _CF,
		F_NEITHER_ABOVE_NOR_EQUAL = _CF,
		//F_NOT_BELOW	= 0x03,
		//F_ABOVE_OR_EQUAL = 0x03,
		F_EQUAL = _ZF,
		F_ZERO  = _ZF,
		//F_NOT_EQUAL = 0x05,
		//F_NOT_ZERO = 0x05,
		//F_BELOW_OR_EQUAL = 0x06,
		//F_NOT_ABOVE = 0x06,
		//F_NEITHER_BELOW_NOR_EQUAL = 0x07,
		//F_ABOVE = 0x07,
		F_SIGN	= _SF,
		//F_NO_SIGN = 0x09,
		F_PARITY = _PF,
		F_PARITY_EVEN = _PF,
		//F_PARITY_ODD = 0x0B,
		//F_LESS = 0x0B,
		//F_NEITHER_GREATER_NOR_EQUAL = 0x0C,
		//F_NOT_LESS = 0x0C,
		//F_GREATER_OR_EQUAL = 0x0D,
		//F_LESS_OR_EQUAL = 0x0D,
		//F_NOT_GREATER = 0x0E,
		//F_NEITHER_LESS_OR_EQUAL = 0x0E,
		//F_GREATER = 0x0F
	};



public:
	Interpreter(unsigned char *code = 0, unsigned int entry = 0, unsigned int stackSize = DEFAULT_STACK_SIZE);
	virtual ~Interpreter();
	void setCode(unsigned char *code, unsigned int entry) {
		mCode = code;
		ip = mCode + entry;
	};
	void* execute_code(unsigned char *code, unsigned char* entry, void* function, void* receiver, int argc, void*** args);
	int execute_regexp_code(void* matcher_func,
                          void* input,
                          int start_offset,
						  void* input_start,
                          void* input_end,
                          void* output,
                          void* stack_base,						  
                          int direct_call);
	
protected:
	void pushCallStack();
	void popCallStack();

	inline unsigned char readUINT8(unsigned char rm);
	inline unsigned char readUINT8_keepIP(unsigned char rm);
	inline unsigned short readUINT16(unsigned char rm);
	inline unsigned int readUINT32(unsigned char rm);
	inline unsigned int readUINT32_keepIP(unsigned char rm);

	inline void writeUINT8(unsigned char rm, unsigned char value);
	inline void writeUINT16(unsigned char rm, unsigned short value);
	inline void writeUINT32(unsigned char rm, unsigned int value);

	unsigned int decodeRM(unsigned char rm, int *size, bool *isAddr);
	inline unsigned int decodeSIB(unsigned char sib);
	
	void executeOpCode();
	virtual bool callBuiltinFunction(unsigned int adr){return false;};
	virtual void* getExternalFunction(unsigned int adr) {return 0;};
protected:
	void _enterExitFrame();
	void _enterAPIExitFrame();
	void _leaveExitFrame();
	void CStubCore();
	void ArgumentTrampoline();
	void ExitArgumentTrampoline();
	void JSConstructCall();
	virtual void checkInlineCache() {};
protected:
	unsigned int m_c_entry_fp_address;
	unsigned int m_context_address;
    unsigned int m_scope_depth;
	unsigned int m_handler_address;
	unsigned int m_external_caught;
	unsigned int m_pending_exception;
	unsigned int m_runtime_gc_function;
	unsigned int m_terminal_exception;
	unsigned int m_the_hole_value_location;
	unsigned int m_fp_address;

protected:
	unsigned int mRegisters[NUMBER_OF_REGISTERS];
	register16_ptr AX, BX, CX, DX, SP, BP, SI, DI;
	register8_ptr  AH, AL, BH, BL, CH, CL, DH, DL;
	register32_ptr eax, ebx, ecx, edx, esp, ebp, esi, edi;
	register8_ptr  mRegisters8[NUMBER_OF_REGISTERS];
	double		   mXMMRegisters[NUMBER_OF_XMM_REGISTERS];

	unsigned char* ip; //instruction pointer
	unsigned int	mStackSize;
	unsigned char* mStack;
	unsigned char* mCode;
	unsigned int  mFlags;
	unsigned int  mX87SP;
    bool     mX87StackStatus[X87_STACK_SIZE];
	double   mX87Stack[X87_STACK_SIZE];
	unsigned int mFPFlags;
	unsigned char *mStartStack;
	bool	 mDirectionFlag;

};

enum ByteCodes {
    OP_PUSHAD,
    OP_POPAD,
    OP_PUSHFD,
    OP_POPFD,
    OP_PUSH_8,
    OP_PUSH_16,
    OP_PUSH_32,
    OP_PUSH_REG,
    OP_PUSH_OPRD,
    OP_PUSH_LB,
    OP_POP_REG,
    OP_POP_OPRD,
    OP_ENTER_8,
    OP_LEAVE,
    OP_MOV_B_REG_OPRD,
    OP_MOV_B_OPRD_8,
    OP_MOV_B_OPRD_REG,
    OP_MOV_W_REG_OPRD,
    OP_MOV_W_OPRD_REG,
    OP_MOV_REG_8,
    OP_MOV_REG_16,
    OP_MOV_REG_32,
    OP_MOV_REG_OPRD,
    OP_MOV_REG_REG,
    OP_MOV_OPRD_8,
    OP_MOV_OPRD_16,
    OP_MOV_OPRD_32,
    OP_MOV_OPRD_REG,
    OP_MOVSX_B_REG_OPRD,
    OP_MOVSX_W_REG_OPRD,
    OP_MOVZX_B_REG_OPRD,
    OP_MOVZX_W_REG_OPRD,
    OP_CMOV_REG_32,
    OP_CMOV_REG_OPRD,
    OP_CLD,
    OP_REP_MOVS,
    OP_REP_STOS,
    OP_STOS,
    OP_XCHG_REG_REG,
    OP_ADC_REG_32,
    OP_ADC_REG_OPRD,
    OP_ADD_REG_OPRD,
    OP_ADD_OPRD_8,
    OP_ADD_OPRD_16,
    OP_ADD_OPRD_32,
    OP_AND_REG_32,
    OP_AND_REG_OPRD,
    OP_AND_OPRD_REG,
    OP_AND_OPRD_8,
    OP_AND_OPRD_16,
    OP_AND_OPRD_32,
    OP_CMPB_OPRD_8,
    OP_CMPB_REG_OPRD,
    OP_CMPB_OPRD_REG,
    OP_CMPB_AL_OPRD,
    OP_CMPW_AX_OPRD,
    OP_CMPW_OPRD_16,
    OP_CMP_REG_32,
    OP_CMP_REG__OPRD,
    OP_CMP_OPRD_8,
    OP_CMP_OPRD_16,
    OP_CMP_OPRD_32,
    OP_DEC_B_REG,
    OP_DEC_REG,
    OP_DEC_OPRD,
    OP_CDG,
    OP_IDIV_REG,
    OP_IMUL_REG,
    OP_IMUL_REG_OPRD,
    OP_IMUL_REG_REG_32,
    OP_INC_REG,
    OP_INC_OPRD,
    OP_LEA_REG_OPRD,
    OP_MUL_REG,
    OP_NEG_REG,
    OP_NOT_REG,
    OP_OR_REG_32,
    OP_OR_REG_OPRD,
    OP_OR_OPRD_REG,
    OP_OR_OPRD_8,
    OP_OR_OPRD_16,
    OP_OR_OPRD_32,
    OP_RCL_REG_8,
    OP_SAR_REG_8,
    OP_SAR_CL_REG,
    OP_SBB_REG_OPRD,
    OP_SHLD_REG_OPRD,
    OP_SHL_REG_8,
    OP_SHL_CL_REG,
    OP_SHRD_REG_OPRD,
    OP_SHR_REG_8,
    OP_SHR_CL_REG,
    OP_SUBB_OPRD_8,
    OP_SUBB_REG_OPRD,
    OP_SUB_OPRD_8,
    OP_SUB_OPRD_16,
    OP_SUB_OPRD_32,
    OP_SUB_REG_OPRD,
    OP_SUB_OPRD_REG,
    OP_TEST_REG_8,
    OP_TEST_REG_16,
    OP_TEST_REG_32,
    OP_TEST_REG_OPRD,
    OP_TEST_B_REG_OPRD,
    OP_TEST_OPRD_8,
    OP_TEST_OPRD_16,
    OP_TEST_OPRD_32,
    OP_TEST_B_OPRD_8,
    OP_XOR_REG_32,
    OP_XOR_REG_OPRD,
    OP_XOR_OPRD_REG,
    OP_XOR_OPRD_8,
    OP_XOR_OPRD_16,
    OP_XOR_OPRD_32,
    OP_BT_OPRD_REG,
    OP_BTS_OPRD_REG,
    OP_HLT,
    OP_INT3,
    OP_NOP,
    OP_RDTSC,
    OP_RET_16,
    OP_CALL_32,
    OP_CALL_OPRD,
    OP_CALL_REG,
    OP_JMP_32,
    OP_JMP_REG,
    OP_JMP_OPRD,
    OP_J_SHORT_OVERFLOW,
    OP_J_SHORT_NO_OVERFLOW,
    OP_J_SHORT_BELOW,
    OP_J_SHORT_ABOVE_EQUAL,
    OP_J_SHORT_EQUAL,
    OP_J_SHORT_NOT_EQUAL,
    OP_J_SHORT_BELOW_EQUAL,
    OP_J_SHORT_ABOVE,
    OP_J_SHORT_NEGATIVE,
    OP_J_SHORT_POSITIVE,
    OP_J_SHORT_PARITY_EVEN,
    OP_J_SHORT_PARITY_ODD,
    OP_J_SHORT_LESS,
    OP_J_SHORT_GREATER_EQUAL,
    OP_J_SHORT_LESS_EQUAL,
    OP_J_SHORT_GREATER,
    OP_J_FAR_OVERFLOW,
    OP_J_FAR_NO_OVERFLOW,
    OP_J_FAR_BELOW,
    OP_J_FAR_ABOVE_EQUAL,
    OP_J_FAR_EQUAL,
    OP_J_FAR_NOT_EQUAL,
    OP_J_FAR_BELOW_EQUAL,
    OP_J_FAR_ABOVE,
    OP_J_FAR_NEGATIVE,
    OP_J_FAR_POSITIVE,
    OP_J_FAR_PARITY_EVEN,
    OP_J_FAR_PARITY_ODD,
    OP_J_FAR_LESS,
    OP_J_FAR_GREATER_EQUAL,
    OP_J_FAR_LESS_EQUAL,
    OP_J_FAR_GREATER,
    OP_FISTTP_S_OPRD,
    OP_FISTTP_D_OPRD,
    OP_FABS,
    OP_FCHS,
    OP_FCOS,
    OP_FSIN,
    OP_FADD_8,
    OP_FSUB_8,
    OP_FMUL_8,
    OP_FDIV_8,
    OP_FISUB_S_OPRD,
    OP_FADDP_8,
    OP_FSUBP_8,
    OP_FSUBRP_8,
    OP_FMULP_8,
    OP_FDIVP_8,
    OP_FPREM,
    OP_FPREM1,
    OP_FXCH_8,
    OP_FINCSTP,
    OP_FFREE_8,
    OP_FTST,
    OP_FUCOMP_8,
    OP_FUCOMPP,
    OP_FUCOMI_8,
    OP_FUCOMIP,
    OP_FCOMPP,
    OP_FNSTSW_AX,
    OP_FWAIT,
    OP_FNCLEX,
    OP_FRNDINT,
    OP_SAHF,
    OP_SETCC_OVERFLOW_REG,
    OP_SETCC_NO_OVERFLOW_REG,
    OP_SETCC_BELOW_REG,
    OP_SETCC_ABOVE_EQUAL_REG,
    OP_SETCC_EQUAL_REG,
    OP_SETCC_NOT_EQUAL_REG,
    OP_SETCC_BELOW_EQUAL_REG,
    OP_SETCC_ABOVE_REG,
    OP_SETCC_NEGATIVE_REG,
    OP_SETCC_POSITIVE_REG,
    OP_SETCC_PARITY_EVEN_REG,
    OP_SETCC_PARITY_ODD_REG,
    OP_SETCC_LESS_REG,
    OP_SETCC_GREATER_EQUAL_REG,
    OP_SETCC_LESS_EQUAL_REG,
    OP_SETCC_GREATER_REG,
    OP_CPUID,
    OP_CVTTSS2SI_REG_OPRD,
    OP_CVTTSD2SI_REG_OPRD,
    OP_CVTSI2SD_XMMREG_OPRD,
    OP_CVTSS2SD_XMMREG_XMMREG,
    OP_ADDSD_XMMREG_XMMREG,
    OP_SUBSD_XMMREG_XMMREG,
    OP_MULSD_XMMREG_XMMREG,
    OP_DIVSD_XMMREG_XMMREG,
    OP_XORPD_XMMREG_XMMREG,
    OP_SQRTSD_XMMREG_XMMREG,
    OP_COMISD_XMMREG_XMMREG,
    OP_UCOMISD_XMMREG_XMMREG,
    OP_MOVDQA_XMMREG_XMMREG,
    OP_MOVDQA_XMMREG_OPRD,
    OP_MOVDQA_OPRD_XMMREG,
    OP_MOVDQU_XMMREG_OPRD,
    OP_MOVDQU_OPRD_XMMREG,
    OP_MOVDBL_XMMREG_OPRD,
    OP_MOVDBL_OPRD_XMMREG,
    OP_MOVD_XMMREG_OPRD,
    OP_MOVSD_XMMREG_XMMREG,
    OP_PXOR_XMMREG_XMMREG,
    OP_PTEST_XMMREG_XMMREG,
    OP_MOVNTDQA_XMMREG_OPRD,
    OP_MOVNTDQ_OPRD_XMMREG,
	OP_ENTER_EXIT_FRAME,
	OP_ENTER_API_EXIT_FRAME,
	OP_ENTER_FRAME,
	OP_THROW_OOME, //throw out of memory exception
	OP_THROW_TME,  // throw_termination_exception
	OP_THROW_NME, //throw normal exception
	OP_CALL_C_BUILTIN,
    NUMBER_OF_OPCODE
};

}
}

#endif