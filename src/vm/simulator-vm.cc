// Copyright 2008 the V8 project authors. 
// Copyright 2013 Thai-Duong Nguyen
//
// All rights reserved.
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
//     * Neither the name of Google Inc. nor the names of its
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

#include <stdlib.h>
#include <math.h>
#include <cstdarg>
#include "v8.h"
#include "assert.h"
#if defined(V8_TARGET_ARCH_VM)

#include "disasm.h"
#include "assembler.h"
#include "simulator-vm.h"


// Only build the simulator if not compiling for real ARM hardware.
namespace v8 {
namespace internal {
static v8::internal::Thread::LocalStorageKey simulator_key;

#define REDIRECTION_HASH_TAB_SIZE  337

class Redirection;
Redirection *redirectionHashTab[REDIRECTION_HASH_TAB_SIZE];

bool Simulator::initialized_ = false;
class Redirection {
 public:
  Redirection(void* external_function, bool fp_return)
      : external_function_(external_function),
        swi_instruction_(0xFB), //0xFB Call redirect
        fp_return_(fp_return),
        next_(NULL) {	
    next_ = NULL;
  }

  void* address_of_swi_instruction() {
    return reinterpret_cast<void*>(&swi_instruction_);
  }

  void* external_function() { return external_function_; }
  bool isFPReturn() {return fp_return_;};
  
  static Redirection* Get(void* external_function, bool fp_return) {

    unsigned int key = reinterpret_cast<unsigned int>(external_function);
    Redirection* current = redirectionHashTab[key % REDIRECTION_HASH_TAB_SIZE];
    
    for (; current != NULL; current = current->next_) {
    
      if (current->external_function_ == external_function) return current;
    }
    
	Redirection *newRedirect = new Redirection(external_function, fp_return);
    newRedirect->next_ = current;
    redirectionHashTab[key % REDIRECTION_HASH_TAB_SIZE] = newRedirect;
    return newRedirect;
  }

  static Redirection* FromSwiInstruction(unsigned char* addr_of_swi) {    
    unsigned char* addr_of_redirection =
        addr_of_swi - OFFSET_OF(Redirection, swi_instruction_);
    return reinterpret_cast<Redirection*>(addr_of_redirection);
  }
  inline Redirection* next() {return next_;}
 private:
  void* external_function_;
  unsigned char swi_instruction_;  
  bool fp_return_;
  Redirection* next_;
};

void* execute_code(void* entry_code, unsigned char* entry, void* function, void* receiver, int argc, void*** args)
{	
	return Simulator::current()->execute_code((unsigned char*)entry_code, entry, function, receiver, argc, args);
}
int execute_regexp_code(void* matcher_func,
                          void* input,
                          int start_offset,
						  void* input_start,
                          void* input_end,
                          void* output,
                          void* stack_base,						  
                          int direct_call)
{
	return Simulator::current()->execute_regexp_code(matcher_func, input, start_offset, input_start, input_end, output, stack_base, direct_call);
}
Simulator::Simulator():
Interpreter()
{
    memset(redirectionHashTab, 0, sizeof(redirectionHashTab));
	initialized_ = false;
}
Simulator::~Simulator()
{
}
void Simulator::Initialize()
{
	if (initialized_) return;
	simulator_key = v8::internal::Thread::CreateThreadLocalKey();
	initialized_ = true;
	ExternalReference::set_redirector(&RedirectExternalReference);
}
// Calls into the V8 runtime are based on this very simple interface.
// Note: To be able to return two values from some calls the code in runtime.cc
// uses the ObjectPair which is essentially two 32-bit values stuffed into a
// 64-bit value. With the code below we assume that all runtime calls return
// 64 bits of result. If they don't, the r1 result register contains a bogus
// value, which is fine because it is caller-saved.
typedef int64_t (*SimulatorRuntimeCall)(uint32_t arg0,
                                        uint32_t arg1,
                                        uint32_t arg2,
                                        uint32_t arg3);
typedef double (*SimulatorRuntimeFPCall)(uint32_t arg0,
                                         uint32_t arg1,
                                         uint32_t arg2,
                                         uint32_t arg3);

// This signature supports direct call in to API function native callback
// (refer to InvocationCallback in v8.h).
typedef v8::Handle<v8::Value> (*SimulatorRuntimeDirectApiCall)(uint32_t arg0);

// This signature supports direct call to accessor getter callback.
typedef v8::Handle<v8::Value> (*SimulatorRuntimeDirectGetterCall)(uint32_t arg0,
                                                                  uint32_t arg1);
void* Simulator::getExternalFunction(unsigned int adr)
{
#ifdef USE_REDIRECTION
	Redirection *ref = Redirection::FromSwiInstruction(reinterpret_cast<unsigned char*>(adr));
	return ref->external_function();
#else
	return reinterpret_cast<void*>(adr);
#endif
}
bool Simulator::callBuiltinFunction(unsigned int adr)
{

	Redirection* redirection = Redirection::FromSwiInstruction(ip - 1);
	void* external = redirection->external_function();
	
	if (redirection->isFPReturn())
	{
		SimulatorRuntimeFPCall target =
			reinterpret_cast<SimulatorRuntimeFPCall>(external);
		unsigned int ret_adr = MEM_GET_UINT32(mRegisters[ESP]);
		ip = (unsigned char*)(ret_adr);
		double result = target(MEM_GET_UINT32(mRegisters[ESP] + 4), 
			MEM_GET_UINT32(mRegisters[ESP] + 8),
			MEM_GET_UINT32(mRegisters[ESP] + 12),
			MEM_GET_UINT32(mRegisters[ESP] + 16));
		uint32_t *retBuff = (uint32_t*)&result;
		*eax = retBuff[0];
		*edx = retBuff[1];
		mRegisters[ESP] += 4;
		ip = (unsigned char*)(ret_adr);
	} else {
		SimulatorRuntimeCall target = reinterpret_cast<SimulatorRuntimeCall>(external);
		unsigned int ret_adr = MEM_GET_UINT32(mRegisters[ESP]);
		ip = (unsigned char*)(ret_adr);
		int64_t ret = target(MEM_GET_UINT32(mRegisters[ESP] + 4),
			MEM_GET_UINT32(mRegisters[ESP] + 8),
			MEM_GET_UINT32(mRegisters[ESP] + 12),
			MEM_GET_UINT32(mRegisters[ESP] + 16));
		*eax = ret & 0xFFFFFFFF;
		*edx = (ret >> 32) & 0xFFFFFFFF;
		
		ip = (unsigned char*)(MEM_GET_UINT32(mRegisters[ESP]));
		mRegisters[ESP] += 4;
	}
	
	return false;
}
// Returns the limit of the stack area to enable checking for stack overflows.
uintptr_t Simulator::StackLimit() const {
  // Leave a safety margin of 1024 bytes to prevent overrunning the stack when
  // pushing values.
	return reinterpret_cast<uintptr_t>(mStartStack) + 256 * 4;
	//return 0;
}
uintptr_t Simulator::PushAddress(uintptr_t address) {
  unsigned int new_sp = *esp - sizeof(uintptr_t);
  uintptr_t* stack_slot = reinterpret_cast<uintptr_t*>(new_sp);
  *stack_slot = address;
  *esp = new_sp;
  mStack = reinterpret_cast<unsigned char*>(new_sp);

  //DuongNT: Idont know! Seem it makes crash
  //return new_sp;
   return address;
}
uintptr_t Simulator::PopAddress() {
  int current_sp = *esp;
  uintptr_t* stack_slot = reinterpret_cast<uintptr_t*>(current_sp);
  uintptr_t address = *stack_slot;
  *esp = current_sp + sizeof(uintptr_t);
  mStack = reinterpret_cast<unsigned char*>(*esp);
  return address;
}
void Simulator::checkInlineCache()
{
	
}
Simulator* Simulator::current()
{
	Simulator* sim = reinterpret_cast<Simulator*>(v8::internal::Thread::GetThreadLocal(simulator_key));
	if (sim == NULL) {
		sim = new Simulator();
		v8::internal::Thread::SetThreadLocal(simulator_key, sim);
	}
	return sim;
}

void* Simulator::RedirectExternalReference(void* external_function, bool fp_return) {

#if !defined(USING_SIMULATOR) || !defined(USE_REDIRECTION)
	return external_function;
#else	
  Redirection* redirection = Redirection::Get(external_function, fp_return);
  return redirection->address_of_swi_instruction();
#endif
}
}}

#endif