// Copyright 2012 the V8 project authors. All rights reserved.
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

#ifndef V8_VM_SIMULATOR_VM_H_
#define V8_VM_SIMULATOR_VM_H_

#ifdef V8_TARGET_ARCH_VM
#include "interpreter-vm.h"
#endif

#include "allocation.h"

namespace v8 {
namespace internal {

#define USING_SIMULATOR

#ifdef V8_TARGET_ARCH_VM

// Since there is no simulator for the vm architecture the only thing we can
// do is to call the entry directly.
#ifdef USING_SIMULATOR
	#define CALL_GENERATED_CODE(entry, p0, p1, p2, p3, p4) ((MaybeObject*)execute_code((void*)entry, (unsigned char*)p0, (void*)p1, (void*)p2, (int)p3, (void***)p4))
	
	//#define CALL_GENERATED_CODE(entry, p0, p1, p2, p3, p4) (entry(p0, p1, p2, p3, p4))
#else
#define CALL_GENERATED_CODE(entry, p0, p1, p2, p3, p4) \
  (entry(p0, p1, p2, p3, p4))
#endif

typedef int (*regexp_matcher)(String*, int, const byte*,
                              const byte*, int*, int, Address, int, Isolate*);

#ifdef USING_SIMULATOR
#define CALL_GENERATED_REGEXP_CODE(entry, p0, p1, p2, p3, p4, p5, p6, p7, p8) ((int)execute_regexp_code((void*)entry, (void*)p0, p1, (void*)p2, (void*)p3, (void*)p4, p5, (void*)p6, p7, (void*)p8))

//#define CALL_GENERATED_REGEXP_CODE(entry, p0, p1, p2, p3, p4, p5, p6, p7, p8) (FUNCTION_CAST<regexp_matcher>(entry)(p0, p1, p2, p3, p4, p5, p6, p7, p8))

#else
// Call the generated regexp code directly. The code at the entry address should
// expect eight int/pointer sized arguments and return an int.
#define CALL_GENERATED_REGEXP_CODE(entry, p0, p1, p2, p3, p4, p5, p6, p7, p8) \
  (FUNCTION_CAST<regexp_matcher>(entry)(p0, p1, p2, p3, p4, p5, p6, p7, p8))
#endif

#define TRY_CATCH_FROM_ADDRESS(try_catch_address) \
  (reinterpret_cast<TryCatch*>(try_catch_address))

// The stack limit beyond which we will throw stack overflow errors in
// generated code. Because generated code on vm uses the C stack, we
// just use the C stack limit.


class CachePage {
 public:
  static const int LINE_VALID = 0;
  static const int LINE_INVALID = 1;

  static const int kPageShift = 12;
  static const int kPageSize = 1 << kPageShift;
  static const int kPageMask = kPageSize - 1;
  static const int kLineShift = 2;  // The cache line is only 4 bytes right now.
  static const int kLineLength = 1 << kLineShift;
  static const int kLineMask = kLineLength - 1;

  CachePage() {
    memset(&validity_map_, LINE_INVALID, sizeof(validity_map_));
  }

  char* ValidityByte(int offset) {
    return &validity_map_[offset >> kLineShift];
  }

  char* CachedData(int offset) {
    return &data_[offset];
  }

 private:
  char data_[kPageSize];   // The cached data.
  static const int kValidityMapSize = kPageSize >> kLineShift;
  char validity_map_[kValidityMapSize];  // One byte per line.
  
};

class Simulator : public Interpreter
{
public:
	Simulator(Isolate* isolate);
	virtual ~Simulator();
	static Simulator* current(v8::internal::Isolate* isolate);
	static void Initialize(Isolate* isolate);
	virtual bool callBuiltinFunction(unsigned int adr);
	 // ICache.
	static void CheckICache(v8::internal::HashMap* i_cache, unsigned char* instr);
	static void FlushOnePage(v8::internal::HashMap* i_cache, intptr_t start,
                           int size);
	static CachePage* GetCachePage(v8::internal::HashMap* i_cache, void* page);
	  // ICache checking.
	static void FlushICache(v8::internal::HashMap* i_cache, void* start,
                          size_t size);
	// Accessor to the internal simulator stack area.
	uintptr_t StackLimit() const;
	// Push an address onto the JS stack.
	uintptr_t PushAddress(uintptr_t address);
	
	// Pop an address from the JS stack.
	uintptr_t PopAddress();
	// EABI variant for double arguments in use.
	bool use_eabi_hardfloat() {
		return false;
	}
protected:
	virtual void checkInlineCache();
private:
	v8::internal::Isolate* mIsolate;
	static void* RedirectExternalReference(void* external_function, v8::internal::ExternalReference::Type type);
	// Icache simulation
    v8::internal::HashMap* i_cache_;
};

#ifdef USING_SIMULATOR
class SimulatorStack : public v8::internal::AllStatic {
 public:
  static inline uintptr_t JsLimitFromCLimit(v8::internal::Isolate* isolate,
                                            uintptr_t c_limit) {
    return Simulator::current(isolate)->StackLimit();
  }

  static inline uintptr_t RegisterCTryCatch(uintptr_t try_catch_address) {
    Simulator* sim = Simulator::current(Isolate::Current());
    return sim->PushAddress(try_catch_address);
  }

  static inline void UnregisterCTryCatch() {
    Simulator::current(Isolate::Current())->PopAddress();
  }
};
#else
class SimulatorStack : public v8::internal::AllStatic {
 public:
  static inline uintptr_t JsLimitFromCLimit(Isolate* isolate,
                                            uintptr_t c_limit) {
    USE(isolate);
    return c_limit;
  }

  static inline uintptr_t RegisterCTryCatch(uintptr_t try_catch_address) {
    return try_catch_address;
  }

  static inline void UnregisterCTryCatch() { }
};
#endif

void* execute_code(void* entry_code, unsigned char* entry, void* function, void* receiver, int argc, void*** args);
int execute_regexp_code(void* matcher_func,
                          void* input,
                          int start_offset,
						  void* input_start,
                          void* input_end,
                          void* output,
						  int	output_size,
                          void* stack_base,						  
                          int   direct_call,
						  void*   isolate);


#endif

} }  // namespace v8::internal

#endif  // V8_VM_SIMULATOR_VM_H_
