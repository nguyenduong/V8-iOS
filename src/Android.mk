TOP_LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
LOCAL_PATH := $(TOP_LOCAL_PATH)
#TOP_LOCAL_PATH:= $(call my-dir)

#INCLUDE_PATH += ${NDK_PATH}/platforms/android-9/arch-arm/usr/include

LOCAL_MODULE := v8_static

LOCAL_MODULE_FILENAME := libv8

LOCAL_CPP_EXTENSION := .cc

LOCAL_SRC_FILES := \
src/accessors.cc \
src/allocation.cc \
src/api.cc \
src/arm/assembler-arm.cc \
src/assembler.cc \
src/ast.cc \
src/bootstrapper.cc \
src/arm/builtins-arm.cc \
src/builtins.cc \
src/checks.cc \
src/circular-queue.cc \
src/code-stubs.cc \
src/arm/codegen-arm.cc \
src/codegen.cc \
src/compilation-cache.cc \
src/compiler.cc \
src/arm/constants-arm.cc \
src/contexts.cc \
src/conversions.cc \
src/counters.cc \
src/arm/cpu-arm.cc \
src/cpu-profiler.cc \
src/data-flow.cc \
src/dateparser.cc \
src/debug-agent.cc \
src/arm/debug-arm.cc \
src/debug.cc \
src/arm/disasm-arm.cc \
src/disassembler.cc \
src/diy-fp.cc \
src/dtoa-config.c \
src/dtoa.cc \
src/execution.cc \
src/factory.cc \
src/arm/fast-codegen-arm.cc \
src/fast-codegen.cc \
src/fast-dtoa.cc \
src/fixed-dtoa.cc \
src/flags.cc \
src/flow-graph.cc \
src/frame-element.cc \
src/arm/frames-arm.cc \
src/frames.cc \
src/arm/full-codegen-arm.cc \
src/full-codegen.cc \
src/func-name-inferrer.cc \
src/global-handles.cc \
src/handles.cc \
src/hashmap.cc \
src/heap-profiler.cc \
src/heap.cc \
src/arm/ic-arm.cc \
src/ic.cc \
src/interpreter-irregexp.cc \
src/jsregexp.cc \
src/arm/jump-target-arm.cc \
src/jump-target-light.cc \
src/jump-target.cc \
src/liveedit.cc \
src/log-utils.cc \
src/log.cc \
src/arm/macro-assembler-arm.cc \
src/mark-compact.cc \
src/messages.cc \
src/objects-debug.cc \
src/objects.cc \
src/oprofile-agent.cc \
src/parser.cc \
src/platform-linux.cc \
src/platform-posix.cc \
src/prettyprinter.cc \
src/profile-generator.cc \
src/property.cc \
src/arm/regexp-macro-assembler-arm.cc \
src/regexp-macro-assembler-irregexp.cc \
src/regexp-macro-assembler-tracer.cc \
src/regexp-macro-assembler.cc \
src/regexp-stack.cc \
src/arm/register-allocator-arm.cc \
src/register-allocator.cc \
src/rewriter.cc \
src/runtime.cc \
src/scanner.cc \
src/scopeinfo.cc \
src/scopes.cc \
src/serialize.cc \
src/arm/simulator-arm.cc \
src/snapshot-common.cc \
src/snapshot-empty.cc \
src/spaces.cc \
src/string-stream.cc \
src/arm/stub-cache-arm.cc \
src/stub-cache.cc \
src/token.cc \
src/top.cc \
src/type-info.cc \
src/unicode.cc \
src/utils.cc \
src/v8-counters.cc \
src/v8.cc \
src/v8threads.cc \
src/variables.cc \
src/version.cc \
src/arm/virtual-frame-arm.cc \
src/virtual-frame-light.cc \
src/virtual-frame.cc \
src/vm-state.cc \
DerivedSources/natives.cc \
src/zone.cc


LOCAL_CFLAGS    := -DV8_TARGET_ARCH_ARM -DARM 
LOCAL_CFLAGS    += -DENABLE_LOGGING_AND_PROFILING
LOCAL_CFLAGS    += -DENABLE_VMSTATE_TRACKING
LOCAL_CFLAGS    += -DENABLE_DEBUGGER_SUPPORT \
				   -DV8_ENABLE_CHECKS 
				   
LOCAL_CFLAGS    += -DDEBUG
				   
LOCAL_CFLAGS    += -fexceptions \
					-Wno-psabi \
					-O3
APP_OPTIM        := release

#LOCAL_CFLAGS    += -ansi \
#				   -fno-rtti \
#				   -Wno-endif-labels \
#				   -Wno-import \
#				   -Wno-format  \
#				   -Wno-psabi \
#				   -std=gnu++0x
				   
#LOCAL_STATIC_LIBRARIES := liblog				   
				   
ifeq ($(ENABLE_V8_LOGGING_AND_PROFILING),true)
	LOCAL_CFLAGS += -DENABLE_LOGGING_AND_PROFILING
endif
				   
#LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/
#LOCAL_C_INCLUDES += bionic/libc/include
#LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include $(LOCAL_PATH)/src


#LOCAL_C_INCLUDES += D:/applications/android-ndk-r8b/platforms/android-14/arch-arm/usr/include
LOCAL_C_INCLUDES += $(call host-path,$(SYSROOT))/usr/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/src 

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include

#					$(LOCAL_PATH)/include \
#					$(LOCAL_PATH)/src/arm
					
#LOCAL_LDLIBS := -llog -landroid					
#LOCAL_C_INCLUDES += $(LOCAL_PATH)/src

                                 
#LOCAL_LDLIBS := -lz
 								 
include $(BUILD_STATIC_LIBRARY)

