/* Zeta API - Z/inspection/data_model/deduction.h
 ______  ______________  ___
|__   / |  ___|___  ___|/   \
  /  /__|  __|   |  |  /  -  \
 /______|_____|  |__| /__/ \__\
Copyright (C) 2006-2024 Manuel Sainz de Baranda y Goñi.
Released under the terms of the GNU Lesser General Public License v3. */

#ifndef Z_inspection_data_model_deduction_H
#define Z_inspection_data_model_deduction_H

#if Z_COMPILER_HAS_CONSTANT(SHORT_WIDTH)
#	define Z_z_SHORT_WIDTH Z_COMPILER_CONSTANT(SHORT_WIDTH)

#elif Z_COMPILER_HAS_CONSTANT(SHORT_SIZE)
#	define Z_z_SHORT_WIDTH (Z_COMPILER_CONSTANT(SHORT_SIZE) * 8)

#elif Z_COMPILER_HAS_CONSTANT(USHORT_MAXIMUM)
#	if Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 65535
#		define Z_z_SHORT_WIDTH 16
#	elif Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 16777215
#		define Z_z_SHORT_WIDTH 24
#	elif Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 4294967295
#		define Z_z_SHORT_WIDTH 32
#	elif Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 1099511627775
#		define Z_z_SHORT_WIDTH 40
#	elif Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 281474976710655
#		define Z_z_SHORT_WIDTH 48
#	elif Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 72057594037927935
#		define Z_z_SHORT_WIDTH 56
#	elif Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 18446744073709551615
#		define Z_z_SHORT_WIDTH 64
#	elif Z_COMPILER_CONSTANT(USHORT_MAXIMUM) == 340282366920938463463374607431768211455
#		define Z_z_SHORT_WIDTH 128
#	endif

#elif Z_COMPILER_HAS_CONSTANT(SSHORT_MAXIMUM)
#	if Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 32767
#		define Z_z_SHORT_WIDTH 16
#	elif Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 8388607
#		define Z_z_SHORT_WIDTH 24
#	elif Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 2147483647
#		define Z_z_SHORT_WIDTH 32
#	elif Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 549755813887
#		define Z_z_SHORT_WIDTH 40
#	elif Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 140737488355327
#		define Z_z_SHORT_WIDTH 48
#	elif Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 36028797018963967
#		define Z_z_SHORT_WIDTH 56
#	elif Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 9223372036854775807
#		define Z_z_SHORT_WIDTH 64
#	elif Z_COMPILER_CONSTANT(SSHORT_MAXIMUM) == 170141183460469231731687303715884105727
#		define Z_z_SHORT_WIDTH 128
#	endif

#elif Z_COMPILER_HAS_CONSTANT(SSHORT_MINIMUM)
#	if Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -32767 - 1
#		define Z_z_SHORT_WIDTH 16
#	elif Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -8388607 - 1
#		define Z_z_SHORT_WIDTH 24
#	elif Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -2147483647 - 1
#		define Z_z_SHORT_WIDTH 32
#	elif Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -549755813887 - 1
#		define Z_z_SHORT_WIDTH 40
#	elif Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -140737488355327 - 1
#		define Z_z_SHORT_WIDTH 48
#	elif Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -36028797018963967 - 1
#		define Z_z_SHORT_WIDTH 56
#	elif Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -9223372036854775807 - 1
#		define Z_z_SHORT_WIDTH 64
#	elif Z_COMPILER_CONSTANT(SSHORT_MINIMUM) == -170141183460469231731687303715884105727 - 1
#		define Z_z_SHORT_WIDTH 64
#	endif

#endif

#ifndef Z_z_SHORT_WIDTH
#	define Z_z_SHORT_WIDTH 0
#endif

#if Z_COMPILER_HAS_CONSTANT(INT_WIDTH)
#	define Z_z_INT_WIDTH Z_COMPILER_CONSTANT(INT_WIDTH)

#elif Z_COMPILER_HAS_CONSTANT(INT_SIZE)
#	define Z_z_INT_WIDTH (Z_COMPILER_CONSTANT(INT_SIZE) * 8)

#elif Z_COMPILER_HAS_CONSTANT(UINT_MAXIMUM)
#	if Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 65535U
#		define Z_z_INT_WIDTH 16
#	elif Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 16777215U
#		define Z_z_INT_WIDTH 24
#	elif Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 4294967295U
#		define Z_z_INT_WIDTH 32
#	elif Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 1099511627775U
#		define Z_z_INT_WIDTH 40
#	elif Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 281474976710655U
#		define Z_z_INT_WIDTH 48
#	elif Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 72057594037927935U
#		define Z_z_INT_WIDTH 56
#	elif Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 18446744073709551615U
#		define Z_z_INT_WIDTH 64
#	elif Z_COMPILER_CONSTANT(UINT_MAXIMUM) == 340282366920938463463374607431768211455U
#		define Z_z_INT_WIDTH 128
#	endif

#elif Z_COMPILER_HAS_CONSTANT(SINT_MAXIMUM)
#	if Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 32767
#		define Z_z_INT_WIDTH 16
#	elif Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 8388607
#		define Z_z_INT_WIDTH 24
#	elif Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 2147483647
#		define Z_z_INT_WIDTH 32
#	elif Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 549755813887
#		define Z_z_INT_WIDTH 40
#	elif Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 140737488355327
#		define Z_z_INT_WIDTH 48
#	elif Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 36028797018963967
#		define Z_z_INT_WIDTH 56
#	elif Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 9223372036854775807
#		define Z_z_INT_WIDTH 64
#	elif Z_COMPILER_CONSTANT(SINT_MAXIMUM) == 170141183460469231731687303715884105727
#		define Z_z_INT_WIDTH 128
#	endif

#elif Z_COMPILER_HAS_CONSTANT(SINT_MINIMUM)
#	if Z_COMPILER_CONSTANT(SINT_MINIMUM) == -32767 - 1
#		define Z_z_INT_WIDTH 16
#	elif Z_COMPILER_CONSTANT(SINT_MINIMUM) == -8388607 - 1
#		define Z_z_INT_WIDTH 24
#	elif Z_COMPILER_CONSTANT(SINT_MINIMUM) == -2147483647 - 1
#		define Z_z_INT_WIDTH 32
#	elif Z_COMPILER_CONSTANT(SINT_MINIMUM) == -549755813887 - 1
#		define Z_z_INT_WIDTH 40
#	elif Z_COMPILER_CONSTANT(SINT_MINIMUM) == -140737488355327 - 1
#		define Z_z_INT_WIDTH 48
#	elif Z_COMPILER_CONSTANT(SINT_MINIMUM) == -36028797018963967 - 1
#		define Z_z_INT_WIDTH 56
#	elif Z_COMPILER_CONSTANT(SINT_MINIMUM) == -9223372036854775807 - 1
#		define Z_z_INT_WIDTH 64
#	elif Z_COMPILER_CONSTANT(SINT_MINIMUM) == -170141183460469231731687303715884105727 - 1
#		define Z_z_INT_WIDTH 64
#	endif

#endif

#ifndef Z_z_INT_WIDTH
#	define Z_z_INT_WIDTH 0
#endif

#if Z_COMPILER_HAS_CONSTANT(LONG_WIDTH)
#	define Z_z_LONG_WIDTH Z_COMPILER_CONSTANT(LONG_WIDTH)

#elif Z_COMPILER_HAS_CONSTANT(LONG_SIZE)
#	define Z_z_LONG_WIDTH (Z_COMPILER_CONSTANT(LONG_SIZE) * 8)

#elif Z_COMPILER_HAS_CONSTANT(ULONG_MAXIMUM)
#	if Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 65535UL
#		define Z_z_LONG_WIDTH 16
#	elif Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 16777215UL
#		define Z_z_LONG_WIDTH 24
#	elif Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 4294967295UL
#		define Z_z_LONG_WIDTH 32
#	elif Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 1099511627775UL
#		define Z_z_LONG_WIDTH 40
#	elif Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 281474976710655UL
#		define Z_z_LONG_WIDTH 48
#	elif Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 72057594037927935UL
#		define Z_z_LONG_WIDTH 56
#	elif Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 18446744073709551615UL
#		define Z_z_LONG_WIDTH 64
#	elif Z_COMPILER_CONSTANT(ULONG_MAXIMUM) == 340282366920938463463374607431768211455UL
#		define Z_z_LONG_WIDTH 128
#	endif

#elif Z_COMPILER_HAS_CONSTANT(SLONG_MAXIMUM)
#	if Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 32767L
#		define Z_z_LONG_WIDTH 16
#	elif Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 8388607L
#		define Z_z_LONG_WIDTH 24
#	elif Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 2147483647L
#		define Z_z_LONG_WIDTH 32
#	elif Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 549755813887L
#		define Z_z_LONG_WIDTH 40
#	elif Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 140737488355327L
#		define Z_z_LONG_WIDTH 48
#	elif Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 36028797018963967L
#		define Z_z_LONG_WIDTH 56
#	elif Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 9223372036854775807L
#		define Z_z_LONG_WIDTH 64
#	elif Z_COMPILER_CONSTANT(SLONG_MAXIMUM) == 170141183460469231731687303715884105727L
#		define Z_z_LONG_WIDTH 128
#	endif

#elif Z_COMPILER_HAS_CONSTANT(SLONG_MINIMUM)
#	if Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -32767L - 1L
#		define Z_z_LONG_WIDTH 16
#	elif Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -8388607L - 1L
#		define Z_z_LONG_WIDTH 24
#	elif Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -2147483647L - 1L
#		define Z_z_LONG_WIDTH 32
#	elif Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -549755813887L - 1L
#		define Z_z_LONG_WIDTH 40
#	elif Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -140737488355327L - 1L
#		define Z_z_LONG_WIDTH 48
#	elif Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -36028797018963967L - 1L
#		define Z_z_LONG_WIDTH 56
#	elif Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -9223372036854775807L - 1L
#		define Z_z_LONG_WIDTH 64
#	elif Z_COMPILER_CONSTANT(SLONG_MINIMUM) == -170141183460469231731687303715884105727L - 1L
#		define Z_z_LONG_WIDTH 64
#	endif

#endif

#ifndef Z_z_LONG_WIDTH
#	define Z_z_LONG_WIDTH 0
#endif

#if Z_COMPILER_HAS_CONSTANT(LLONG_WIDTH)
#	define Z_z_LLONG_WIDTH Z_COMPILER_CONSTANT(LLONG_WIDTH)

#elif Z_COMPILER_HAS_CONSTANT(LLONG_SIZE)
#	define Z_z_LLONG_WIDTH (Z_COMPILER_CONSTANT(LLONG_SIZE) * 8)

#elif	(Z_COMPILER_HAS_CONSTANT(ULLONG_MAXIMUM) && Z_COMPILER_HAS_CONSTANT(ULONG_MAXIMUM) && \
	 Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == Z_COMPILER_CONSTANT(ULONG_MAXIMUM)) ||	      \
	(Z_COMPILER_HAS_CONSTANT(SLLONG_MAXIMUM) && Z_COMPILER_HAS_CONSTANT(SLONG_MAXIMUM) && \
	 Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == Z_COMPILER_CONSTANT(SLONG_MAXIMUM)) ||	      \
	(Z_COMPILER_HAS_CONSTANT(SLONG_MINIMUM) && Z_COMPILER_HAS_CONSTANT(SLONG_MINIMUM)  && \
	 Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == Z_COMPILER_CONSTANT(SLONG_MINIMUM))

#	define Z_z_LLONG_WIDTH Z_z_LONG_WIDTH

#elif Z_DIALECT_HAS_TYPE(C99, LONG_LONG) || Z_DIALECT_HAS_TYPE(CPP11, LONG_LONG)

#	if Z_COMPILER_HAS_CONSTANT(ULLONG_MAXIMUM)
#		if Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 65535ULL
#			define Z_z_LLONG_WIDTH 16
#		elif Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 16777215ULL
#			define Z_z_LLONG_WIDTH 24
#		elif Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 4294967295ULL
#			define Z_z_LLONG_WIDTH 32
#		elif Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 1099511627775ULL
#			define Z_z_LLONG_WIDTH 40
#		elif Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 281474976710655ULL
#			define Z_z_LLONG_WIDTH 48
#		elif Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 72057594037927935ULL
#			define Z_z_LLONG_WIDTH 56
#		elif Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 18446744073709551615ULL
#			define Z_z_LLONG_WIDTH 64
#		elif Z_COMPILER_CONSTANT(ULLONG_MAXIMUM) == 340282366920938463463374607431768211455ULL
#			define Z_z_LLONG_WIDTH 128
#		endif

#	elif Z_COMPILER_HAS_CONSTANT(SLLONG_MAXIMUM)
#		if Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 32767LL
#			define Z_z_LLONG_WIDTH 16
#		elif Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 8388607LL
#			define Z_z_LLONG_WIDTH 24
#		elif Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 2147483647LL
#			define Z_z_LLONG_WIDTH 32
#		elif Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 549755813887LL
#			define Z_z_LLONG_WIDTH 40
#		elif Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 140737488355327LL
#			define Z_z_LLONG_WIDTH 48
#		elif Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 36028797018963967LL
#			define Z_z_LLONG_WIDTH 56
#		elif Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 9223372036854775807LL
#			define Z_z_LLONG_WIDTH 64
#		elif Z_COMPILER_CONSTANT(SLLONG_MAXIMUM) == 170141183460469231731687303715884105727LL
#			define Z_z_LLONG_WIDTH 128
#		endif

#	elif Z_COMPILER_HAS_CONSTANT(SLLONG_MINIMUM)
#		if Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -32767LL - 1LL
#			define Z_z_LLONG_WIDTH 16
#		elif Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -8388607LL - 1LL
#			define Z_z_LLONG_WIDTH 24
#		elif Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -2147483647LL - 1LL
#			define Z_z_LLONG_WIDTH 32
#		elif Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -549755813887LL - 1LL
#			define Z_z_LLONG_WIDTH 40
#		elif Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -140737488355327LL - 1LL
#			define Z_z_LLONG_WIDTH 48
#		elif Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -36028797018963967LL - 1LL
#			define Z_z_LLONG_WIDTH 56
#		elif Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -9223372036854775807LL - 1LL
#			define Z_z_LLONG_WIDTH 64
#		elif Z_COMPILER_CONSTANT(SLLONG_MINIMUM) == -170141183460469231731687303715884105727LL - 1LL
#			define Z_z_LLONG_WIDTH 64
#		endif

#	endif

#endif

#ifndef Z_z_LLONG_WIDTH
#	define Z_z_LLONG_WIDTH 0
#endif

#if Z_COMPILER_HAS_CONSTANT(POINTER_WIDTH)
#	define Z_z_POINTER_WIDTH Z_COMPILER_CONSTANT(POINTER_WIDTH)
#elif Z_COMPILER_HAS_CONSTANT(POINTER_SIZE)
#	define Z_z_POINTER_WIDTH (Z_COMPILER_CONSTANT(POINTER_SIZE) * 8)
#elif Z_COMPILER_HAS_CONSTANT(SIZE_WIDTH)
#	define Z_z_POINTER_WIDTH Z_COMPILER_CONSTANT(SIZE_WIDTH)
#elif Z_COMPILER_HAS_CONSTANT(SIZE_SIZE)
#	define Z_z_POINTER_WIDTH (Z_COMPILER_CONSTANT(SIZE_SIZE) * 8)
#elif Z_COMPILER_HAS_CONSTANT(UINTPTR_WIDTH)
#	define Z_z_POINTER_WIDTH Z_COMPILER_CONSTANT(UINTPTR_WIDTH)
#elif Z_COMPILER_HAS_CONSTANT(UINTPTR_SIZE)
#	define Z_z_POINTER_WIDTH (Z_COMPILER_CONSTANT(UINTPTR_SIZE) * 8)
#elif Z_COMPILER_HAS_CONSTANT(SINTPTR_WIDTH)
#	define Z_z_POINTER_WIDTH Z_COMPILER_CONSTANT(SINTPTR_WIDTH)
#elif Z_COMPILER_HAS_CONSTANT(SINTPTR_SIZE)
#	define Z_z_POINTER_WIDTH (Z_COMPILER_CONSTANT(SINTPTR_SIZE) * 8)
#else
#	define Z_z_POINTER_WIDTH 0
#endif

#if Z_z_SHORT_WIDTH == 16
#	if Z_z_INT_WIDTH == 16

#		if	Z_z_LONG_WIDTH	  == 32 && \
			Z_z_LLONG_WIDTH	  == 32 && \
			Z_z_POINTER_WIDTH == 32

#			define Z_DATA_MODEL Z_DATA_MODEL_I16LP32

#		elif	Z_z_LONG_WIDTH	  == 32 && \
			Z_z_LLONG_WIDTH	  == 32 && \
			Z_z_POINTER_WIDTH == 16

#			define Z_DATA_MODEL Z_DATA_MODEL_IP16L32

#		elif	Z_z_LONG_WIDTH	  == 32 && \
			Z_z_LLONG_WIDTH	  == 64 && \
			Z_z_POINTER_WIDTH == 32

#			define Z_DATA_MODEL Z_DATA_MODEL_LP32
#		endif

#	elif Z_z_INT_WIDTH == 32

#		if	Z_z_LONG_WIDTH	  == 32 && \
			Z_z_LLONG_WIDTH	  == 64 && \
			Z_z_POINTER_WIDTH == 32

#			define Z_DATA_MODEL Z_DATA_MODEL_ILP32

#		elif	Z_z_LONG_WIDTH	  == 32 && \
			Z_z_LLONG_WIDTH	  == 64 && \
			Z_z_POINTER_WIDTH == 64

#			define Z_DATA_MODEL Z_DATA_MODEL_LLP64

#		elif	Z_z_LONG_WIDTH	  == 64 && \
			Z_z_LLONG_WIDTH	  == 64 && \
			Z_z_POINTER_WIDTH == 64

#			define Z_DATA_MODEL Z_DATA_MODEL_LP64
#		endif

#	elif	Z_z_INT_WIDTH	  == 64 && \
		Z_z_LONG_WIDTH	  == 64 && \
		Z_z_LLONG_WIDTH	  == 64 && \
		Z_z_POINTER_WIDTH == 64

#		define Z_DATA_MODEL Z_DATA_MODEL_ILP64
#	endif

#elif	Z_z_SHORT_WIDTH	  == 64 && \
	Z_z_INT_WIDTH	  == 64 && \
	Z_z_LONG_WIDTH	  == 64 && \
	Z_z_LLONG_WIDTH	  == 64 && \
	Z_z_POINTER_WIDTH == 64

#	define Z_DATA_MODEL Z_DATA_MODEL_SILP64
#endif

#undef Z_z_SHORT_WIDTH
#undef Z_z_INT_WIDTH
#undef Z_z_LONG_WIDTH
#undef Z_z_LLONG_WIDTH
#undef Z_z_POINTER_WIDTH

#ifndef Z_DATA_MODEL
#	include <Z/inspection/ISA.h>
#	include <Z/inspection/platform.h>
#	include <Z/inspection/OS.h>

#	if Z_OS_IS(UNICOS)
#		define Z_DATA_MODEL Z_DATA_MODEL_SILP64

#	elif Z_OS_IS(WINDOWS) && (Z_ISA_IS(X86_64) || Z_ISA_IS(ITANIUM))
#		define Z_DATA_MODEL Z_DATA_MODEL_LLP64

#	elif Z_OS_IS(MAC_OS_X) && (Z_ISA_IS(X86_64) || Z_ISA_IS(POWERPC_64BIT))
#		define Z_DATA_MODEL Z_DATA_MODEL_LP64

#	elif	(Z_OS_IS(LINUX	 ) && Z_ISA_IS(X86_32)) || \
		(Z_OS_IS(WINDOWS ) && Z_ISA_IS(X86_32)) || \
		(Z_OS_IS(MAC_OS_X) && ((Z_ISA_IS(X86_32) || Z_ISA_IS(POWERPC_32BIT))))

#		define Z_DATA_MODEL Z_DATA_MODEL_ILP32
#	endif
#endif

#endif /* Z_inspection_data_model_deduction_H */