#ifndef PIC32LIB_PERIPHERAL_REGISTER
#define PIC32LIB_PERIPHERAL_REGISTER

#include <xc.h>

//
// Peripheral Library classes for Abstracting access to specific peripheral instances
//
// The goal is to be able to move code between instances of a peripheral on a chip without 
// having to change references to specific registers.
//
// We will rely on the chip specific defines from Microchip so we are relatively portable.
// The only problem will be adding and removing devices based on the specific chip.
// We also rely on the regular offsets for all registers in a module and the common 
// bit masks for those registers.
//
namespace Peripheral 
{
	//
	// Most Peripheral registers have a main value, and 3 shadow registers that
	// allow atomic bit manipulation using a CLR / SET / INV variants.
	// 
	// These are all consecutive in memory so we will wrap them into a single struct
	// and present this as the interface for many of the peripheral instance templates.
	// 
	struct AtomicBitManipulationRegister 
	{
	public:
		uint32_t volatile Value;
	private:
		uint32_t volatile MaskedClear;
		uint32_t volatile MaskedSet;
		uint32_t volatile MaskedInvert;
		
	public:
		// default assignment to avoid the need for .Value everywhere
		inline void operator=(uint32_t value) { Value = value; }
		// default conversion to uint32_t to read the value
		inline operator uint32_t() const { return Value; }
		
		inline void ClearBits(uint32_t mask) { MaskedClear = mask; }
		inline void SetBits(uint32_t mask) { MaskedSet = mask; }
		inline void InvertBits(uint32_t mask) { MaskedInvert = mask; }
	};

	static_assert(sizeof(AtomicBitManipulationRegister) == sizeof(uint32_t) * 4, "AtomicBitManipulationRegister incorrect size");
	static_assert(alignof(AtomicBitManipulationRegister) == alignof(uint32_t), "AtomicBitManipulationRegister alignment bad");
}

#endif /* PIC32LIB_PERIPHERAL_REGISTER */