#ifndef PIC32LIB_PERIPHERAL_OSCILLATOR
#define PIC32LIB_PERIPHERAL_OSCILLATOR

#include <xc.h>

#include "Register.h"

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
	// Port and Pin / IO related templates
	//
	namespace Osc
	{
		// used to select the port
		// We detect which ports are available using the xc.h provided macros
		namespace PBClock {
			enum Index : uintptr_t {
				PBClock1 = 1,
				PBClock2 = 2,
				PBClock3 = 3,
				PBClock4 = 4,
				PBClock5 = 5,
				PBClock7 = 7,
				PBClock8 = 8,
			};
		}
		
		//
		// Peripheral Clocks
		//
		// NOTE: Changes will not take effect unless a system unlock has been preformed beforehand
		//
		template<PBClock::Index TIndex> 
		class PBClockInstance
		{
		private:
			static AtomicBitManipulationRegister & DIV;
		public:
			
			static bool IsDividerReady() { return 0 != (DIV & _PB1DIV_PBDIVRDY_MASK); }
		
			static bool IsEnabled() { return 0 != (DIV & _PB2DIV_ON_MASK); }
			
			static void Disable() { DIV.ClearBits(_PB2DIV_ON_MASK); }
			
			// set the divisor (between 1 and 128) and enable the clock
			static void Enable(uint8_t divisor) { 
				// Note the divisor of 0 = divide by 1
				DIV = (
					  (((uint32_t)(divisor-1) << _PB2DIV_PBDIV_POSITION) & _PB2DIV_PBDIV_MASK)
					| _PB2DIV_ON_MASK
				);
			}
			
			// return the current divisor between 1 and 128
			static uint8_t GetDivisor() { return ((DIV & _PB2DIV_PBDIV_MASK) >> _PB2DIV_PBDIV_POSITION) + 1; }
			
			// compute the actual frequency of the clock with the given system clock input
			static uint32_t ComputeFrequency(uint32_t systemClock_Hz) { return systemClock_Hz / GetDivisor(); }
			
		};
		
		// we specialise on the divider register only
		template<PBClock::Index TIndex> AtomicBitManipulationRegister& PBClockInstance<TIndex>::DIV
			= *(AtomicBitManipulationRegister*)((uintptr_t)&PB1DIV + (((uintptr_t)TIndex-1) * ((uintptr_t)&PB2DIV - (uintptr_t)&PB1DIV)));
		
		
		namespace RefClock {
			enum Index {
				RefClock1 = 1,
				RefClock2 = 2,
				RefClock3 = 3,
				RefClock4 = 4
			};
		}
		
		namespace RefClockSource {
			enum Source : uint8_t {
				SYSClock = 0,
				PBClock1 = 1,
				PrimaryOsc = 2,
				FastRCOsc = 3,
				LowPowerRCOsc = 4,
				SecondaryOsc = 5,
				SystemPLL = 7,
				RefClockIn = 8,
				BackupFastRCOsc = 9
			};
		}
		
		
		template<RefClock::Index TIndex> 
		class RefClockInstance
		{
		private:
			static AtomicBitManipulationRegister & CON;
			static AtomicBitManipulationRegister & TRIM;
			
		public:

			static void EnableOsc() { CON.SetBits(_REFO1CON_ON_MASK); }
			static void DisableOsc() { CON.ClearBits(_REFO1CON_ON_MASK); }
			
			static void EnableOutput() { CON.SetBits(_REFO1CON_OE_MASK); }
			static void DisableOutput() { CON.ClearBits(_REFO1CON_OE_MASK); }
			
			// Currently only configuring while disabled is supported.
			// this will disable all outputs and the module.
			static void Configure(RefClockSource::Source source, uint16_t divisor, uint16_t trim )
			{
				// disable first
				CON.ClearBits(_REFO1CON_ON_MASK | _REFO1CON_OE_MASK);
				
				// set the trim value
				TRIM = (trim << _REFO1TRIM_ROTRIM_POSITION) & _REFO1TRIM_ROTRIM_MASK;
				
				// build the source and divisor values for CON. We will leave all other bits as 0's (default)
				CON = (
					  ((((uint32_t)divisor - 1) << _REFO1CON_RODIV_POSITION) & _REFO1CON_RODIV_MASK)
					| (((uint32_t)source << _REFO2CON_ROSEL_POSITION) & _REFO2CON_ROSEL_MASK)
				);
			}
			
			static uint16_t GetDivisor() { return ((CON & _REFO1CON_RODIV_MASK) >> _REFO1CON_RODIV_POSITION) + 1; }
			static uint16_t GetTrim() { return (TRIM & _REFO1TRIM_ROTRIM_MASK) >> _REFO1TRIM_ROTRIM_POSITION; }
		};
		
		template<RefClock::Index TIndex> AtomicBitManipulationRegister& RefClockInstance<TIndex>::CON
			= *(AtomicBitManipulationRegister*)((uintptr_t)&REFO1CON + (((uintptr_t)TIndex-1) * ((uintptr_t)&REFO2CON - (uintptr_t)&REFO1CON)));
		
		template<RefClock::Index TIndex> AtomicBitManipulationRegister& RefClockInstance<TIndex>::TRIM
			= *(AtomicBitManipulationRegister*)((uintptr_t)&REFO1TRIM + (((uintptr_t)TIndex-1) * ((uintptr_t)&REFO2TRIM - (uintptr_t)&REFO1TRIM)));

		
	}
}

#endif /* PIC32LIB_PERIPHERAL_OSCILLATOR */