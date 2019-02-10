#ifndef PIC32LIB_PERIPHERAL_INTERRUPT
#define PIC32LIB_PERIPHERAL_INTERRUPT

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
	// Interrupt Support (used by other peripherals)
	//
	namespace Interrupt
	{
		//
		// The global interrupt guard is responsible for
		// disabling global interrupts and re-enabling them 
		// if they were previously enabled on exit.
		//
		// Use this in a block scope to prevent any interrupts in
		// a critical section of code.
		//
		// This is Resource Acquisition Is Initialisation (RAII) pattern
		class GlobalInterruptGuard final
		{
		public:
			explicit inline GlobalInterruptGuard()
			{
				// call the builtin helper to disable the interrupts and return the 
				// previous state. The IE flag is bit 0 in this value.
				m_WasEnabled = (__builtin_disable_interrupts() & 0x01) == 0x01;
			}
			
			inline bool WasEnabled() { return m_WasEnabled; }

			inline ~GlobalInterruptGuard() 
			{
				// going out of scope, return the global interrupts to the previous state
				if(m_WasEnabled) {
					__builtin_enable_interrupts();
				}
			}

			// prevent copy and move operations
			GlobalInterruptGuard(const GlobalInterruptGuard &other) = delete;
			GlobalInterruptGuard(const GlobalInterruptGuard &&other) = delete;
			GlobalInterruptGuard &operator=(const GlobalInterruptGuard &other) = delete;
			GlobalInterruptGuard &operator=(const GlobalInterruptGuard &&other) = delete;

		private:
			bool m_WasEnabled;
		};
		
		
		
		
		
		
		//----------------------------------------------------------------
		// Interrupt Control Bits
		//
		// The Interrupt Vector Number (which is available as an XC32 define)
		// is used to compute the register and bit locations for all interrupt
		// enable and flag bits. These are simply packed in ascending bit order
		// 
		// The priority and sub-priorty registers are also calculated in a similar
		// manner but the packing is different.
		//
		//----------------------------------------------------------------
		
		
		template<uint8_t TInterruptVector>
		class InterruptVector
		{
		private:
			// compile time constants derived from template parameters
			enum _Masks : uint32_t {
				EnableFlagRegOffset = (TInterruptVector / 32),			// 32 bits per IEC / IFS reg
				EnableFlagBitMask = (1 << (TInterruptVector % 32)),		// bit position in register
				
				PriorityRegOffset = (TInterruptVector / 4),				// 4 interrupt priority sets per register
				PriorityBitShift = (2 + ((TInterruptVector % 4) * 8)),	// Each byte contains Priority / SubPriority
				SubPriorityBitShift = ((TInterruptVector % 4) * 8)		// SubPriority  is first 2 bits, priority is next 3 bits
			};
			
		public:
			// allow access to the specific vector number via the type name
			enum _IntVector : uint8_t { InterruptVectorNumber = TInterruptVector };


			static void Enable()
			{
				((AtomicBitManipulationRegister*)&IEC0)[EnableFlagRegOffset].SetBits(EnableFlagBitMask);
			}

			static void Disable()
			{ 
				((AtomicBitManipulationRegister*)&IEC0)[EnableFlagRegOffset].ClearBits(EnableFlagBitMask);
			}
			
			static void IsEnabled()
			{
				return (0 != (((AtomicBitManipulationRegister*)&IEC0)[EnableFlagRegOffset].Value & EnableFlagBitMask));
			}

			static bool IsFlagSet()
			{
				return 0 != (((AtomicBitManipulationRegister*)&IFS0)[EnableFlagRegOffset].Value & EnableFlagBitMask);
			}

			static void ClearFlag()
			{
				((AtomicBitManipulationRegister*)&IFS0)[EnableFlagRegOffset].ClearBits(EnableFlagBitMask);
			}
			
			static void SetFlag()
			{
				((AtomicBitManipulationRegister*)&IFS0)[EnableFlagRegOffset].SetBits(EnableFlagBitMask);
			}
			
			static void SetPriority(uint8_t priority, uint8_t subPriority)
			{
				AtomicBitManipulationRegister &reg = (((AtomicBitManipulationRegister*)&IPC0)[PriorityRegOffset]);
				// priority is 3 bits, sub-priority is 2
				// first, clear the existing bits
				reg.ClearBits( 
					  (0x00000007u << PriorityBitShift) 
					| (0x00000003u << SubPriorityBitShift) 
				);
				// now set the new ones
				reg.SetBits( 
					  (((uint32_t)priority & 0x00000007u) << PriorityBitShift) 
					| (((uint32_t)subPriority & 0x00000003) << SubPriorityBitShift) 
				);
			}
				
		};
	}
}

#endif /* PIC32LIB_PERIPHERAL_INTERRUPT */