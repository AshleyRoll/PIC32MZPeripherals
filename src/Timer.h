#ifndef PIC32LIB_PERIPHERAL_TIMER
#define PIC32LIB_PERIPHERAL_TIMER

#include <xc.h>
#include <stdint.h>
#include "Register.h"

#include "Oscillator.h"

//
// Peripheral Library Classes for Timers.
//
// NOTE: this is a minimal implementation at the moment sufficient to 
//       support the I2C Master using Timer2. No other timers are defined here yet.
//
//

namespace Peripheral { namespace Timers	{
		
	// used to select the appropriate timer for our templates
	namespace Timer {
		enum Index : uint8_t {
			// Type A timer
		//	Timer1 = 1,		
			
			// Type B timers
			Timer2 = 2,
		//	Timer2 = 3,
		// etc
		};
	}
	
	namespace TimerTypeBCONMask {
		enum Mask : uint32_t {
			TCS = _T2CON_TCS_MASK,
		//	T32 = _T2CON_T32_MASK,	don't support 32 bit mode yet
			TCKPS = _T2CON_TCKPS_MASK,
			TGATE = _T2CON_TGATE_MASK,
			SIDL = _T2CON_SIDL_MASK,
			ON = _T2CON_ON_MASK
		};
	}
	
	namespace TimerTypeBCONPosition {
		enum Position : uint32_t {
			TCS = _T2CON_TCS_POSITION,
		//	T32 = _T2CON_T32_POSITION, don't support 32 bit mode yet
			TCKPS = _T2CON_TCKPS_POSITION,
			TGATE = _T2CON_TGATE_POSITION,
			SIDL = _T2CON_SIDL_POSITION,
			ON = _T2CON_ON_POSITION
		};
	}
	
	namespace TimerTypeBPrescale {
		// this is a 3 bit value, and 128 is not used.
		enum Value : uint8_t {
			Prescale_1   = 0x00, // 0b000,
			Prescale_2   = 0x01, // 0b001,
			Prescale_4   = 0x02, // 0b010,
			Prescale_8   = 0x03, // 0b011,
			Prescale_16  = 0x04, // 0b100,
			Prescale_32  = 0x05, // 0b101,
			Prescale_64  = 0x06, // 0b110,
			Prescale_256 = 0x07, // 0b111,
		};
	}
	
	
	template<Timer::Index TIndex> 
	class TimerTypeBInstance {
		// The vector offset changes based on index as there are some timers without external vectors interleaved
		static constexpr uint8_t IntVector = (TIndex < 5) ? 
				(_TIMER_1_VECTOR + (((uint8_t)TIndex-1)*5)) : (_TIMER_6_VECTOR + (((uint8_t)TIndex-6)*4));
		
	public:
		static AtomicBitManipulationRegister & CON;
		static AtomicBitManipulationRegister & TMR;
		static AtomicBitManipulationRegister & PR;
		
		// The interrupt associated with this timer
		typedef Interrupt::InterruptVector<IntVector> PeripheralInterrupt;
		
		// The internal clock source for this timer (before prescaler)
		typedef Peripheral::Osc::PBClockInstance<Peripheral::Osc::PBClock::PBClock3> InternalClockSource;
		
		static void Reset()
		{
			PeripheralInterrupt::Disable();
			PeripheralInterrupt::ClearFlag();
			
			CON = 0;
			TMR = 0;
			PR = 0;
		}
		
		static void SetPrescaler(TimerTypeBPrescale::Value prescale)
		{
			CON.ClearBits(TimerTypeBCONMask::TCKPS);
			CON.SetBits((uint32_t)prescale << TimerTypeBCONPosition::TCKPS);
		}
		
		static void SetPreload(uint16_t preload)
		{
			PR = preload;
		}
		
		static void On() { CON.SetBits(TimerTypeBCONMask::ON); }
		static bool IsOn() { return 0 != (CON.Value & TimerTypeBCONMask::ON); }
		
	};
	
	// calculate / define the registers. Note that Timer1 is different (for when we do it later)
	// the TIndex is 1 based, and we are skipping timer 1 (so TIndex-2)
	template<Timer::Index TIndex> AtomicBitManipulationRegister& TimerTypeBInstance<TIndex>::CON
		= *(AtomicBitManipulationRegister*)((uintptr_t)&T2CON  + (((uintptr_t)&T3CON - (uintptr_t)&T2CON) * (TIndex-2)));
		
	template<Timer::Index TIndex> AtomicBitManipulationRegister& TimerTypeBInstance<TIndex>::TMR
		= *(AtomicBitManipulationRegister*)((uintptr_t)&TMR2  + (((uintptr_t)&TMR3 - (uintptr_t)&TMR2) * (TIndex-2)));

	template<Timer::Index TIndex> AtomicBitManipulationRegister& TimerTypeBInstance<TIndex>::PR
		= *(AtomicBitManipulationRegister*)((uintptr_t)&PR2  + (((uintptr_t)&PR3 - (uintptr_t)&PR2) * (TIndex-2)));

}}




#endif /* PIC32LIB_PERIPHERAL_TIMER */