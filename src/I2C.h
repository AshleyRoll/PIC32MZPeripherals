#ifndef PIC32LIB_PERIPHERAL_I2C
#define PIC32LIB_PERIPHERAL_I2C

#include <xc.h>
#include "Register.h"
#include "Interrupt.h"
#include "Oscillator.h"


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
	// I2C peripheral
	//
	namespace I2C
	{
		// The peripheral index is used to select the specific device.
		// We use the base address defined in the platform / chip header.
		namespace Module {
			enum Index : uintptr_t {
#ifdef _I2C1
				I2C1 = (uintptr_t)_I2C1_BASE_ADDRESS,
#endif
#ifdef _I2C2
				IC22 = (uintptr_t)_I2C2_BASE_ADDRESS,
#endif
#ifdef _I2C3
				I2C3 = (uintptr_t)_I2C3_BASE_ADDRESS,
#endif
#ifdef _I2C4
				I2C4 = (uintptr_t)_I2C4_BASE_ADDRESS,
#endif
#ifdef _I2C5
				I2C5 = (uintptr_t)_I2C5_BASE_ADDRESS,
#endif
			};
		}
		
		//----------------------------------------------------------------
		// Register Bit Masks and Positions
		//----------------------------------------------------------------
		
		namespace I2CCONMask {
			enum Mask : uint32_t {
			SEN = _I2C1CON_SEN_MASK,
			RSEN = _I2C1CON_RSEN_MASK,
			PEN = _I2C1CON_PEN_MASK,
			RCEN = _I2C1CON_RCEN_MASK,
			ACKEN = _I2C1CON_ACKEN_MASK,
			ACKDT = _I2C1CON_ACKDT_MASK,
			STREN = _I2C1CON_STREN_MASK,
			GCEN = _I2C1CON_GCEN_MASK,
			SMEN = _I2C1CON_SMEN_MASK,
			DISSLW = _I2C1CON_DISSLW_MASK,
			A10M = _I2C1CON_A10M_MASK,
			STRICT = _I2C1CON_STRICT_MASK,
			SCLREL = _I2C1CON_SCLREL_MASK,
			SIDL = _I2C1CON_SIDL_MASK,
			ON = _I2C1CON_ON_MASK,
			DHEN = _I2C1CON_DHEN_MASK,
			AHEN = _I2C1CON_AHEN_MASK,
			SBCDE = _I2C1CON_SBCDE_MASK,
			SDAHT = _I2C1CON_SDAHT_MASK,
			BOEN = _I2C1CON_BOEN_MASK,
			SCIE = _I2C1CON_SCIE_MASK,
			PCIE = _I2C1CON_PCIE_MASK,
			IPMIEN = _I2C1CON_IPMIEN_MASK,
			I2CSIDL = _I2C1CON_I2CSIDL_MASK,
			I2CEN = _I2C1CON_I2CEN_MASK,
			};
		}

		namespace I2CCONPosition {
			enum Position : uint32_t {
			SEN = _I2C1CON_SEN_POSITION,
			RSEN = _I2C1CON_RSEN_POSITION,
			PEN = _I2C1CON_PEN_POSITION,
			RCEN = _I2C1CON_RCEN_POSITION,
			ACKEN = _I2C1CON_ACKEN_POSITION,
			ACKDT = _I2C1CON_ACKDT_POSITION,
			STREN = _I2C1CON_STREN_POSITION,
			GCEN = _I2C1CON_GCEN_POSITION,
			SMEN = _I2C1CON_SMEN_POSITION,
			DISSLW = _I2C1CON_DISSLW_POSITION,
			A10M = _I2C1CON_A10M_POSITION,
			STRICT = _I2C1CON_STRICT_POSITION,
			SCLREL = _I2C1CON_SCLREL_POSITION,
			SIDL = _I2C1CON_SIDL_POSITION,
			ON = _I2C1CON_ON_POSITION,
			DHEN = _I2C1CON_DHEN_POSITION,
			AHEN = _I2C1CON_AHEN_POSITION,
			SBCDE = _I2C1CON_SBCDE_POSITION,
			SDAHT = _I2C1CON_SDAHT_POSITION,
			BOEN = _I2C1CON_BOEN_POSITION,
			SCIE = _I2C1CON_SCIE_POSITION,
			PCIE = _I2C1CON_PCIE_POSITION,
			IPMIEN = _I2C1CON_IPMIEN_POSITION,
			I2CSIDL = _I2C1CON_I2CSIDL_POSITION,
			I2CEN = _I2C1CON_I2CEN_POSITION,
			};
		}

		namespace I2CSTATMask {
			enum Mask : uint32_t {
			TBF = _I2C1STAT_TBF_MASK,
			RBF = _I2C1STAT_RBF_MASK,
			R_W = _I2C1STAT_R_W_MASK,
			S = _I2C1STAT_S_MASK,
			P = _I2C1STAT_P_MASK,
			D_A = _I2C1STAT_D_A_MASK,
			I2COV = _I2C1STAT_I2COV_MASK,
			IWCOL = _I2C1STAT_IWCOL_MASK,
			ADD10 = _I2C1STAT_ADD10_MASK,
			GCSTAT = _I2C1STAT_GCSTAT_MASK,
			BCL = _I2C1STAT_BCL_MASK,
			ACKTIM = _I2C1STAT_ACKTIM_MASK,
			TRSTAT = _I2C1STAT_TRSTAT_MASK,
			ACKSTAT = _I2C1STAT_ACKSTAT_MASK,
			};
		}

		namespace I2CSTATPosition {
			enum Position : uint32_t {
			TBF = _I2C1STAT_TBF_POSITION,
			RBF = _I2C1STAT_RBF_POSITION,
			R_W = _I2C1STAT_R_W_POSITION,
			S = _I2C1STAT_S_POSITION,
			P = _I2C1STAT_P_POSITION,
			D_A = _I2C1STAT_D_A_POSITION,
			I2COV = _I2C1STAT_I2COV_POSITION,
			IWCOL = _I2C1STAT_IWCOL_POSITION,
			ADD10 = _I2C1STAT_ADD10_POSITION,
			GCSTAT = _I2C1STAT_GCSTAT_POSITION,
			BCL = _I2C1STAT_BCL_POSITION,
			ACKTIM = _I2C1STAT_ACKTIM_POSITION,
			TRSTAT = _I2C1STAT_TRSTAT_POSITION,
			ACKSTAT = _I2C1STAT_ACKSTAT_POSITION,
			};
		}
		
	
		//----------------------------------------------------------------
		// Interrupt Control Bits
		//
		// We build these using template specialization so we can
		// define the specific vectors needed for each.
		//----------------------------------------------------------------
				
		template<Module::Index TIndex>
		struct I2CInterruptManager
		{
			// no implementation by default to prevent incorrect values
		};

#ifdef _I2C1
		// specialization / implementation for I2C1
		template<> struct I2CInterruptManager<Module::Index::I2C1>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				BusCollisionVector = _I2C1_BUS_VECTOR,
				SlaveEventVector = _I2C1_SLAVE_VECTOR,
				MasterEventVector = _I2C1_MASTER_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)BusCollisionVector> BusCollision;
			typedef Interrupt::InterruptVector<(uint8_t)SlaveEventVector> SlaveEvent;
			typedef Interrupt::InterruptVector<(uint8_t)MasterEventVector> MasterEvent;
		};
#endif
#ifdef _I2C2
		// specialization / implementation for I2C2
		template<> struct I2CInterruptManager<Module::Index::I2C2>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				BusCollisionVector = _I2C2_BUS_VECTOR,
				SlaveEventVector = _I2C2_SLAVE_VECTOR,
				MasterEventVector = _I2C2_MASTER_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)BusCollisionVector> BusCollision;
			typedef Interrupt::InterruptVector<(uint8_t)SlaveEventVector> SlaveEvent;
			typedef Interrupt::InterruptVector<(uint8_t)MasterEventVector> MasterEvent;
		};
#endif
#ifdef _I2C3
		// specialization / implementation for I2C3
		template<> struct I2CInterruptManager<Module::Index::I2C3>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				BusCollisionVector = _I2C3_BUS_VECTOR,
				SlaveEventVector = _I2C3_SLAVE_VECTOR,
				MasterEventVector = _I2C3_MASTER_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)BusCollisionVector> BusCollision;
			typedef Interrupt::InterruptVector<(uint8_t)SlaveEventVector> SlaveEvent;
			typedef Interrupt::InterruptVector<(uint8_t)MasterEventVector> MasterEvent;
		};
#endif
#ifdef _I2C4	
		// specialization / implementation for I2C4
		template<> struct I2CInterruptManager<Module::Index::I2C4>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				BusCollisionVector = _I2C4_BUS_VECTOR,
				SlaveEventVector = _I2C4_SLAVE_VECTOR,
				MasterEventVector = _I2C4_MASTER_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)BusCollisionVector> BusCollision;
			typedef Interrupt::InterruptVector<(uint8_t)SlaveEventVector> SlaveEvent;
			typedef Interrupt::InterruptVector<(uint8_t)MasterEventVector> MasterEvent;
		};
#endif
#ifdef _I2C5
		// specialization / implementation for I2C5
		template<> struct I2CInterruptManager<Module::Index::I2C5>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				BusCollisionVector = _I2C5_BUS_VECTOR,
				SlaveEventVector = _I2C5_SLAVE_VECTOR,
				MasterEventVector = _I2C5_MASTER_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)BusCollisionVector> BusCollision;
			typedef Interrupt::InterruptVector<(uint8_t)SlaveEventVector> SlaveEvent;
			typedef Interrupt::InterruptVector<(uint8_t)MasterEventVector> MasterEvent;
		};
#endif

				
		// Bit structure access alias types
		typedef __I2C1CONbits_t I2CCONBits;
		typedef __I2C1STATbits_t I2CSTATBits;

		
		//
		// The I2CInstance template allows us to specify (by typedef) a specific module to
		// use, but write code without any direct dependency on specific hardware registers or pins.
		//
		// The static register references will be optimized away the by
		// the compiler, leaving code that is pretty close to, if not identical 
		// to directly accessing the associated registers.
		//
		// NOTE: we only support simple transactions, either Write or Read, but not 
		// one after the other using a Repeated Start. Basically we don't need too.
		// 
		// NOTE: only 7-bit addresses are supported
		// 
		template<Module::Index TIndex>
		class I2CInstance
		{
		public:
			// The control registers for the SPI peripheral
			static AtomicBitManipulationRegister & CON;
			static AtomicBitManipulationRegister & STAT;
			static AtomicBitManipulationRegister & ADD;
			static AtomicBitManipulationRegister & MSK;
			static AtomicBitManipulationRegister & BRG;
			static AtomicBitManipulationRegister & TRN;
			
			// the RCV register has no bit manipulation shadows
			static uint32_t volatile & RCV;
			
							
			typedef I2CInterruptManager<TIndex> PeripheralInterrupts;
								
			static I2CCONBits GetCONBits() { return CON; }
			static I2CSTATBits GetSTATBits() { return STAT; }
			
			// reset the peripheral and disable all features / interrupts.
			static void Reset() 
			{
				// disable all interrupts and clear the flags
				PeripheralInterrupts::BusCollision::Disable();
				PeripheralInterrupts::SlaveEvent::Disable();		
				PeripheralInterrupts::MasterEvent::Disable();
				
				CON = 0x00001000;		// Stops and resets I2C, SCLREL = 1 to release clock line.
				
				PeripheralInterrupts::BusCollision::ClearFlag();
				PeripheralInterrupts::SlaveEvent::ClearFlag();
				PeripheralInterrupts::MasterEvent::ClearFlag();	
				
				// clear receive overflow bit
				STAT.ClearBits(I2CSTATMask::BCL | I2CSTATMask::IWCOL );
			}
				
			
			// operational helpers
			static void Enable() { CON.SetBits(I2CCONMask::ON); }
			
			static void AssertStartCondition() { CON.SetBits(I2CCONMask::SEN); }
			static void AssertStopCondition() { CON.SetBits(I2CCONMask::PEN); }
			
			static void AssertNack() { CON.SetBits(I2CCONMask::ACKDT | I2CCONMask::ACKEN ); }
			static void AssertAck() { CON.ClearBits(I2CCONMask::ACKDT); CON.SetBits(I2CCONMask::ACKEN ); }
			
			static bool SlaveAckTransmit() { return 0 == (STAT & I2CSTATMask::ACKSTAT); }
			static bool SlaveNackTransmit() { return 0 != (STAT & I2CSTATMask::ACKSTAT); }
			
			static void MasterReceiveOneByte() { CON.SetBits(I2CCONMask::RCEN); }
			
			static void SetBaudRate(uint32_t baudRate, uint32_t incomingClockFrequency_Hz)
			{
				BRG = ((incomingClockFrequency_Hz / (2 * baudRate)) - (incomingClockFrequency_Hz / 10000000) - 2);
			}
			
		};
		
		// we compute the address of the registers by looking for an offset (using I2C1 base and associated register)
		// and adding in the TIndex which is the base address for the specific peripheral instance
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& I2CInstance<TIndex>::CON 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&I2C1CON - (uintptr_t)_I2C1_BASE_ADDRESS));
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& I2CInstance<TIndex>::STAT 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&I2C1STAT - (uintptr_t)_I2C1_BASE_ADDRESS));
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& I2CInstance<TIndex>::ADD 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&I2C1ADD - (uintptr_t)_I2C1_BASE_ADDRESS));
	
		template<Module::Index TIndex> AtomicBitManipulationRegister& I2CInstance<TIndex>::MSK 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&I2C1MSK - (uintptr_t)_I2C1_BASE_ADDRESS));

		template<Module::Index TIndex> AtomicBitManipulationRegister& I2CInstance<TIndex>::BRG 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&I2C1BRG - (uintptr_t)_I2C1_BASE_ADDRESS));
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& I2CInstance<TIndex>::TRN 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&I2C1TRN - (uintptr_t)_I2C1_BASE_ADDRESS));
		
		template<Module::Index TIndex> uint32_t volatile& I2CInstance<TIndex>::RCV 
			= *(uint32_t volatile*)((uintptr_t)TIndex + ((uintptr_t)&I2C1RCV - (uintptr_t)_I2C1_BASE_ADDRESS));
	}
}

#endif /* PIC32LIB_PERIPHERAL_I2C */