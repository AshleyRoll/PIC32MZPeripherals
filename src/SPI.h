#ifndef PIC32LIB_PERIPHERAL_SPI
#define PIC32LIB_PERIPHERAL_SPI

#include <xc.h>
#include "Register.h"
#include "Interrupt.h"

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
	// SPI peripheral
	//
	namespace SPI
	{
		// The peripheral index is used to select the specific device.
		// We use the base address defined in the platform / chip header.
		namespace Module {
			enum Index : uintptr_t {
#ifdef _SPI1
				SPI1 = (uintptr_t)_SPI1_BASE_ADDRESS,
#endif
#ifdef _SPI2
				SPI2 = (uintptr_t)_SPI2_BASE_ADDRESS,
#endif
#ifdef _SPI3
				SPI3 = (uintptr_t)_SPI3_BASE_ADDRESS,
#endif
#ifdef _SPI4
				SPI4 = (uintptr_t)_SPI4_BASE_ADDRESS,
#endif
#ifdef _SPI5
				SPI5 = (uintptr_t)_SPI5_BASE_ADDRESS,
#endif 
#ifdef _SPI6
				SPI6 = (uintptr_t)_SPI6_BASE_ADDRESS,
#endif 
			};
		}
		
		//----------------------------------------------------------------
		// Register Bit Masks and Positions
		//----------------------------------------------------------------
		
		namespace SPICONMask {
			enum Mask : uint32_t {
			SRXISEL = _SPI1CON_SRXISEL_MASK,
			STXISEL = _SPI1CON_STXISEL_MASK,
			DISSDI = _SPI1CON_DISSDI_MASK,
			MSTEN = _SPI1CON_MSTEN_MASK,
			CKP = _SPI1CON_CKP_MASK,
			SSEN = _SPI1CON_SSEN_MASK,
			CKE = _SPI1CON_CKE_MASK,
			SMP = _SPI1CON_SMP_MASK,
			MODE16 = _SPI1CON_MODE16_MASK,
			MODE32 = _SPI1CON_MODE32_MASK,
			DISSDO = _SPI1CON_DISSDO_MASK,
			SIDL = _SPI1CON_SIDL_MASK,
			ON = _SPI1CON_ON_MASK,
			ENHBUF = _SPI1CON_ENHBUF_MASK,
			SPIFE = _SPI1CON_SPIFE_MASK,
			MCLKSEL = _SPI1CON_MCLKSEL_MASK,
			FRMCNT = _SPI1CON_FRMCNT_MASK,
			FRMSYPW = _SPI1CON_FRMSYPW_MASK,
			MSSEN = _SPI1CON_MSSEN_MASK,
			FRMPOL = _SPI1CON_FRMPOL_MASK,
			FRMSYNC = _SPI1CON_FRMSYNC_MASK,
			FRMEN = _SPI1CON_FRMEN_MASK,
			};
		}

		namespace SPICONPosition {
			enum Position : uint32_t {
			SRXISEL = _SPI1CON_SRXISEL_POSITION,
			STXISEL = _SPI1CON_STXISEL_POSITION,
			DISSDI = _SPI1CON_DISSDI_POSITION,
			MSTEN = _SPI1CON_MSTEN_POSITION,
			CKP = _SPI1CON_CKP_POSITION,
			SSEN = _SPI1CON_SSEN_POSITION,
			CKE = _SPI1CON_CKE_POSITION,
			SMP = _SPI1CON_SMP_POSITION,
			MODE16 = _SPI1CON_MODE16_POSITION,
			MODE32 = _SPI1CON_MODE32_POSITION,
			DISSDO = _SPI1CON_DISSDO_POSITION,
			SIDL = _SPI1CON_SIDL_POSITION,
			ON = _SPI1CON_ON_POSITION,
			ENHBUF = _SPI1CON_ENHBUF_POSITION,
			SPIFE = _SPI1CON_SPIFE_POSITION,
			MCLKSEL = _SPI1CON_MCLKSEL_POSITION,
			FRMCNT = _SPI1CON_FRMCNT_POSITION,
			FRMSYPW = _SPI1CON_FRMSYPW_POSITION,
			MSSEN = _SPI1CON_MSSEN_POSITION,
			FRMPOL = _SPI1CON_FRMPOL_POSITION,
			FRMSYNC = _SPI1CON_FRMSYNC_POSITION,
			FRMEN = _SPI1CON_FRMEN_POSITION,
			};
		}

		namespace SPISTATMask {
			enum Mask : uint32_t {
			SPIRBF = _SPI1STAT_SPIRBF_MASK,
			SPITBF = _SPI1STAT_SPITBF_MASK,
			SPITBE = _SPI1STAT_SPITBE_MASK,
			SPIRBE = _SPI1STAT_SPIRBE_MASK,
			SPIROV = _SPI1STAT_SPIROV_MASK,
			SRMT = _SPI1STAT_SRMT_MASK,
			SPITUR = _SPI1STAT_SPITUR_MASK,
			SPIBUSY = _SPI1STAT_SPIBUSY_MASK,
			FRMERR = _SPI1STAT_FRMERR_MASK,
			TXBUFELM = _SPI1STAT_TXBUFELM_MASK,
			RXBUFELM = _SPI1STAT_RXBUFELM_MASK,
			};
		}

		namespace SPISTATPosition {
			enum Position : uint32_t {
			SPIRBF = _SPI1STAT_SPIRBF_POSITION,
			SPITBF = _SPI1STAT_SPITBF_POSITION,
			SPITBE = _SPI1STAT_SPITBE_POSITION,
			SPIRBE = _SPI1STAT_SPIRBE_POSITION,
			SPIROV = _SPI1STAT_SPIROV_POSITION,
			SRMT = _SPI1STAT_SRMT_POSITION,
			SPITUR = _SPI1STAT_SPITUR_POSITION,
			SPIBUSY = _SPI1STAT_SPIBUSY_POSITION,
			FRMERR = _SPI1STAT_FRMERR_POSITION,
			TXBUFELM = _SPI1STAT_TXBUFELM_POSITION,
			RXBUFELM = _SPI1STAT_RXBUFELM_POSITION,
			};
		}

		namespace SPICON2Mask {
			enum Mask : uint32_t {
			AUDMOD = _SPI1CON2_AUDMOD_MASK,
			AUDMONO = _SPI1CON2_AUDMONO_MASK,
			AUDEN = _SPI1CON2_AUDEN_MASK,
			IGNTUR = _SPI1CON2_IGNTUR_MASK,
			IGNROV = _SPI1CON2_IGNROV_MASK,
			SPITUREN = _SPI1CON2_SPITUREN_MASK,
			SPIROVEN = _SPI1CON2_SPIROVEN_MASK,
			FRMERREN = _SPI1CON2_FRMERREN_MASK,
			SPISGNEXT = _SPI1CON2_SPISGNEXT_MASK,
			AUDMOD0 = _SPI1CON2_AUDMOD0_MASK,
			AUDMOD1 = _SPI1CON2_AUDMOD1_MASK,
			};
		}

		namespace SPICON2Position {
			enum Position : uint32_t {
			AUDMOD = _SPI1CON2_AUDMOD_POSITION,
			AUDMONO = _SPI1CON2_AUDMONO_POSITION,
			AUDEN = _SPI1CON2_AUDEN_POSITION,
			IGNTUR = _SPI1CON2_IGNTUR_POSITION,
			IGNROV = _SPI1CON2_IGNROV_POSITION,
			SPITUREN = _SPI1CON2_SPITUREN_POSITION,
			SPIROVEN = _SPI1CON2_SPIROVEN_POSITION,
			FRMERREN = _SPI1CON2_FRMERREN_POSITION,
			SPISGNEXT = _SPI1CON2_SPISGNEXT_POSITION,
			AUDMOD0 = _SPI1CON2_AUDMOD0_POSITION,
			AUDMOD1 = _SPI1CON2_AUDMOD1_POSITION,
			};
		}
		
	
		//----------------------------------------------------------------
		// Interrupt Control Bits
		//
		// We build these using template specialization so we can
		// define the specific vectors needed for each.
		//----------------------------------------------------------------
				
		template<Module::Index TIndex>
		struct SPIInterruptManager
		{
			// no implementation by default to prevent incorrect values
		};

#ifdef _SPI1
		// specialization / implementation for SPI1
		template<> struct SPIInterruptManager<Module::Index::SPI1>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				ErrorVector = _SPI1_FAULT_VECTOR,
				ReceiveDoneVector = _SPI1_RX_VECTOR,
				TransmitDoneVector = _SPI1_TX_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)ErrorVector> Error;
			typedef Interrupt::InterruptVector<(uint8_t)ReceiveDoneVector> ReceiveDone;
			typedef Interrupt::InterruptVector<(uint8_t)TransmitDoneVector> TransmitDone;
		};
#endif
#ifdef _SPI2
		// specialization / implementation for SPI2
		template<> struct SPIInterruptManager<Module::Index::SPI2>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				ErrorVector = _SPI2_FAULT_VECTOR,
				ReceiveDoneVector = _SPI2_RX_VECTOR,
				TransmitDoneVector = _SPI2_TX_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)ErrorVector> Error;
			typedef Interrupt::InterruptVector<(uint8_t)ReceiveDoneVector> ReceiveDone;
			typedef Interrupt::InterruptVector<(uint8_t)TransmitDoneVector> TransmitDone;
		};
#endif
#ifdef _SPI3
		// specialization / implementation for SPI3
		template<> struct SPIInterruptManager<Module::Index::SPI3>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				ErrorVector = _SPI3_FAULT_VECTOR,
				ReceiveDoneVector = _SPI3_RX_VECTOR,
				TransmitDoneVector = _SPI3_TX_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)ErrorVector> Error;
			typedef Interrupt::InterruptVector<(uint8_t)ReceiveDoneVector> ReceiveDone;
			typedef Interrupt::InterruptVector<(uint8_t)TransmitDoneVector> TransmitDone;
		};
#endif
#ifdef _SPI4	
		// specialization / implementation for SPI4
		template<> struct SPIInterruptManager<Module::Index::SPI4>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				ErrorVector = _SPI4_FAULT_VECTOR,
				ReceiveDoneVector = _SPI4_RX_VECTOR,
				TransmitDoneVector = _SPI4_TX_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)ErrorVector> Error;
			typedef Interrupt::InterruptVector<(uint8_t)ReceiveDoneVector> ReceiveDone;
			typedef Interrupt::InterruptVector<(uint8_t)TransmitDoneVector> TransmitDone;
		};
#endif
#ifdef _SPI5
		// specialization / implementation for SPI5
		template<> struct SPIInterruptManager<Module::Index::SPI5>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				ErrorVector = _SPI5_FAULT_VECTOR,
				ReceiveDoneVector = _SPI5_RX_VECTOR,
				TransmitDoneVector = _SPI5_TX_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)ErrorVector> Error;
			typedef Interrupt::InterruptVector<(uint8_t)ReceiveDoneVector> ReceiveDone;
			typedef Interrupt::InterruptVector<(uint8_t)TransmitDoneVector> TransmitDone;
		};
#endif
#ifdef _SPI6
		// specialization / implementation for SPI6
		template<> struct SPIInterruptManager<Module::Index::SPI6>
		{
		public:
			// allow extraction of the Interrupt vector by accessing the type information
			enum _IntVectors : uint8_t {
				ErrorVector = _SPI6_FAULT_VECTOR,
				ReceiveDoneVector = _SPI6_RX_VECTOR,
				TransmitDoneVector = _SPI6_TX_VECTOR	
			};
					
			typedef Interrupt::InterruptVector<(uint8_t)ErrorVector> Error;
			typedef Interrupt::InterruptVector<(uint8_t)ReceiveDoneVector> ReceiveDone;
			typedef Interrupt::InterruptVector<(uint8_t)TransmitDoneVector> TransmitDone;
		};
#endif	
				
	
		// The SPIInstance template allows us to specify (by typedef) a specific module to
		// use, but write code without any direct dependency on specific hardware registers or pins.
		//
		// The static register references will be optimized away the by
		// the compiler, leaving code that is pretty close to, if not identical 
		// to directly accessing the associated registers.
		//
		template<Module::Index TIndex>
		class SPIInstance
		{
		public:
			// The control registers for the SPI peripheral
			static AtomicBitManipulationRegister & CON;
			static AtomicBitManipulationRegister & CON2;
			static AtomicBitManipulationRegister & STAT;
			static AtomicBitManipulationRegister & BRG;
			
			// the BUF register has no bit manipulation shadows
			static uint32_t volatile & BUF;
			
			typedef SPIInterruptManager<TIndex> PeripheralInterrupts;
	
			static uint32_t ReadAndDiscardBuffer() { return BUF; }
			
			// reset the peripheral and disable all features / interrupts.
			static void Reset() 
			{
				// disable all interrupts and clear the flags
				PeripheralInterrupts::Error::Disable();
				PeripheralInterrupts::Error::ClearFlag();
				
				PeripheralInterrupts::ReceiveDone::Disable();
				PeripheralInterrupts::ReceiveDone::ClearFlag();
				
				PeripheralInterrupts::TransmitDone::Disable();
				PeripheralInterrupts::TransmitDone::ClearFlag();	
				
				CON = 0;		// Stops and resets SPI.
				CON2 = 0;		// Reset audio settings	
				ReadAndDiscardBuffer();	// this will clear the buffer now that we are not in EHBUF mode
				
				// clear receive / transmit overflow bits
				STAT.ClearBits(SPISTATMask::SPIROV | SPISTATMask::SPITUR);
			}
			
			static void Enable() { CON.SetBits(SPICONMask::ON); }
			static bool IsEnabled() { return 0 != (CON.Value & SPICONMask::ON); }
			
			static bool IsBusy() { return 0 != (STAT.Value & SPISTATMask::SPIBUSY); }
			
			static void SetBaudRate(uint32_t baudRate, uint32_t incomingClockFrequency_Hz)
			{
				BRG = ((incomingClockFrequency_Hz / (2 * baudRate)) - 1);
			}
		};
		
		// we compute the address of the registers by looking for an offset (using SPI1 base and associated register)
		// and adding in the TIndex which is the base address for the specific peripheral instance
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& SPIInstance<TIndex>::CON 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&SPI1CON - (uintptr_t)_SPI1_BASE_ADDRESS));
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& SPIInstance<TIndex>::CON2 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&SPI1CON2 - (uintptr_t)_SPI1_BASE_ADDRESS));
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& SPIInstance<TIndex>::STAT 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&SPI1STAT - (uintptr_t)_SPI1_BASE_ADDRESS));
		
		template<Module::Index TIndex> AtomicBitManipulationRegister& SPIInstance<TIndex>::BRG 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&SPI1BRG - (uintptr_t)_SPI1_BASE_ADDRESS));
		
		template<Module::Index TIndex> uint32_t volatile & SPIInstance<TIndex>::BUF 
			= *(uint32_t volatile*)((uintptr_t)TIndex + ((uintptr_t)&SPI1BUF - (uintptr_t)_SPI1_BASE_ADDRESS));

	}
}

#endif /* PIC32LIB_PERIPHERAL_SPI */