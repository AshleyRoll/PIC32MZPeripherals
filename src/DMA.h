#ifndef PIC32LIB_PERIPHERAL_DMA
#define PIC32LIB_PERIPHERAL_DMA

#include <xc.h>
#include <sys/kmem.h>

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
	// DMA Channels Peripherals
	//
	namespace DMA 
	{
		// the channel index is used to select the specific channel
		namespace Channel {
			enum Index : uint8_t {
				Channel0 = 0,
				Channel1 = 1,
				Channel2 = 2,
				Channel3 = 3,
				Channel4 = 4,
				Channel5 = 5,
				Channel6 = 6,
				Channel7 = 7
			};
		}
		
		namespace DMAChannelCONMask {
			enum Mask : uint32_t {
			CHPRI = _DCH0CON_CHPRI_MASK,
			CHEDET = _DCH0CON_CHEDET_MASK,
			CHAEN = _DCH0CON_CHAEN_MASK,
			CHCHN = _DCH0CON_CHCHN_MASK,
			CHAED = _DCH0CON_CHAED_MASK,
			CHEN = _DCH0CON_CHEN_MASK,
			CHCHNS = _DCH0CON_CHCHNS_MASK,
			CHPATLEN = _DCH0CON_CHPATLEN_MASK,
			CHPIGNEN = _DCH0CON_CHPIGNEN_MASK,
			CHBUSY = _DCH0CON_CHBUSY_MASK,
			CHPIGN = _DCH0CON_CHPIGN_MASK,
			};
		}

		namespace DMAChannelCONPosition {
			enum Position : uint32_t {
			CHPRI = _DCH0CON_CHPRI_POSITION,
			CHEDET = _DCH0CON_CHEDET_POSITION,
			CHAEN = _DCH0CON_CHAEN_POSITION,
			CHCHN = _DCH0CON_CHCHN_POSITION,
			CHAED = _DCH0CON_CHAED_POSITION,
			CHEN = _DCH0CON_CHEN_POSITION,
			CHCHNS = _DCH0CON_CHCHNS_POSITION,
			CHPATLEN = _DCH0CON_CHPATLEN_POSITION,
			CHPIGNEN = _DCH0CON_CHPIGNEN_POSITION,
			CHBUSY = _DCH0CON_CHBUSY_POSITION,
			CHPIGN = _DCH0CON_CHPIGN_POSITION,
			};
		}

		namespace DMAChannelECONMask {
			enum Mask : uint32_t {
			AIRQEN = _DCH0ECON_AIRQEN_MASK,
			SIRQEN = _DCH0ECON_SIRQEN_MASK,
			PATEN = _DCH0ECON_PATEN_MASK,
			CABORT = _DCH0ECON_CABORT_MASK,
			CFORCE = _DCH0ECON_CFORCE_MASK,
			CHSIRQ = _DCH0ECON_CHSIRQ_MASK,
			CHAIRQ = _DCH0ECON_CHAIRQ_MASK,
			};
		}

		namespace DMAChannelECONPosition {
			enum Position : uint32_t {
			AIRQEN = _DCH0ECON_AIRQEN_POSITION,
			SIRQEN = _DCH0ECON_SIRQEN_POSITION,
			PATEN = _DCH0ECON_PATEN_POSITION,
			CABORT = _DCH0ECON_CABORT_POSITION,
			CFORCE = _DCH0ECON_CFORCE_POSITION,
			CHSIRQ = _DCH0ECON_CHSIRQ_POSITION,
			CHAIRQ = _DCH0ECON_CHAIRQ_POSITION,
			};
		}

		namespace DMAChannelINTMask {
			enum Mask : uint32_t {
			CHERIF = _DCH0INT_CHERIF_MASK,
			CHTAIF = _DCH0INT_CHTAIF_MASK,
			CHCCIF = _DCH0INT_CHCCIF_MASK,
			CHBCIF = _DCH0INT_CHBCIF_MASK,
			CHDHIF = _DCH0INT_CHDHIF_MASK,
			CHDDIF = _DCH0INT_CHDDIF_MASK,
			CHSHIF = _DCH0INT_CHSHIF_MASK,
			CHSDIF = _DCH0INT_CHSDIF_MASK,
			CHERIE = _DCH0INT_CHERIE_MASK,
			CHTAIE = _DCH0INT_CHTAIE_MASK,
			CHCCIE = _DCH0INT_CHCCIE_MASK,
			CHBCIE = _DCH0INT_CHBCIE_MASK,
			CHDHIE = _DCH0INT_CHDHIE_MASK,
			CHDDIE = _DCH0INT_CHDDIE_MASK,
			CHSHIE = _DCH0INT_CHSHIE_MASK,
			CHSDIE = _DCH0INT_CHSDIE_MASK,
			};
		}

		namespace DMAChannelINTPosition {
			enum Position : uint32_t {
			CHERIF = _DCH0INT_CHERIF_POSITION,
			CHTAIF = _DCH0INT_CHTAIF_POSITION,
			CHCCIF = _DCH0INT_CHCCIF_POSITION,
			CHBCIF = _DCH0INT_CHBCIF_POSITION,
			CHDHIF = _DCH0INT_CHDHIF_POSITION,
			CHDDIF = _DCH0INT_CHDDIF_POSITION,
			CHSHIF = _DCH0INT_CHSHIF_POSITION,
			CHSDIF = _DCH0INT_CHSDIF_POSITION,
			CHERIE = _DCH0INT_CHERIE_POSITION,
			CHTAIE = _DCH0INT_CHTAIE_POSITION,
			CHCCIE = _DCH0INT_CHCCIE_POSITION,
			CHBCIE = _DCH0INT_CHBCIE_POSITION,
			CHDHIE = _DCH0INT_CHDHIE_POSITION,
			CHDDIE = _DCH0INT_CHDDIE_POSITION,
			CHSHIE = _DCH0INT_CHSHIE_POSITION,
			CHSDIE = _DCH0INT_CHSDIE_POSITION,
			};
		}
		
	
		typedef __DCH0CONbits_t DMAChannelCONBits;
		typedef __DCH0ECONbits_t DMAChannelECONBits;
		typedef __DCH0INTbits_t DMAChannelINTBits;
		
		//
		// The ChannelInstance template allows us to specify (by typedef) a specific 
		// DMA channel and write code to manipulate it without direct dependency on the 
		// specify hardware registers.
		//
		// The static register references will be optimized away the by
		// the compiler, leaving code that is pretty close to, if not identical 
		// to directly accessing the associated registers.
		//
		template<Channel::Index TIndex>
		class ChannelInstance
		{
		public:
			static AtomicBitManipulationRegister & CON;
			static AtomicBitManipulationRegister & ECON;
			static AtomicBitManipulationRegister & INT;
			static AtomicBitManipulationRegister & SSIZ;
			static AtomicBitManipulationRegister & DSIZ;
			static AtomicBitManipulationRegister & SPTR;
			static AtomicBitManipulationRegister & DPTR;
			static AtomicBitManipulationRegister & CSIZ;
			static AtomicBitManipulationRegister & CPTR;
			static AtomicBitManipulationRegister & DAT;
		
			// use the helper methods for these to ease the the virtual -> physical mapping
			static AtomicBitManipulationRegister & SSA;
			static AtomicBitManipulationRegister & DSA;
			
			// The Peripheral Interrupt for the Channel This is what you bind a ISR against
			typedef Interrupt::InterruptVector<((uint8_t)(_DMA0_VECTOR + (uint8_t)TIndex))> PeripheralInterrupt;
					
			static DMAChannelCONBits GetCONBits() { return CON; }
			static DMAChannelECONBits GetECONBits() { return ECON; }
			static DMAChannelINTBits GetINTBits() { return INT; }
			
			// Set source address helper (will do physical address translation)
		//	static void SetSourceAddress(void volatile * ptr) 
		//	{
		//		SSA.Value = KVA_TO_PA(ptr);
		//	}
			
			static void SetSourceAddress(void const * ptr)
			{
				SSA.Value = KVA_TO_PA(ptr);
			}
			
			// Set destination address helper (will do physical address translation)
			static void SetDestinationAddress(void * ptr) 
			{
				DSA.Value = KVA_TO_PA(ptr);
			}
			
			// reset the peripheral and disable all features / interrupts.
			static void Reset() 
			{
				// disable all interrupts and clear the flags
				PeripheralInterrupt::Disable();
				PeripheralInterrupt::ClearFlag();
								
				CON = 0;			// Stops and resets DMA Channel
				ECON = 0x00FFFF00;	// All flags off, default IRQ channels selected (255)
				DAT = 0;			// Reset audio settings	
				INT = 0;			// all interrupts disabled and flags cleared
				
			}
			
			// channel helpers
			static void Enable() { CON.SetBits(DMAChannelCONMask::CHEN); }
			static bool IsEnabled() { return 0 != (CON.Value & DMAChannelCONMask::CHEN); }
			static void Abort() { ECON.SetBits(DMAChannelECONMask::CABORT); }
			static void ForceStart() { ECON.SetBits(DMAChannelECONMask::CFORCE); }
			
			static void SetChannelPriority(uint8_t priority) 
			{
				CON.ClearBits(DMAChannelCONMask::CHPRI);
				CON.SetBits(((priority << DMAChannelCONPosition::CHPRI) & DMAChannelCONMask::CHPRI));
			}
			
			// configuring the start IRQ source
			static void SelectAndEnableStartIRQ(uint8_t interruptVector)
			{
				ECON.ClearBits(DMAChannelECONMask::SIRQEN | (0xFF << DMAChannelECONPosition::CHSIRQ));
				ECON.SetBits(DMAChannelECONMask::SIRQEN | (interruptVector << DMAChannelECONPosition::CHSIRQ));
			}
			static void SelectStartIRQ(uint8_t interruptVector)
			{
				ECON.ClearBits((0xFF << DMAChannelECONPosition::CHSIRQ));
				ECON.SetBits((interruptVector << DMAChannelECONPosition::CHSIRQ));
			}
			
			static void EnableStartIRQ() { ECON.SetBits(DMAChannelECONMask::SIRQEN); }
			static void DisableStartIRQ() { ECON.ClearBits(DMAChannelECONMask::SIRQEN); }
			
			// configuring the abort IRQ source
			static void SelectAndEnableAbortIRQ(uint8_t interruptVector)
			{
				ECON.ClearBits(DMAChannelECONMask::AIRQEN | (0xFF << DMAChannelECONPosition::CHAIRQ));
				ECON.SetBits(DMAChannelECONMask::AIRQEN | (interruptVector << DMAChannelECONPosition::CHAIRQ));
			}			

			static void SelectAbortIRQ(uint8_t interruptVector)
			{
				ECON.ClearBits((0xFF << DMAChannelECONPosition::CHAIRQ));
				ECON.SetBits((interruptVector << DMAChannelECONPosition::CHAIRQ));
			}			
			
			static void EnableAbortIRQ() { ECON.SetBits(DMAChannelECONMask::AIRQEN); }
			static void DisableAbortIRQ() { ECON.ClearBits(DMAChannelECONMask::AIRQEN); }
			
			// Wrapper/Helper class for the various channel interrupt sources
			// These interrupt sources determine what actually triggers the PeripheralInterrupt
			template<DMAChannelINTMask::Mask TEnableMask, DMAChannelINTMask::Mask TFlagMask>
			struct ChannelInterrupt
			{
				static void Enable() { INT.SetBits(TEnableMask); }
				static void Disable() { INT.ClearBits(TEnableMask); }
				static void ClearFlag() { INT.ClearBits(TFlagMask); }
				static bool IsFlagSet() { return 0 != (INT.Value & TFlagMask); }
			};
			
			// define all the different sources for generating the peripheral interrupt from the channel
			
			typedef ChannelInterrupt<DMAChannelINTMask::CHSDIE,DMAChannelINTMask::CHSDIF> SourceDoneInterrupt;
			typedef ChannelInterrupt<DMAChannelINTMask::CHSHIE,DMAChannelINTMask::CHSHIF> SourceHalfEmptyInterrupt;
			
			typedef ChannelInterrupt<DMAChannelINTMask::CHDDIE,DMAChannelINTMask::CHDDIF> DestinationDoneInterrupt;
			typedef ChannelInterrupt<DMAChannelINTMask::CHDHIE,DMAChannelINTMask::CHDHIF> DestinationHalfFullInterrupt;
			
			typedef ChannelInterrupt<DMAChannelINTMask::CHBCIE,DMAChannelINTMask::CHBCIF> BlockTransferCompleteInterrupt;
			typedef ChannelInterrupt<DMAChannelINTMask::CHCCIE,DMAChannelINTMask::CHCCIF> CellTransferCompleteInterrupt;
			
			typedef ChannelInterrupt<DMAChannelINTMask::CHTAIE,DMAChannelINTMask::CHTAIF> ChannelTransferAbortInterrupt;
			typedef ChannelInterrupt<DMAChannelINTMask::CHERIE,DMAChannelINTMask::CHERIF> ChannelAddressErrorInterrupt;
			
		};
	
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::CON
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0CON  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::ECON 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0ECON  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::INT
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0INT  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::SSIZ 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0SSIZ  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::DSIZ 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0DSIZ  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::SPTR 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0SPTR  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::DPTR 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0DPTR  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::CSIZ 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0CSIZ  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::CPTR 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0CPTR  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::DAT 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0DAT  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::SSA 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0SSA  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		template<Channel::Index TIndex> AtomicBitManipulationRegister& ChannelInstance<TIndex>::DSA 
			= *(AtomicBitManipulationRegister*)((uintptr_t)&DCH0DSA  + (((uintptr_t)&DCH1CON - (uintptr_t)&DCH0CON) * TIndex));
		
		// there is actually only a single DMA controller, but we will stick to the pattern
		namespace Controller {
			enum Index : uintptr_t {
				DMA0 = _DMAC_BASE_ADDRESS,
			};
		}
				
		namespace DMACONMask {
			enum Mask : uint32_t {
			DMABUSY = _DMACON_DMABUSY_MASK,
			SUSPEND = _DMACON_SUSPEND_MASK,
			ON = _DMACON_ON_MASK,
			};
		}

		namespace DMACONPosition {
			enum Position : uint32_t {
			DMABUSY = _DMACON_DMABUSY_POSITION,
			SUSPEND = _DMACON_SUSPEND_POSITION,
			ON = _DMACON_ON_POSITION,
			};
		}

		namespace DMASTATMask {
			enum Mask : uint32_t {
			DMACH = _DMASTAT_DMACH_MASK,
			RDWR = _DMASTAT_RDWR_MASK,
			};
		}

		namespace DMASTATPosition {
			enum Position : uint32_t {
			DMACH = _DMASTAT_DMACH_POSITION,
			RDWR = _DMASTAT_RDWR_POSITION,
			};
		}
		
		namespace DMACRCCONMask {
			enum Mask : uint32_t {
			CRCCH = _DCRCCON_CRCCH_MASK,
			CRCTYP = _DCRCCON_CRCTYP_MASK,
			CRCAPP = _DCRCCON_CRCAPP_MASK,
			CRCEN = _DCRCCON_CRCEN_MASK,
			PLEN = _DCRCCON_PLEN_MASK,
			BITO = _DCRCCON_BITO_MASK,
			WBO = _DCRCCON_WBO_MASK,
			BYTO = _DCRCCON_BYTO_MASK,
			};
		}

		namespace DMACRCCONPosition {
			enum Position : uint32_t {
			CRCCH = _DCRCCON_CRCCH_POSITION,
			CRCTYP = _DCRCCON_CRCTYP_POSITION,
			CRCAPP = _DCRCCON_CRCAPP_POSITION,
			CRCEN = _DCRCCON_CRCEN_POSITION,
			PLEN = _DCRCCON_PLEN_POSITION,
			BITO = _DCRCCON_BITO_POSITION,
			WBO = _DCRCCON_WBO_POSITION,
			BYTO = _DCRCCON_BYTO_POSITION,
			};
		}	
		
		
		//
		// DMA Controller Peripheral
		//
		// This provides access to the DMA Controller level features. 
		// Not complete yet. Ash. 2016-06-04
		//
		template<Controller::Index TIndex>
		class ControllerInstance
		{
		public:
			static AtomicBitManipulationRegister & CON;
			static AtomicBitManipulationRegister & STAT;
			static AtomicBitManipulationRegister & ADDR;
			
			static AtomicBitManipulationRegister & CRCCON;
			static AtomicBitManipulationRegister & CRCDATA;
			static AtomicBitManipulationRegister & CRCXOR;
			
			static void Enable() { CON.SetBits(DMACONMask::ON); }
			static void Disable() { CON.ClearBits(DMACONMask::ON);}
			
			static void SuspendAndWaitUntilIdle() 
			{
				CON.SetBits(DMACONMask::SUSPEND);
				while(CON & DMACONMask::DMABUSY);	// wait for busy to be cleared
			}
		
			static void Resume() 
			{
				CON.ClearBits(DMACONMask::SUSPEND);
			}
		};
		
		template<Controller::Index TIndex> AtomicBitManipulationRegister& ControllerInstance<TIndex>::CON 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&DMACON - (uintptr_t)_DMAC_BASE_ADDRESS));
	
		template<Controller::Index TIndex> AtomicBitManipulationRegister& ControllerInstance<TIndex>::STAT 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&DMASTAT - (uintptr_t)_DMAC_BASE_ADDRESS));
		
		template<Controller::Index TIndex> AtomicBitManipulationRegister& ControllerInstance<TIndex>::ADDR 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&DMAADDR - (uintptr_t)_DMAC_BASE_ADDRESS));
		
		
		template<Controller::Index TIndex> AtomicBitManipulationRegister& ControllerInstance<TIndex>::CRCCON 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&DCRCCON - (uintptr_t)_DMAC_BASE_ADDRESS));
		
		template<Controller::Index TIndex> AtomicBitManipulationRegister& ControllerInstance<TIndex>::CRCDATA 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&DCRCDATA - (uintptr_t)_DMAC_BASE_ADDRESS));
		
		template<Controller::Index TIndex> AtomicBitManipulationRegister& ControllerInstance<TIndex>::CRCXOR 
			= *(AtomicBitManipulationRegister*)((uintptr_t)TIndex + ((uintptr_t)&DCRCXOR - (uintptr_t)_DMAC_BASE_ADDRESS));
		
		
		
	}
}

#endif /* PIC32LIB_PERIPHERAL_DMA */