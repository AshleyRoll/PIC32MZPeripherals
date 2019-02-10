#ifndef PIC32LIB_PERIPHERAL_PORTS
#define PIC32LIB_PERIPHERAL_PORTS

#include <xc.h>
#include <stdint.h>
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
	namespace Ports
	{
		// used to select the port
		// We detect which ports are available using the xc.h provided macros
		namespace Port {
			enum Index : uintptr_t {
#ifdef _PORTA
			PortA = (uintptr_t)_PORTA_BASE_ADDRESS,
#endif
#ifdef _PORTB
			PortB = (uintptr_t)_PORTB_BASE_ADDRESS,
#endif
#ifdef _PORTC
			PortC = (uintptr_t)_PORTC_BASE_ADDRESS,
#endif
#ifdef _PORTD
			PortD = (uintptr_t)_PORTD_BASE_ADDRESS,
#endif
#ifdef _PORTE
			PortE = (uintptr_t)_PORTE_BASE_ADDRESS,
#endif
#ifdef _PORTF
			PortF = (uintptr_t)_PORTF_BASE_ADDRESS,
#endif
#ifdef _PORTG
			PortG = (uintptr_t)_PORTG_BASE_ADDRESS,
#endif
#ifdef _PORTH
			PortH = (uintptr_t)_PORTH_BASE_ADDRESS,
#endif
#ifdef _PORTJ
			PortJ = (uintptr_t)_PORTJ_BASE_ADDRESS,
#endif
#ifdef _PORTK
			PortK = (uintptr_t)_PORTK_BASE_ADDRESS,
#endif
			};
		}
		
		//
		// Pin Index
		// 
		namespace Pin {
			enum Index : uint32_t
			{
//                             33222222222111111111110000000000
//                             10987654321098765432109876543210
			P0  = 0x0001, // 0b00000000000000000000000000000001,
			P1  = 0x0002, // 0b00000000000000000000000000000010,
			P2  = 0x0004, // 0b00000000000000000000000000000100,
			P3  = 0x0008, // 0b00000000000000000000000000001000,
			P4  = 0x0010, // 0b00000000000000000000000000010000,
			P5  = 0x0020, // 0b00000000000000000000000000100000,
			P6  = 0x0040, // 0b00000000000000000000000001000000,
			P7  = 0x0080, // 0b00000000000000000000000010000000,
			P8  = 0x0100, // 0b00000000000000000000000100000000,
			P9  = 0x0200, // 0b00000000000000000000001000000000,
			P10 = 0x0400, // 0b00000000000000000000010000000000,
			P11 = 0x0800, // 0b00000000000000000000100000000000,
			P12 = 0x1000, // 0b00000000000000000001000000000000,
			P13 = 0x2000, // 0b00000000000000000010000000000000,
			P14 = 0x4000, // 0b00000000000000000100000000000000,
			P15 = 0x8000, // 0b00000000000000001000000000000000,
			};
		}
				
		//
		// Peripheral Pin Selection Outputs
		//
		// These are divided into 4 groups, and are set on the Ports::PinInstance
		//
		namespace PPSOutGroup1
		{
			// Define the group 1 output functions
			enum Function : uint8_t {
				None		= 0x00, // 0b0000,
				U3TX		= 0x01, // 0b0001,
				U4RTS		= 0x02, // 0b0010,
				SDO1		= 0x05, // 0b0101,
				SDO2		= 0x06, // 0b0110,
				SDO3		= 0x07, // 0b0111,
#ifdef _SPI5
				SDO5		= 0x09, // 0b1001,
#endif
#ifdef _SPI6
				SS6			= 0x0A, // 0b1010,
#endif
				OC3			= 0x0B, // 0b1011,
				OC6			= 0x0C, // 0b1100,
				REFCLKO4	= 0x0D, // 0b1101,
				C2OUT		= 0x0E, // 0b1110,
#ifdef _CAN1
				C1TX		= 0x0F, // 0b1111,
#endif
			};
		}

		namespace PPSOutGroup2 {
			// Define the group 2 output functions
			enum Function : uint8_t {
				None		= 0x00, // 0b0000,
				U1TX		= 0x01, // 0b0001,
				U2RTS		= 0x02, // 0b0010,
				U5TX		= 0x03, // 0b0011,
				U6RTS		= 0x04, // 0b0100,
				SDO1		= 0x05, // 0b0101,
				SDO2		= 0x06, // 0b0110,
				SDO3		= 0x07, // 0b0111,
				SDO4		= 0x08, // 0b1000,
#ifdef _SPI5
				SDO5		= 0x09,	// 0b1001,
#endif
				OC4			= 0x0B, // 0b1011,
				OC7			= 0x0C, // 0b1100,
				REFCLKO1	= 0x0F, // 0b1111,
			};
		}
		
		namespace PPSOutGroup3 {	
			// Define the group 3 output functions
			enum Function : uint8_t {
				None		= 0x00, // 0b0000,
				U3RTS		= 0x01, // 0b0001,
				U4TX		= 0x02, // 0b0010,
				U6TX		= 0x04, // 0b0100,
				SS1			= 0x05, // 0b0101,
				SS3			= 0x07, // 0b0111,
				SS4			= 0x08, // 0b1000,
#ifdef _SPI5
				SS5			= 0x09, //0b1001,
#endif
#ifdef _SPI6
				SDO6		= 0x0A, // 0b1010,
#endif
				OC5			= 0x0B, // 0b1011,
				OC8			= 0x0C, // 0b1100,
				C1OUT		= 0x0E, // 0b1110,
				REFCLKO3	= 0x0F, // 0b1111,
			};
		}

		namespace PPSOutGroup4 {
			// Define the group 4 output functions
			enum Function : uint8_t {
				None		= 0x00, // 0b0000,
				U1RTS		= 0x01, // 0b0001,
				U2TX		= 0x02, // 0b0010,
				U5RTS		= 0x03, // 0b0011,
				U6TX		= 0x04, // 0b0100,
				SS2			= 0x06, // 0b0110,
				SDO4		= 0x08, // 0b1000,
#ifdef _SPI6
				SDO6		= 0x0A, // 0b1010,	
#endif
				OC2			= 0x0B, // 0b1011,
				OC1			= 0x0C, // 0b1100,
				OC9			= 0x0D, // 0b1101,
#ifdef _CAN2
				C2TX		= 0x0F, // 0b1111,
#endif
			};
		}
		
		//
		// Peripheral Pin Selection Inputs
		//
		// These are divided into 4 groups, and are set on the Ports::PinInstance
		//
		
		namespace PPSInGroup1 {
			// Define the group input functions
			enum Function : uint8_t {
				INT3,
				T2CK,
				T6CK,
				IC3,
				IC7,
				U1RX,
				U2CTS,
				U5RX,
				U6CTS,
				SDI1,
				SDI3,
#ifdef _SPI5
				SDI5,
#endif
#ifdef _SPI6
				SS6,
#endif
				REFCLKI1,
			};
		}
		
			
		namespace PPSInGroup2 {
			// Define the group input functions
			enum Function : uint8_t {
				INT4,
				T5CK,
				T7CK,
				IC4,
				IC8,
				U3RX,
				U4CTS,
				SDI2,
				SDI4,
#ifdef _CAN1
				C1RX,
#endif
				REFCLKI4,
			};
			
		}	
		
		namespace PPSInGroup3 {
			// Define the group input functions
			enum Function : uint8_t {
				INT2,
				T3CK,
				T8CK,
				IC2,
				IC5,
				IC9,
				U1CTS,
				U2RX,
				U5CTS,
				SS1,
				SS3,
				SS4,
#ifdef _SPI5
				SS5,
#endif
#ifdef _CAN2
				C2RX
#endif
			};
			
		}	
		
		namespace PPSInGroup4 {
			// Define the group input functions
			enum Function : uint8_t {
				INT1,
				T4CK,
				T9CK,
				IC1,
				IC6,
				U3CTS,
				U4RX,
				U6RX,
				SS2,
#ifdef _SPI6
				SDI6,
#endif
				OCFA,
				REFCLKI3,
			};
			
		}	
		
		
		
		
		// hide the implementation specifics from the general namespace as it just pollutes it
		namespace Impl 
		{
			// Define the group input pin selection map values
			enum PPSInGroup1PinMap : uint32_t {
				RPD2  = 0x00, // 0b0000,
				RPG8  = 0x01, // 0b0001,
				RPF4  = 0x02, // 0b0010,
				RPD10 = 0x03, // 0b0011,
				RPF1  = 0x04, // 0b0100,
				RPB9  = 0x05, // 0b0101,
				RPB10 = 0x06, // 0b0110,
				RPC14 = 0x07, // 0b0111,
				RPB5  = 0x08, // 0b1000,
#ifdef _PORTC_RC1_MASK
				RPC1  = 0x0A, // 0b1010,
#endif
#ifdef _PORTD_RD14_MASK				
				RPD14 = 0x0B, // 0b1011,
#endif
#ifdef _PORTG_RG1_MASK
				RPG1  = 0x0C, // 0b1100,
#endif
#ifdef _PORTA_RA14_MASK
				RPA14 = 0x0D, // 0b1101,
#endif
#ifdef _PORTD_RD6_MASK
				RPD6  = 0x0E, // 0b1110,
#endif
			};
			
			enum PPSInGroup2PinMap : uint32_t {
				RPD3  = 0x00, // 0b0000,
				RPG7  = 0x01, // 0b0001,
				RPF5  = 0x02, // 0b0010,
				RPD11 = 0x03, // 0b0011,
				RPF0  = 0x04, // 0b0100,
				RPB1  = 0x05, // 0b0101,
				RPE5  = 0x06, // 0b0110,
				RPC13 = 0x07, // 0b0111,
				RPB3  = 0x08, // 0b1000,
#ifdef _PORTC_RC4_MASK
				RPC4  = 0x0A, // 0b1010,
#endif
#ifdef _PORTD_RD15_MASK
				RPD15 = 0x0B, // 0b1011,
#endif
#ifdef _PORTG_RG0_MASK				
				RPG0  = 0x0C, // 0b1100,
#endif
#ifdef _PORTA_RA15_MASK
				RPA15 = 0x0D, // 0b1101,
#endif
#ifdef _PORTD_RD7_MASK
				RPD7  = 0x0E, // 0b1110,
#endif
			};
			enum PPSInGroup3PinMap : uint32_t {
				RPD9  = 0x00, // 0b0000,
				RPG6  = 0x01, // 0b0001,
				RPB8  = 0x02, // 0b0010,
				RPB15 = 0x03, // 0b0011,
				RPD4  = 0x04, // 0b0100,
				RPB0  = 0x05, // 0b0101,
				RPE3  = 0x06, // 0b0110,
				RPB7  = 0x07, // 0b0111,
#ifdef _PORTF_RF12_MASK
				RPF12 = 0x09, // 0b1001,
#endif
#ifdef _PORTD_RD12_MASK				
				RPD12 = 0x0A, // 0b1010,
#endif
#ifdef _PORTF_RF8_MASK
				RPF8  = 0x0B, // 0b1011,
#endif
#ifdef _PORTC_RC3_MASK
				RPC3  = 0x0C, // 0b1100,
#endif
#ifdef _PORTE_RE9_MASK				
				RPE9  = 0x0D, // 0b1101,
#endif
			};			
			enum PPSInGroup4PinMap : uint32_t {
				RPD1  = 0x00, // 0b0000,
				RPG9  = 0x01, // 0b0001,
				RPB14 = 0x02, // 0b0010,
				RPD0  = 0x03, // 0b0011,
				RPB6  = 0x05, // 0b0101,
				RPD5  = 0x06, // 0b0110,
				RPB2  = 0x07, // 0b0111,
				RPF3  = 0x08, // 0b1000,
#ifdef _PORTF_RF13_MASK
				RPF13 = 0x09, // 0b1001,
#endif
				//NoConnect = 0b1010,
#ifdef _PORTF_RF2_MASK			
				RPF2  = 0x0B, // 0b1011,
#endif
#ifdef _PORTC_RC2_MASK
				RPC2  = 0x0C, // 0b1100,
#endif
#ifdef _PORTE_RE8_MASK
				RPE8  = 0x0D, // 0b1101,
#endif
			};
			
			
			//
			// Because we want to "invert" the control for inputs (we have to specify a pin for a function, but 
			// we want to select a function for a pin, we need to make some mappings
			//
			
				
			//
			// Setup the mapping groups to enable selecting the correct register
			//
			struct PPSInPinGroup1Config {
				typedef PPSInGroup1PinMap PinMap;
				typedef PPSInGroup1::Function Function;
				
				template<Function TFunc>
				static void Assign(PinMap map);
			};
			

			
			struct PPSInPinGroup2Config {
				typedef PPSInGroup2PinMap PinMap;
				typedef PPSInGroup2::Function Function;
				
				template<Function TFunc>
				static void Assign(PinMap map);
			};
				
			struct PPSInPinGroup3Config {
				typedef PPSInGroup3PinMap PinMap;
				typedef PPSInGroup3::Function Function;
				
				template<Function TFunc>
				static void Assign(PinMap map);
			};
			
			struct PPSInPinGroup4Config {
				typedef PPSInGroup4PinMap PinMap;
				typedef PPSInGroup4::Function Function;
				
				template<Function TFunc>
				static void Assign(PinMap map);
			};
			
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::INT3>(PPSInPinGroup1Config::PinMap map) { INT3R = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::T2CK>(PPSInPinGroup1Config::PinMap map) { T2CKR = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::T6CK>(PPSInPinGroup1Config::PinMap map) { T6CKR = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::IC3>(PPSInPinGroup1Config::PinMap map) { IC3R = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::IC7>(PPSInPinGroup1Config::PinMap map) { IC7R = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::U1RX>(PPSInPinGroup1Config::PinMap map) { U1RXR = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::U2CTS>(PPSInPinGroup1Config::PinMap map) { U2CTSR = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::U5RX>(PPSInPinGroup1Config::PinMap map) { U5RXR = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::U6CTS>(PPSInPinGroup1Config::PinMap map) { U6CTSR = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::SDI1>(PPSInPinGroup1Config::PinMap map) { SDI1R = map; }
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::SDI3>(PPSInPinGroup1Config::PinMap map) { SDI3R = map; }
		
#ifdef _SPI5
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::SDI5>(PPSInPinGroup1Config::PinMap map) { SDI5R = map; }
#endif
#ifdef _SPI6
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::SS6>(PPSInPinGroup1Config::PinMap map) { SS6R = map; }
#endif			
			template<> inline void PPSInPinGroup1Config::Assign<PPSInPinGroup1Config::Function::REFCLKI1>(PPSInPinGroup1Config::PinMap map) { REFCLKI1R = map; }
			
			
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::INT4>(PPSInPinGroup2Config::PinMap map) { INT4R = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::T5CK>(PPSInPinGroup2Config::PinMap map) { T5CKR = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::T7CK>(PPSInPinGroup2Config::PinMap map) { T7CKR = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::IC4>(PPSInPinGroup2Config::PinMap map) { IC4R = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::IC8>(PPSInPinGroup2Config::PinMap map) { IC8R = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::U3RX>(PPSInPinGroup2Config::PinMap map) { U3RXR = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::U4CTS>(PPSInPinGroup2Config::PinMap map) { U4CTSR = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::SDI2>(PPSInPinGroup2Config::PinMap map) { SDI2R = map; }
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::SDI4>(PPSInPinGroup2Config::PinMap map) { SDI4R = map; }				
#ifdef _CAN1
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::C1RX>(PPSInPinGroup2Config::PinMap map) { C1RXR = map; }
#endif
			template<> inline void PPSInPinGroup2Config::Assign<PPSInPinGroup2Config::Function::REFCLKI4>(PPSInPinGroup2Config::PinMap map) { REFCLKI4R = map; }
					
	
			
			
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::INT2>(PPSInPinGroup3Config::PinMap map) { INT2R = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::T3CK>(PPSInPinGroup3Config::PinMap map) { T3CKR = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::T8CK>(PPSInPinGroup3Config::PinMap map) { T8CKR = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::IC2>(PPSInPinGroup3Config::PinMap map) { IC2R = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::IC5>(PPSInPinGroup3Config::PinMap map) { IC5R = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::IC9>(PPSInPinGroup3Config::PinMap map) { IC9R = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::U1CTS>(PPSInPinGroup3Config::PinMap map) { U1CTSR = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::U2RX>(PPSInPinGroup3Config::PinMap map) { U2RXR = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::U5CTS>(PPSInPinGroup3Config::PinMap map) { U5CTSR = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::SS1>(PPSInPinGroup3Config::PinMap map) { SS1R = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::SS3>(PPSInPinGroup3Config::PinMap map) { SS3R = map; }
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::SS4>(PPSInPinGroup3Config::PinMap map) { SS4R = map; }

#ifdef _SPI5
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::SS5>(PPSInPinGroup3Config::PinMap map) { SS5R = map; }
#endif
#ifdef _CAN2
			template<> inline void PPSInPinGroup3Config::Assign<PPSInPinGroup3Config::Function::C2RX>(PPSInPinGroup3Config::PinMap map) { C2RXR = map; }
#endif		
			
			
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::INT1>(PPSInPinGroup4Config::PinMap map) { INT1R = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::T4CK>(PPSInPinGroup4Config::PinMap map) { T4CKR = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::T9CK>(PPSInPinGroup4Config::PinMap map) { T9CKR = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::IC1>(PPSInPinGroup4Config::PinMap map) { IC1R = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::IC6>(PPSInPinGroup4Config::PinMap map) { IC6R = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::U3CTS>(PPSInPinGroup4Config::PinMap map) { U3CTSR = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::U4RX>(PPSInPinGroup4Config::PinMap map) { U4RXR = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::U6RX>(PPSInPinGroup4Config::PinMap map) { U6RXR = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::SS2>(PPSInPinGroup4Config::PinMap map) { SS2R = map; }
	
#ifdef _SPI6
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::SDI6>(PPSInPinGroup4Config::PinMap map) { SDI6R = map; }
#endif
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::OCFA>(PPSInPinGroup4Config::PinMap map) { OCFAR = map; }
			template<> inline void PPSInPinGroup4Config::Assign<PPSInPinGroup4Config::Function::REFCLKI3>(PPSInPinGroup4Config::PinMap map) { REFCLKI3R = map; }			

			
			
			//
			// Setup each PIN that is able to be remapped
			//
			template<Port::Index TPort, Pin::Index TPin> 
			struct PPSInputPinMapper {	};
			
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P2> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD2;
			};
			template<> struct PPSInputPinMapper<Port::PortG, Pin::P8> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPG8;
			};
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P4> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF4;
			};
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P10> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD10;
			};
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P1> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF1;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P9> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB9;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P10> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB10;
			};
			template<> struct PPSInputPinMapper<Port::PortC, Pin::P14> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPC14;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P5> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB5;
			};
#ifdef _PORTC_RC1_MASK		
			template<> struct PPSInputPinMapper<Port::PortC, Pin::P1> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPC1;
			};
#endif	
#ifdef _PORTD_RD14_MASK				
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P14> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD14;
			};
#endif
#ifdef _PORTG_RG1_MASK
			template<> struct PPSInputPinMapper<Port::PortG, Pin::P1> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPG1;
			};
#endif
#ifdef _PORTA_RA14_MASK
			template<> struct PPSInputPinMapper<Port::PortA, Pin::P14> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPA14;
			};
#endif
#ifdef _PORTD_RD6_MASK
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P6> 
			{
				typedef PPSInPinGroup1Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD6;
			};
#endif			
			
			
			
			
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P3> 
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD3;
			};
			template<> struct PPSInputPinMapper<Port::PortG, Pin::P7> 
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPG7;
			};
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P5> 
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF5;
			};
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P11>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD11;
			};
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P0>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF0;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P1>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB1;
			};
			template<> struct PPSInputPinMapper<Port::PortE, Pin::P5>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPE5;
			};			
			template<> struct PPSInputPinMapper<Port::PortC, Pin::P13>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPC13;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P3>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB3;
			};
#ifdef _PORTC_RC4_MASK
			template<> struct PPSInputPinMapper<Port::PortC, Pin::P4>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPC4;
			};
#endif
#ifdef _PORTD_RD15_MASK
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P15>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD15;
			};
#endif
#ifdef _PORTG_RG0_MASK				
			template<> struct PPSInputPinMapper<Port::PortG, Pin::P0>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPG0;
			};
#endif
#ifdef _PORTA_RA15_MASK
			template<> struct PPSInputPinMapper<Port::PortA, Pin::P15>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPA15;
			};
#endif
#ifdef _PORTD_RD7_MASK
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P7>
			{
				typedef PPSInPinGroup2Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD7;
			};
#endif


			template<> struct PPSInputPinMapper<Port::PortD, Pin::P9>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD9;
			};
			template<> struct PPSInputPinMapper<Port::PortG, Pin::P6>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPG6;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P8>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB8;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P15>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB15;
			};
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P4>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD4;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P0>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB0;
			};
			template<> struct PPSInputPinMapper<Port::PortE, Pin::P3>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPE3;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P7>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB7;
			};	
#ifdef _PORTF_RF12_MASK
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P12>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF12;
			};
#endif
#ifdef _PORTD_RD12_MASK				
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P12>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD12;
			};
#endif
#ifdef _PORTF_RF8_MASK
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P8>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF8;
			};
#endif
#ifdef _PORTC_RC3_MASK
			template<> struct PPSInputPinMapper<Port::PortC, Pin::P3>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPC3;
			};
#endif
#ifdef _PORTE_RE9_MASK				
			template<> struct PPSInputPinMapper<Port::PortE, Pin::P9>
			{
				typedef PPSInPinGroup3Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPE9;
			};
#endif			
			
			
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P1>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD1;
			};
			template<> struct PPSInputPinMapper<Port::PortG, Pin::P9>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPG9;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P14>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB14;
			};
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P0>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD0;
			};
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P6>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB6;
			};
			template<> struct PPSInputPinMapper<Port::PortD, Pin::P5>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPD5;
			};			
			template<> struct PPSInputPinMapper<Port::PortB, Pin::P2>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPB2;
			};		
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P3>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF3;
			};				
#ifdef _PORTF_RF13_MASK
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P13>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF13;
			};
#endif
#ifdef _PORTF_RF2_MASK		
			template<> struct PPSInputPinMapper<Port::PortF, Pin::P2>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPF2;
			};			
#endif
#ifdef _PORTC_RC2_MASK
			template<> struct PPSInputPinMapper<Port::PortC, Pin::P2>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPC2;
			};	
#endif
#ifdef _PORTE_RE8_MASK
			template<> struct PPSInputPinMapper<Port::PortE, Pin::P8>
			{
				typedef PPSInPinGroup4Config Conf;
				static constexpr Conf::PinMap PinSelect = Conf::PinMap::RPE8;
			};	
#endif			

			
			// finally use the mappers above to implement input selection
			template<Port::Index TPort, Pin::Index TPin>  
			struct PPSInputSelector
			{
				typedef PPSInputPinMapper<TPort, TPin> Mapper;
				typedef typename Mapper::Conf::Function Function;
				
				template<typename Mapper::Conf::Function TFunction>
				static void Assign() 
				{
					Mapper::Conf:: template Assign<TFunction>(Mapper::PinSelect);
				}
			};
			
			
			
			//
			// Template to allow the assignment of a pin to a specific peripheral output
			//
			// NOTE: only the specialized ones are valid, the "generic" one will not have any methods
			// 
			
			template<Port::Index TPort, Pin::Index TPin>
			struct PPSOutputFunctionGroupMapper
			{				
			};
						
			// Group 1
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P2> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPD2R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortG, Pin::P8> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPG8R = function; }
			};			
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P4> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPF4R = function; }
			};			
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P10> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPD10R = function; }
			};		
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P1> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPF1R = function; }
			};			
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P9> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPB9R = function; }
			};			
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P10> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPB10R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortC, Pin::P14> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPC14R = function; }
			};		
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P5> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPB5R = function; }
			};	
			
		
#ifdef _PORTC_RC1_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortC, Pin::P1> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPC1R = function; }
			};
#endif
#ifdef _PORTD_RD14_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P14> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPD14R = function; }
			};
#endif
#ifdef _PORTG_RG1_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortG, Pin::P1> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPG1R = function; }
			};
#endif
#ifdef _PORTA_RA14_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortA, Pin::P14> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPA14R = function; }
			};
#endif
#ifdef _PORTD_RD6_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P6> { 
				typedef PPSOutGroup1::Function Function; 
				static void Assign(Function function) { RPD6R = function; }
			};
#endif
			
			
		
			// Group 2
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P3> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPD3R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortG, Pin::P7> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPG7R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P5> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPF5R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P11> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPD11R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P0> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPF0R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P1> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPB1R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortE, Pin::P5> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPE5R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortC, Pin::P13> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPC13R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P3> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPB3R = function; }
			};
			
#ifdef _PORTC_RC4_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortC, Pin::P4> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPC4R = function; }
			};
#endif
#ifdef _PORTD_RD15_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P15> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPD15R = function; }
			};
#endif
#ifdef _PORTG_RG0_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortG, Pin::P0> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPG0R = function; }
			};
#endif
#ifdef _PORTA_RA15_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortA, Pin::P15> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPA15R = function; }
			};
#endif
#ifdef _PORTD_RD7_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P7> { 
				typedef PPSOutGroup2::Function Function; 
				static void Assign(Function function) { RPD7R = function; }
			};
#endif	
			
			
					


			// Group 3
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P9> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPD9R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortG, Pin::P6> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPG6R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P8> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPB8R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P15> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPB15R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P4> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPD4R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P0> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPB0R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortE, Pin::P3> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPE3R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P7> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPB7R = function; }
			};			
			
#ifdef _PORTF_RF12_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P12> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPF12R = function; }
			};
#endif
#ifdef _PORTD_RD12_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P12> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPD12R = function; }
			};
#endif
#ifdef _PORTF_RF8_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P8> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPF8R = function; }
			};
#endif
#ifdef _PORTC_RC3_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortC, Pin::P3> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPC3R = function; }
			};
#endif
#ifdef _PORTE_RE9_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortE, Pin::P9> { 
				typedef PPSOutGroup3::Function Function; 
				static void Assign(Function function) { RPE9R = function; }
			};
#endif	

			
			// Group 4
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P1> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPD1R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortG, Pin::P9> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPG9R = function; }
			};			
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P14> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPB14R = function; }
			};				
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P0> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPD0R = function; }
			};			
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P6> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPB6R = function; }
			};		
			template<> struct PPSOutputFunctionGroupMapper<Port::PortD, Pin::P5> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPD5R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortB, Pin::P2> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPB2R = function; }
			};
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P3> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPF3R = function; }
			};
			
#ifdef _PORTF_RF13_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P13> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPF13R = function; }
			};
#endif
#ifdef _PORTC_RC2_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortC, Pin::P2> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPC2R = function; }
			};
#endif
#ifdef _PORTE_RE8_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortE, Pin::P8> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPE8R = function; }
			};
#endif
#ifdef _PORTF_RF2_MASK
			template<> struct PPSOutputFunctionGroupMapper<Port::PortF, Pin::P2> { 
				typedef PPSOutGroup4::Function Function; 
				static void Assign(Function function) { RPF2R = function; }
			};
#endif
	
			template<Port::Index TPort, Pin::Index TPin>
			struct PPSOutputSelector 
			{
				typedef PPSOutputFunctionGroupMapper<TPort, TPin> Mapper;
				typedef typename Mapper::Function Function;
				
				template<typename Mapper::Function TFunction>
				static void Assign() 
				{
					Mapper::Assign(TFunction);
				}
			};

		}
		
	
		//
		// The PortInstance template allows us access to each of the
		// registers for a port. 
		//
		// The static register references will be optimized away the by
		// the compiler, leaving code that is pretty close to, if not identical 
		// to directly accessing the associated registers.
		//
		template<Port::Index TPort>
		class PortInstance
		{
		public:
			static AtomicBitManipulationRegister & ANSEL;
			static AtomicBitManipulationRegister & TRIS;
			static AtomicBitManipulationRegister & PORT;
			static AtomicBitManipulationRegister & LAT;
			static AtomicBitManipulationRegister & ODC;
			static AtomicBitManipulationRegister & CNPU;
			static AtomicBitManipulationRegister & CNPD;
			
		};
		
		// we compute the address of the registers by looking for an offset (using PORTB base and associated register)
		// and adding in the TIndex which is the base address for the specific peripheral instance.
		// Port B is available on all devices.
		
		template<Port::Index TPort> AtomicBitManipulationRegister & PortInstance<TPort>::ANSEL
			= *(AtomicBitManipulationRegister*)((uintptr_t)TPort + ((uintptr_t)&ANSELB - (uintptr_t)_PORTB_BASE_ADDRESS));

		template<Port::Index TPort> AtomicBitManipulationRegister & PortInstance<TPort>::TRIS
			= *(AtomicBitManipulationRegister*)((uintptr_t)TPort + ((uintptr_t)&TRISB - (uintptr_t)_PORTB_BASE_ADDRESS));

		template<Port::Index TPort> AtomicBitManipulationRegister & PortInstance<TPort>::PORT
			= *(AtomicBitManipulationRegister*)((uintptr_t)TPort + ((uintptr_t)&PORTB - (uintptr_t)_PORTB_BASE_ADDRESS));

		template<Port::Index TPort> AtomicBitManipulationRegister & PortInstance<TPort>::LAT
			= *(AtomicBitManipulationRegister*)((uintptr_t)TPort + ((uintptr_t)&LATB - (uintptr_t)_PORTB_BASE_ADDRESS));	
		
		template<Port::Index TPort> AtomicBitManipulationRegister & PortInstance<TPort>::ODC
			= *(AtomicBitManipulationRegister*)((uintptr_t)TPort + ((uintptr_t)&ODCB - (uintptr_t)_PORTB_BASE_ADDRESS));

		template<Port::Index TPort> AtomicBitManipulationRegister & PortInstance<TPort>::CNPU
			= *(AtomicBitManipulationRegister*)((uintptr_t)TPort + ((uintptr_t)&CNPUB - (uintptr_t)_PORTB_BASE_ADDRESS));
		
		template<Port::Index TPort> AtomicBitManipulationRegister & PortInstance<TPort>::CNPD
			= *(AtomicBitManipulationRegister*)((uintptr_t)TPort + ((uintptr_t)&CNPDB - (uintptr_t)_PORTB_BASE_ADDRESS));

		
		//
		// The PinInstance template allows us to specify (by typedef) specific pins and write code
		// to manipulate these pins without direct dependency on the specific allocated hardware pin.
		//
		// The PinInstance is the generic implementation that can be used to do anything, it is probably
		// more likely you are looking for one of the more specific Input or Output types below
		// as they simplify the logic and code that needs to be written
		//
		// This relies on the PortInstance to access the correct registers. This will be optimized away
		// at compile time leaving code similar to direct register access.
		//
		template<Port::Index TPort, Pin::Index TPin>
		class PinInstance
		{
		public:
			enum _Constants : uint32_t {
				PinMask = TPin
			};
						
			// PPS support
			typedef Impl::PPSOutputSelector<TPort, TPin> PPSOutSelect;
			typedef Impl::PPSInputSelector<TPort, TPin> PPSInSelect;
			
			static void ConfigureAsInput(bool isDigital)
			{
				DisableWeakPullUp();
				DisableWeakPullDown();

				EnableTristate();
				
				if(isDigital) {
					DisableAnalogue();
				} else {
					EnableAnalogue();
				}
			}
			
			static void ConfigureAsOutput(bool openDrain, bool initialState)
			{
				DisableWeakPullUp();
				DisableWeakPullDown();
				
				// set initial LAT then ODC, then ANSEL and TRIS
				WritePin(initialState);
				if(openDrain) {
					EnableOpenDrain(); 
				} else {
					DisableOpenDrain();
				}
				
				DisableAnalogue();
				DisableTristate();
			}
			
			static uint32_t ReadPin() { return (PortInstance<TPort>::PORT & PinMask); }
			static uint32_t ReadLatch() { return (PortInstance<TPort>::LAT & PinMask); }
			
			static bool IsPinHigh() { return 0 != ReadPin(); }
			static bool IsPinLow() { return 0 == ReadPin(); }
			
			static bool IsLatchHigh() { return 0 != ReadLatch(); }
			static bool IsLatchLow() { return 0 == ReadLatch(); }
			
			static void SetPin() { PortInstance<TPort>::LAT.SetBits(PinMask); }
			static void ClearPin() { PortInstance<TPort>::LAT.ClearBits(PinMask); }
			static void InvertPin() { PortInstance<TPort>::LAT.InvertBits(PinMask); }
			
			static void WritePin(bool state) 
			{
				if(state) {
					SetPin();
				} else {
					ClearPin();
				}
			}
			
			static void EnableAnalogue() { PortInstance<TPort>::ANSEL.SetBits(PinMask); }
			static void DisableAnalogue() { PortInstance<TPort>::ANSEL.ClearBits(PinMask); }
			
			static void EnableTristate() { PortInstance<TPort>::TRIS.SetBits(PinMask); }
			static void DisableTristate() { PortInstance<TPort>::TRIS.ClearBits(PinMask); }
			
			static void EnableOpenDrain() { PortInstance<TPort>::ODC.SetBits(PinMask); }
			static void DisableOpenDrain() { PortInstance<TPort>::ODC.ClearBits(PinMask); }
			
			static void EnableWeakPullUp() { PortInstance<TPort>::CNPU.SetBits(PinMask); }
			static void DisableWeakPullUp() { PortInstance<TPort>::CNPU.ClearBits(PinMask); }
			
			static void EnableWeakPullDown() { PortInstance<TPort>::CNPD.SetBits(PinMask); }
			static void DisableWeakPullDown() { PortInstance<TPort>::CNPD.ClearBits(PinMask); }
			
		};
		
		//
		// Output Pin; Open Drain enabled, and Active Low logic
		//
		template<Port::Index TPort, Pin::Index TPin>
		class OpenDrainOutputActiveLow
		{
		public:
			static void Configure(bool initialiseOn) 
			{
				PinInstance<TPort, TPin>::ConfigureAsOutput(true, !initialiseOn);	// "on" is low
			}
			
			static void On() { PinInstance<TPort, TPin>::ClearPin(); }
			static void Off() { PinInstance<TPort, TPin>::SetPin(); }
			
			static void Invert() { PinInstance<TPort, TPin>::InvertPin(); }
			
			static bool IsOn() { return PinInstance<TPort, TPin>::IsLatchLow(); }
			static bool IsOff() { return PinInstance<TPort, TPin>::IsLatchHigh(); }
		};
		
		//
		// Output Pin; Open Drain enabled, and Active High logic
		//		
		template<Port::Index TPort, Pin::Index TPin>
		class OpenDrainOutputActiveHigh
		{
		public:
			static void Configure(bool initialiseOn) 
			{
				PinInstance<TPort, TPin>::ConfigureAsOutput(true, initialiseOn);	// "on" is high
			}
			
			static void Off() { PinInstance<TPort, TPin>::ClearPin(); }
			static void On() { PinInstance<TPort, TPin>::SetPin(); }
			
			static void Invert() { PinInstance<TPort, TPin>::InvertPin(); }
			
			static bool IsOff() { return PinInstance<TPort, TPin>::IsLatchLow(); }
			static bool IsOn() { return PinInstance<TPort, TPin>::IsLatchHigh(); }
		};
		
		//
		// Output and Input pin; Open drain enabled, suitable for bit bashing I2C etc.
		//
		template<Port::Index TPort, Pin::Index TPin>
		class OpenDrainOutputInput
		{
		public:
			static void Configure() 
			{
				PinInstance<TPort, TPin>::ConfigureAsOutput(true, true);	// float by default
			}
			
			static void Drive() { PinInstance<TPort, TPin>::ClearPin(); }
			static void Float() { PinInstance<TPort, TPin>::SetPin(); }
			
			static void Invert() { PinInstance<TPort, TPin>::InvertPin(); }
			
			static bool IsDriving() { return PinInstance<TPort, TPin>::IsLatchLow(); }
			static bool IsFloating() { return PinInstance<TPort, TPin>::IsLatchHigh(); }
			
			// specific read back of actual pin state so that we can remain in output configuration
			// but read back what another device on the bus is actively driving the pin.
			//
			// Returns True is the pin is high (floating), False if the pin is actively driven
			static bool PinIsFloatingState()
			{
				return 0 != PinInstance<TPort, TPin>::ReadPin();
			}
			
		};
		
		//
		// Output Pin; Active High logic
		//
		template<Port::Index TPort, Pin::Index TPin>
		class OutputActiveHigh
		{
		public:
			static void Configure(bool initialiseOn) 
			{
				PinInstance<TPort, TPin>::ConfigureAsOutput(false, initialiseOn);	// "on" is high
			}
			
			static void Off() { PinInstance<TPort, TPin>::ClearPin(); }
			static void On() { PinInstance<TPort, TPin>::SetPin(); }
			
			static void Invert() { PinInstance<TPort, TPin>::InvertPin(); }
			
			static bool IsOff() { return PinInstance<TPort, TPin>::IsLatchLow(); }
			static bool IsOn() { return PinInstance<TPort, TPin>::IsLatchHigh(); }
		};

		//
		// Output Pin; Active Low logic
		//		
		template<Port::Index TPort, Pin::Index TPin>
		class OutputActiveLow
		{
		public:
			static void Configure(bool initialiseOn) 
			{
				PinInstance<TPort, TPin>::ConfigureAsOutput(false, !initialiseOn);	// "on" is low
			}
			
			static void On() { PinInstance<TPort, TPin>::ClearPin(); }
			static void Off() { PinInstance<TPort, TPin>::SetPin(); }
			
			static void Invert() { PinInstance<TPort, TPin>::InvertPin(); }
			
			static bool IsOn() { return PinInstance<TPort, TPin>::IsLatchLow(); }
			static bool IsOff() { return PinInstance<TPort, TPin>::IsLatchHigh(); }
		};
		
		//
		// Digital Input Pin; Active Low logic
		//
		template<Port::Index TPort, Pin::Index TPin>
		class InputActiveLow
		{
		public:
			static void Configure(bool usePullUp) 
			{
				PinInstance<TPort, TPin>::ConfigureAsInput(true);
				if(usePullUp) {
					PinInstance<TPort, TPin>::EnableWeakPullUp();
				}
			}
			
			// "on" is low
			static bool IsActive() { return PinInstance<TPort, TPin>::IsPinLow(); }
			static bool IsInActive() { return PinInstance<TPort, TPin>::IsPinHigh(); }
		};
		
		//
		// Digital Input Pin; Active High logic
		//		
		template<Port::Index TPort, Pin::Index TPin>
		class InputActiveHigh
		{
		public:
			static void Configure(bool usePullDown) 
			{
				PinInstance<TPort, TPin>::ConfigureAsInput(true);
				if(usePullDown) {
					PinInstance<TPort, TPin>::EnableWeakPullDown();
				}
			}
			
			// "on" is high
			static bool IsInActive() { return PinInstance<TPort, TPin>::IsPinLow(); }
			static bool IsActive() { return PinInstance<TPort, TPin>::IsPinHigh(); }
		};
		
		//
		// Analogue Input Pin; Used for configuring ADC or Comparator inputs
		//
		template<Port::Index TPort, Pin::Index TPin>
		class AnalogueInput
		{
		public:
			static void Configure()
			{
				PinInstance<TPort, TPin>::ConfigureAsInput(false);
			}
		};
		
		//
		// Unused Pin: set the pin into a safe low power state 
		//
		template<Port::Index TPort, Pin::Index TPin>
		class UnusedPin
		{
		public:
			static void Configure()
			{
				// we will make all pins digital inputs (some may not be analog) if needed later we can change this
				// to do analog inputs as they are higher impedance.
				PinInstance<TPort, TPin>::ConfigureAsInput(true);
				// turn on the weak pull down
				PinInstance<TPort, TPin>::EnableWeakPullDown();
			}
		};
		
		//
		// Helper template to generate a Mask from a pin list
		//
		template<Pin::Index TFirst, Pin::Index ... TNext>
		struct PinListMask 
		{
			static constexpr uint32_t Mask = (uint32_t)TFirst | PinListMask<TNext...>::Mask;
		};
		
		template<Pin::Index TFirst>
		struct PinListMask<TFirst>
		{
			static constexpr uint32_t Mask = (uint32_t)TFirst;
		};
		
		
		//
		// This is used to support access to a set of pins on a common port
		// Not complete yet. Ash. 2016-06-04
		//
		template<Port::Index TPort, Pin::Index ... TPins>
		class PortPinGroup
		{
		public:
			enum _Constants : uint32_t {
				PinMask = PinListMask<TPins...>::Mask,
			};
			
			static void ConfigureAsInput(bool isDigital)
			{
				DisableWeakPullUp();
				DisableWeakPullDown();
				
				EnableTristate();
							
				if(isDigital) {
					DisableAnalogue();
				} else {
					EnableAnalogue();
				}
			}
			
			static void ConfigureAsOutput(bool openDrain, uint32_t initialState)
			{
				DisableWeakPullUp();
				DisableWeakPullDown();
				
				// set initial LAT then ODC, then ANSEL and TRIS
				WritePins(initialState);
				if(openDrain) {
					EnableOpenDrain(); 
				} else {
					DisableOpenDrain();
				}
				DisableAnalogue();
				DisableTristate();
			}
			
			static uint32_t ReadPins() { return (PortInstance<TPort>::PORT & PinMask); }
			static uint32_t ReadLatch() { return (PortInstance<TPort>::LAT & PinMask); }
			
			static void WritePins(uint32_t value) 
			{ 
				// this required a read-modify-write cycle to avoid messing up other pins
				// the operation is: LATINV = (LAT ^ Value) & Mask)
				// this flips the bits that are different for our pins
				PortInstance<TPort>::LAT.InvertBits( (PortInstance<TPort>::LAT ^ value) & PinMask );
			}
			
			static void EnableAnalogue() { PortInstance<TPort>::ANSEL.SetBits(PinMask); }
			static void DisableAnalogue() { PortInstance<TPort>::ANSEL.ClearBits(PinMask); }
			
			static void EnableTristate() { PortInstance<TPort>::TRIS.SetBits(PinMask); }
			static void DisableTristate() { PortInstance<TPort>::TRIS.ClearBits(PinMask); }
			
			static void EnableOpenDrain() { PortInstance<TPort>::ODC.SetBits(PinMask); }
			static void DisableOpenDrain() { PortInstance<TPort>::ODC.ClearBits(PinMask); }
			
			static void EnableWeakPullUp() { PortInstance<TPort>::CNPU.SetBits(PinMask); }
			static void DisableWeakPullUp() { PortInstance<TPort>::CNPU.ClearBits(PinMask); }
			
			static void EnableWeakPullDown() { PortInstance<TPort>::CNPD.SetBits(PinMask); }
			static void DisableWeakPullDown() { PortInstance<TPort>::CNPD.ClearBits(PinMask); }
			
		};
		
	}
}

#endif /* PIC32LIB_PERIPHERAL_PORTS */