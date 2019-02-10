# PIC32MZ Peripherals

C++ Template driven peripheral library for the PIC32MZ

This is part of a private project, I'm sharing this to allow feedback from the cpplang slack #embedded community

Note that the compiler is limited C++11 features.

I'm no C++ expert and I'm sure I've not avoided all footguns. I'm sure others will have different and better ways to 
achieve this, so I'm keen to get feedback. This is working in a commercial product, but there is always opportunity
to imporove!

# Usage

The basic idea is that we define a type from the templates that specialises based on the the specific instace
of a peripheral. Then in code we refer to this typedef and call static methods on it.

This allows the code to be writen abstracted from the specific register set for the peripheral.

Some common header defines:

``` c++

typedef Peripheral::Ports::OutputActiveHigh<Peripheral::Ports::Port::PortE, Peripheral::Ports::Pin::P4> LED1;

```

Then we can do the following:

``` c++

void init()
{
	LED1::Configure(false);		// setup port direction and init to inactive
}


int main(int, char**) 
{
	init();

	LED1::On();
	// ...
	LED1::Off();

	while(1) {
		LED1::Toggle();
	}
}
```

A more complex example for configuring peripheral pin select

``` c++

typedef Peripheral::Ports::PinInstance<Peripheral::Ports::Port::PortG, Peripheral::Ports::Pin::P7> SPI_SDI;
typedef Peripheral::Ports::PinInstance<Peripheral::Ports::Port::PortG, Peripheral::Ports::Pin::P8> SPI_SDO;
typedef Peripheral::Ports::PinInstance<Peripheral::Ports::Port::PortG, Peripheral::Ports::Pin::P6> SPI_SCLK;


SPI_SCLK::ConfigureAsOutput(false, false);	// make a digital output and low

SPI_SDI::ConfigureAsInput(true);
SPI_SDI::EnableWeakPullDown();	// weak pull downs to prevent floating and phantom powering the device
SPI_SDI::PPSInSelect::Assign<SPI_SDI::PPSInSelect::Function::SDI2>();

SPI_SDO::ConfigureAsOutput(false, false);	// make digital pin
SPI_SDO::PPSOutSelect::Assign<SPI_SDO::PPSOutSelect::Function::SDO2>();

```

# Reasons

I've chosen to keep this low level becuase there are so many different ways to use peripherals
it is difficult (expensive is time and space) to write generic drivers. This is why vendor libraries
are painful in my experience.

I'd rather have _safe_ operation on low level peripherals then build my application specific code 
for each use case I need. 

Of course these are trivial examples, but when it comes to serial peripherals with 6 instances on a chip or 
a DMA controller with 8 channels, we can move around the allocated hardware just by changing a single typedef.

Additionaly, there is no actual instances of classes, just types with static methods so there is essentially
a zero overhead with this code. The compiler simply emits the appropriate register operations even on 
the optimisation crippled free version of the xc32 compiler (with `-O1` flag)

