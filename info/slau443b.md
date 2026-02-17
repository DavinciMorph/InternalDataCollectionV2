# EEG Front-End Performance Demonstration Kit

This user's guide describes the characteristics, operation, and use of the ADS1299EEG-FE. This EVM is an evaluation module for the ADS1299, an eight-channel, 24-bit, low-power; integrated analog front-end (AFE) designed for electroencephalography (EEG) applications. The ADS1299ECG-FE is intended for prototyping and evaluation. This user's guide includes a complete circuit description, schematic diagram, and bill of materials.

The following related documents are available through the Texas Instruments web site at www.ti.com.

|  Device | Literature Number  |
| --- | --- |
|  ADS1299 | SBAS499  |

# Contents

1 ADS1299EEG-FE Overview 4

1.1 Important Disclaimer Information 4
1.2 Information About Cautions and Warnings 4

2 Overview 5

2.1 Introduction 5
2.2 Supported Features 5
2.3 Features Not Supported in Current Version 5
2.4 ADS1299EEG-FE Hardware 5
2.5 Factory Default Jumper Settings 6

3 Software Installation 7

3.1 Minimum Requirements 7
3.2 Installing the Software 7
3.3 Install the ADS1299 EVM Hardware Drivers 10

4 ADS1299EEG-FE Daughter Card Hardware Introduction 13

4.1 Power Supply 14
4.2 Clock 15
4.3 Reference 16
4.4 Accessing ADS1299 Analog Signals 16
4.5 Accessing ADS1299 Digital Signals 16
4.6 Analog Inputs 17

5 Using the Software: ADS1299 Control Registers and GUI 18

5.1 Overview and Features 18
5.2 Global Channel Registers 19
5.3 Channel Control Registers 19
5.4 GPIO and Other Registers 23
5.5 Lead-Off and BIAS Registers 23
5.6 Register Map 26

6 ADS1299EEG-FE Analysis Tools 27

6.1 Scope Tab 27
6.2 Histogram Tool 28
6.3 FFT Tool 29

7 EEG Specific Features 32

7.1 Reference Signal and Patient Bias Signal 32

Intel, Pentium, Celeron are registered trademarks of Intel Corporation.

SPI is a trademark of Motorola.

7.2 Lead-Off Detection ... 36
7.3 External Calibration/Test Signals ... 40
8 Test Options on the EVM ... 43
8.1 On-Chip (ADS1299) Input Short ... 43
8.2 External Input Short with 5K Resistor ... 44
8.3 Noise with Common Reference on Negative Inputs ... 46
8.4 Noise with Buffered Common Reference Input ... 48
8.5 Internally Generated Test Signal and Other Multiplexer Inputs ... 49
8.6 Arbitrary Input Signal ... 49
9 Bill of Materials, Layouts and Schematics ... 50
9.1 ADS1299EEG-FE Front-End Board Schematics ... 50
9.2 Printed Circuit Board Layout ... 54
9.3 Bill of Materials ... 57
9.4 ADS1299EEG-FE Power-Supply Recommendations ... 59

# List of Figures

1 ADS1299EEG-FE Kit ... 6
2 Executable to Run ADS1299 Software Installation ... 7
3 Initialization of ADS1299EEG-FE ... 8
4 License Agreement ... 8
5 Installation Process ... 9
6 USBStyx Driver Preinstallation ... 9
7 Completion of ADS1299 Software Installation ... 10
8 New Hardware Wizard ... 10
9 New Hardware Wizard Screen 3 ... 11
10 Completion of the Initial USB Drive ... 11
11 Second 'New Hardware' Wizard ... 12
12 Install the USBStyx Driver ... 12
13 ADS1299 EEG-FE Front End Block Diagram ... 14
14 Input Configurations Supported by the EEG-FE a) Differential Inputs b) Single-Ended Inputs ... 17
15 File Save Option Under 'Save' Tab ... 18
16 Channel Registers GUI for Global Registers ... 19
17 Input Multiplexer for a Single Channel (MAIN = [000 or 110 or 111]) ... 20
18 Channel Control Registers GUI Panel ... 20
19 Register Bit for SRB1 Routing ... 20
20 Internal Test Signals ... 21
21 Simplified Diode Arrangement ... 22
22 Eight Channel Read of Internal Temperature Data ... 22
23 GPIO Control Register GUI Panel ... 23
24 LOFF_STATP and LOFF_STATN Comparators ... 24
25 LOFF_SENSP and LOFF_SENSN Registers GUI Panel ... 24
26 Lead-Off Status Indicator ... 25
27 BIAS_SENSP and BIAS_SENSN GUI Panel ... 26
28 Device Register Settings ... 26
29 Scope Tool Features ... 27
30 Zoom Option on the Waveform Examination Tool ... 28
31 Histogram Bins for Input Short Noise ... 29
32 Analysis : FFT Graph of Input Short Test ... 30
33 Analysis : FFT : AC Analysis Parameters : Windowing Options ... 31
34 Analysis : FFT : FFT Analysis : Input Short Condition ... 31
35 Changing the User-Defined Dynamic Range for Channel 1 ... 32

36 Dedicated Reference and Bias Electrode 33
37 Programmable Reference and Bias Electrode 34
38 Settings for Normal Electrode 35
39 Configuring BIASREF and Bias Drive Buffer 35
40 Setting up the Bias Drive Loop 36
41 Setting the LOFF Register Bits 37
42 Configuring the Lead Off Comparator 37
43 Lead off Status Registers 37
44 Scope tab for Impedance Measurement at 31.25 Hz 38
45 FFT Analysis for Impedance Measurement at 31.25 Hz 39
46 Scope Tab for Impedance Measurement at fDR/4 (DR = 4ksps) 40
47 Multiplexer Setting for Calibration with Electrode Disconnected 41
48 Multiplexer Setting with Positive Electrode Connected to Test Signal 42
49 Multiplexer Setting with Both Electrodes Connected to Test Signal 43
50 Channel Settings for Input Short Test 44
51 Scope Tab for Input Short Test 44
52 Channel Settings for External Input Short Test 45
53 Scope Showing Noise for Input Short with 5k Resistors 46
54 MISC1 Register Setting for SRB1 47
55 Noise with Negative Input Connected to SRB1 Pin 47
56 Noise with OPA376 in SRB1 Path 48
57 Scope Tab with Sinusoidal Inputs on AIN1 49
58 ADS1299EEG-FC Schematic 50
59 ADS1299EEG-FC Jumper Schematic 51
60 ECG Power Supplies 52
61 External Reference Drivers (Not Installed) 53
62 ECG MDK Board Interface Adapter 53
63 ADS1299EEG-FE Top Assembly 54
64 ADS1299EEG-FE Top Layer 54
65 ADS1299EEG-FE Internal Layer (1) 55
66 ADS1299EEG-FE Internal Layer (2) 55
67 ADS1299EEG-FE Bottom Layer 56
68 ADS1299EEG-FE Bottom Assembly 56
69 Recommended Power Supply for ADS1299EEG-FE 59

# List of Tables

1 Factory Default Jumper Settings 6
2 Power Supply Test Points 15
3 Analog Supply Configurations 15
4 Digital Supply Configurations 15
5 Clock Jumper Options 15
6 External Reference Jumper Options 16
7 Test Signals 16
8 Serial Interface Pin Out 16
9 Dedicated Reference Drive Options through REF_ELEC 33
10 Bill of Materials 57

# 1 ADS1299EEG-FE Overview

## 1.1 Important Disclaimer Information

**CAUTION**

The ADS1299EEG-FE is intended for feasibility and evaluation testing only in laboratory and development environments. This product is not for diagnostic use.

The ADS1299EEG-FE is to be used only under these conditions:

- The ADS1299EEG-FE is intended only for **electrical** evaluation of the features of the ADS1299 device in a laboratory, simulation, or development environment.
- The ADS1299EEG-FE is not intended for direct interface with a patient, patient diagnostics, or with a defibrillator.
- The ADS1299EEG-FE is intended for development purposes **ONLY**. It is not intended to be used as all or part of an end-equipment application.
- The ADS1299EEG-FE should be used only by qualified engineers and technicians who are familiar with the risks associated with handling electrical and mechanical components, systems, and subsystems.
- You are responsible for the safety of yourself, your fellow employees and contractors, and your coworkers when using or handling the ADS1299EEG-FE. Furthermore, you are fully responsible for the contact interface between the human body and electronics; consequently, you are responsible for preventing electrical hazards such as shock, electrostatic discharge, and electrical overstress of electric circuit components.

## 1.2 Information About Cautions and Warnings

This document contains caution statements. The information in a caution statement is provided for your protection. Be sure to read each caution carefully.

**CAUTION**

This is an example of a caution statement. A caution statement describes a situation that could potentially damage your software or equipment.

# 2 Overview

## 2.1 Introduction

The ADS1299EEG-FE is intended for evaluating the ADS1299 low-power, low noise 24-bit, simultaneously sampling, eight-channel front-end for EEG applications. The digital SPI™ control interface is provided by the MMB0 Modular EVM motherboard (Rev. D or higher) that connects to the ADS1299EEG FE evaluation board (Rev A). The ADS1299EEG-FE (see Figure 1) is NOT a reference design for EEG applications; rather, its purpose is to expedite evaluation and system development. The output of the ADS1299 yields a raw, unfiltered EEG signal.

The MMB0 motherboard allows the ADS1299EEG-FE to be connected to the computer via an available USB port. This manual shows how to use the MMB0 as part of the ADS1299EEG-FE, but does not provide technical details about the MMB0 itself.

Throughout this document, the abbreviation EVM and the term evaluation module are synonymous with the ADS1299EEG-FE.

## 2.2 Supported Features

### Hardware Features:

- Configurable for bipolar or unipolar supply operation
- Configurable for internal and external clock and reference via jumper settings
- Configurable for dc-coupled inputs
- External bias electrode drive
- Option to provide a common reference to all channels negative terminals.
- Option to select any electrode as reference electrode
- Option to choose any electrode as bias electrode
- External shield drive amplifier

### Software Features:

- Analysis tools including a virtual oscilloscope, histogram, FFT.
- Data export for post-processing of raw EEG data

## 2.3 Features Not Supported in Current Version

**NOTE:**

The following features are NOT SUPPORTED by the current version of the evaluation kit.

- Real-time data processing
- AC lead-off detection filters

## 2.4 ADS1299EEG-FE Hardware

Figure 1 shows the hardware included in the ADS1299EEG-FE kit. Contact the factory if any component is missing. Also, it is highly recommended that you check the TI website at http://www.ti.com to verify that you have the latest software.

![img-0.jpeg](./images/img-0.jpeg)
Figure 1. ADS1299EEG-FE Kit

The complete kit includes the following items:
- ADS1299EEG FE printed circuit board (PCB), Rev A
- MMB0 (Modular EVM motherboard, Rev D or higher)

## 2.5 Factory Default Jumper Settings

Table 1. Factory Default Jumper Settings

|  Jumper Name | Settings | Comment  |
| --- | --- | --- |
|  JP1 | Not Installed | Used for programmable bias drive.  |
|  JP2 | 2-3 | Unipolar analog supply (AVDD = 5V)  |
|  JP3 | Not Installed | Related to external reference generation circuitry  |
|  JP4 | 1-2 | 5V supply to board  |
|  JP5 | Not Installed | Option to provide hardware PWDN signal.  |
|  JP6 | 1-2 | BIAS_ELEC to onboard midsupply  |
|  JP7 | 1-2 | Route REF_ELEC to buffer input (buffer output is not used by default)  |
|  JP8 | 1-2 | Route REF_ELEC directly to SRB1  |
|  JP17 | Not Installed | Related to shield drive circuitry  |
|  JP18 | 2-3 | Clock from Oscillator on the EVM  |
|  JP19 | 1-2 | Power for Oscillator on the EVM  |
|  JP20 | 1-2 | Unipolar supply (AVSS = 0V)  |
|  JP21 | 1-2 |   |
|  JP22 | 2-3 |   |
|  JP23 | 1-2 | CLKSEL = 0  |
|  JP24 | 2-3 | Digital supply (DVDD =3.3)  |

Table 1. Factory Default Jumper Settings (continued)

|  Jumper Name | Settings | Comment  |
| --- | --- | --- |
|  JP25 | 1-2/ 3-4/5-6 | – BIAS_ELEC connected to all INPs
– BIAS_ELEC connected to all INMs
– BIAS_ELEC shorted to REF_ELEC which connects to SRB1  |
|  J6 | 5-6/ 7-8/ 9-10/ ... 32-34 / 35-36 | connect jumpers on all channels  |

# 3 Software Installation

## 3.1 Minimum Requirements

Before installing the software that is intended for use with the EVM kit, verify that your PC meets the minimum requirements outlined in this section.

### 3.1.1 Required Setup for ADS1299EEG-FE Software

Install the software on a PC-compatible computer that meets these specifications:

- Intel® Pentium® III/ Celeron® processor, 866 MHz or equivalent
- Minimum 256MB of RAM (512MB or greater recommended)
- USB 1.1-compatible input
- Hard disk drive with at least 200MB free space
- Microsoft® Windows® XP operating system with SP2 (Windows Vista and Windows 7 are NOT supported at this time)
- Mouse or other pointing device
- 1280 x 960 minimum display resolution

## 3.2 Installing the Software

**CAUTION**

Do not connect the ADS1299EEG-FE hardware before installing the software on a suitable PC. Failure to observe this caution may cause Microsoft Windows to not recognize the ADS1299EEG-FE.

The latest software is available from the TI web site at www.ti.com/ads1299. Check the ADS1299 Product Folder on the TI web site regularly for updated versions.

To install the ADS1299 software, click on the executable shown in Figure 2. Then follow the prompts illustrated in Figure 3 through Figure 7.

|  Name | Size | Type  |
| --- | --- | --- |
|  ads1299-eeg-fe-1.0.0.exe | 159,612 KB | Application  |

Figure 2. Executable to Run ADS1299 Software Installation

![img-1.jpeg](./images/img-1.jpeg)
Figure 3. Initialization of ADS1299EEG-FE

You must accept the license agreement (shown in Figure 4) before you can proceed with the installation.

![img-2.jpeg](./images/img-2.jpeg)
Figure 4. License Agreement

![img-3.jpeg](./images/img-3.jpeg)
Figure 5. Installation Process

![img-4.jpeg](./images/img-4.jpeg)
Figure 6. USBStyx Driver Preinstallation

![img-5.jpeg](./images/img-5.jpeg)
Figure 7. Completion of ADS1299 Software Installation

## 3.3 Install the ADS1299 EVM Hardware Drivers

Apply power to the MMB0 using the supplied wall mount power supply and connect the MMB0 to your PC via any available USB port. There are two USB drivers which will be installed. Follow the steps shown in the figures below to install the USB drivers.

![img-6.jpeg](./images/img-6.jpeg)
Figure 8. New Hardware Wizard

![img-7.jpeg](./images/img-7.jpeg)
Figure 9. New Hardware Wizard Screen 3

Click Next and allow the wizard to find and install the driver.

![img-8.jpeg](./images/img-8.jpeg)
Figure 10. Completion of the Initial USB Drive

# 3.3.1 Initial Launch of the ADS1299EEG FE Software

Launch ADS1299EEG FE software from the program menu. The software will load and begin downloading firmware to the processor on data capture card (MMB0). Once the firmware is loaded and running, it will cause the USB to re-enumerate.

![img-9.jpeg](./images/img-9.jpeg)
Figure 11. Second 'New Hardware' Wizard

Click Next.

![img-10.jpeg](./images/img-10.jpeg)
Figure 12. Install the USBStyx Driver

By this time the ADS1299EEG FE software will have prompted the user an with error message. Click 'OK'. It may be necessary to close the program, power cycle the ADS1299EEG FE and restart the program. This process may need to be done again should you plug the ADS1299ECG FE into a different USB port on your computer.

# 4 ADS1299EEG-FE Daughter Card Hardware Introduction

## CAUTION

Many of the components on the ADS1299EEG-FE are susceptible to damage by electrostatic discharge (ESD). Customers are advised to observe proper ESD handling procedures when unpacking and handling the EVM, including the use of a grounded wrist strap, bootstraps, or mats at an approved ESD workstation. An electrostatic smock and safety glasses should also be worn.

The ADS1299 ECG front-end evaluation board is configured to be used with the TI MMB0 data converter evaluation platform. The key features of the ADS1299 system on a chip (SOC) are:

- Eight integrated INAs and eight 24-bit high-resolution ADCs
- Low channel noise of 1 µVpp for 65-Hz bandwidth
- Low power consumption (5mW/channel)
- Data rates of 250SPS to 16kSPS
- 5V unipolar or bipolar analog supply, 1.8V to 3.6V digital supply.
- DC /AC Lead off detection
- On-chip oscillator
- On-chip bias amplifier
- Versatile MUX to enable programmable reference and bias electrode
- SPI data interface

The ADS1299EEG-FE can be used to evaluate the performance of ADS1299 chip. Users can provide any type of signal directly to the ADS1299 through a variety of hardware jumper settings (J6, JP25). External support circuits are provided for testing purposes such as external references, clocks, lead-off resistors, and shield drive amplifiers.

Figure 13 shows the functional block diagram with important jumper names for the EVM.

![img-11.jpeg](./images/img-11.jpeg)
Figure 13. ADS1299 EEG-FE Front End Block Diagram

The ADS1299EEG-FE board is a four-layer circuit board. The board layout is provided in Section 9; the schematics are appended to this document. The following sections explain some of the hardware settings possible with the EVM for evaluating the ADS1299 under various test conditions.

## 4.1 Power Supply

The EEG front-end EVM mounts on the MMB0 EVM with connectors J2, J3 and J4. The main power supplies (+5V, +3V and +1.8V) for the front-end board are supplied by the host board (MMB0) through connector J4. All other power supplies needed for the front-end board are generated on board by power management devices. The EVM is shipped in +5V unipolar supply configuration.

The ADS1299 can operate from +5.0V analog supply (AVDD/AVSS) and +1.8V to +3.0V digital supply (DVDD). A bipolar analog supply (±2.5V) can be used as well. The analog power consumption of the front-end board can be measured by the current flowing through the JP2 jumper and JP20 jumper. The ADS1299 can be powered down by shorting jumper JP5.

Test points TP5, TP6, TP7, TP8, TP9, TP10, and TP14 are provided to verify that the host power supplies are correct. The corresponding voltages are shown in Table 2.

Table 2. Power Supply Test Points

|  Test Point | Voltage  |
| --- | --- |
|  TP7 | +5.0V  |
|  TP9 | +1.8V  |
|  TP10 | +3.3V  |
|  TP5 | +5.0V  |
|  TP13 | +2.5V  |
|  TP6 | -2.5V  |
|  TP8 | GND  |

The front-end board must be properly configured in order to achieve the various power-supply schemes. The default power-supply setting for the ADS1299EEG-FE is a unipolar analog supply of 5V and DVDD of either +3V or +1.8V. Table 3 shows the board and component configurations for each analog power-supply scheme; Table 4 shows the board configurations for the digital supply.

Table 3. Analog Supply Configurations

|  Power Supplies | Unipolar Analog Supply | Bipolar Analog Supply  |
| --- | --- | --- |
|   |  5V | ±2.5V  |
|  JP2 (AVDD) | 2-3 (default) | 1-2  |
|  JP20 (AVSS) | 1-2 (default) | 2-3  |
|  U8 | Don't Care | TPS72325  |
|  U9 | Don't Care | TPS73225  |

Table 4. Digital Supply Configurations

|  DVDD | +3.0V | +1.8V  |
| --- | --- | --- |
|  JP24 | 2-3 (default) | 1-2  |

## 4.2 Clock

The ADS1299 has an on-chip oscillator circuit that generates a 2.048-MHz clock (nominal). This clock can vary by $\pm 5\%$ over temperature. For applications that require higher accuracy, the ADS1299 can also accept an external clock signal. The ADS1299EEG-FE provides an option to test both internal and external clock configurations. It also provides an option to generate the external clock from either the onboard oscillator or from an external clock source.

The onboard oscillator is powered by the DVDD supply of the ADS1299. Care must be taken to ensure that the external oscillator can operate either with +1.8V or +3.0V, depending on the DVDD supply configuration. Table 5 shows the jumper settings for the three options for the ADS1299 clocks.

Table 5. Clock Jumper Options

|  ADS1299 Clock | Internal Clock from the ADS1299 | Clock from Oscillator on the EVM | External Clock Source  |
| --- | --- | --- | --- |
|  JP18 | Not Installed | 2-3 (default) | 1-2  |
|  JP19 | Don't Care | 1-2 | Don't Care  |
|  J3– pin 17 | Don't Care | Don't Care | External Clock Source  |

A 2.048-MHz oscillator available for +3V and +1.8V DVDD is the FXO-HC735-2.048 MHz and SiT8002AC-34-18E-2.048, respectively. The EVM is shipped with the external oscillator enabled.

## 4.3 Reference

The ADS1299 has an on-chip internal reference circuit that provides reference voltages to the device. Alternatively, the internal reference can be powered down and VREFP can be applied externally. This configuration is achieved with the external reference generator (U3) and driver buffer. The EVM has the footprints for the necessary circuitry, but the components are not installed at the factory.

The external reference voltage can be set to 4.096V. Measure TP3 to make sure the external reference is correct. The setting for the external reference is described in Table 6.

Table 6. External Reference Jumper Options

|  ADS1299 Reference | Internal Reference | External Reference  |
| --- | --- | --- |
|   |  VREF = 4.5V | VREFP = 4.096V  |
|  JP3 | Not Installed | Installed  |

The software uses the VREF value entered in the Global Registers control tab (refer to Section 5.2) to calculate the input-referred voltage value for all the tests. The default value is 4.5V. If any other value is used, the user must update this field in the Global Registers control tab.

## 4.4 Accessing ADS1299 Analog Signals

Some ADS1299 output signals are provided as test points for probing purposes through J5. Table 7 lists the various test signals with the corresponding test points.

Table 7. Test Signals

|  Signal | J5 Pin Number |   | Signal  |
| --- | --- | --- | --- |
|  RESERVE | 1 | 2 | RESERVE  |
|  RESERVE | 3 | 4 | RESERVE  |
|  PWDN | 5 | 6 | GPIO4  |
|  DAISY_IN | 7 | 8 | GPIO3  |
|  AGND | 9 | 10 | RESERVE  |

## 4.5 Accessing ADS1299 Digital Signals

The ADS1299 digital signals (including SPI interface signals, some GPIO signals, and some of the control signals) are available at connector J3. These signals are used to interface to the MMB0 board DSP. The pin out for this connector is given in Table 8.

Table 8. Serial Interface Pin Out

|  Signal | J3 Pin Number |   | Signal  |
| --- | --- | --- | --- |
|  START/CS | 1 | 2 | CLKSEL  |
|  CLK | 3 | 4 | GND  |
|  NC | 5 | 6 | GPIO1  |
|  CS | 7 | 8 | RESETB  |
|  NC | 9 | 10 | GND  |
|  DIN | 11 | 12 | GPIO2  |
|  DOUT | 13 | 14 | NC/START  |
|  DRDYB | 15 | 16 | SCL  |
|  EXT_CLK | 17 | 18 | GND  |
|  NC | 19 | 20 | SDA  |

# 4.6 Analog Inputs

The ADS1299EEG-FE is designed so that it can be used as a eight channel data acquisition board. Arbitrary input signals can be fed to the ADS1299 by feeding the signal directly at connector J6. Figure 14 shows the input configurations that are available in the EVM.

![img-12.jpeg](./images/img-12.jpeg)
(a)

![img-13.jpeg](./images/img-13.jpeg)
(b)
A single channel without mux is shown for simplicity.
Figure 14. Input Configurations Supported by the EEG-FE a) Differential Inputs b) Single-Ended Inputs

# 4.6.1 Differential Inputs

To digitize eight differential inputs,

1. Set all jumpers to factory defaults as described in Section 2.5.
2. Remove jumpers from pin 5-36 of J6.
3. Provide the differential inputs on the even pins 6-36 of J6.

While used with differential inputs, care needs to be taken to ensure that the analog inputs are within the input common mode range of the PGA. If the input differential signal is centered around 0V, the ADS1299 needs to be operated with a bipolar supply. Refer to Section 4.1 for details on setting the EVM to operate with a bipolar supply.

# 4.6.2 Single-Ended Inputs

For single-ended inputs, the measurement can be done with respect to the voltage applied to the SRB1 pin of the ADS1299. To digitize eight single-ended inputs,

1. Set all jumpers to factory defaults as described in
2. Remove jumpers from pin 5-36 of J6.
3. Short pin 5 and 6 of JP25. Set the SRB1 bit in the MISC1 register to route the SRB1 pin to the negative input of the channels (refer to Section 8.3 for details). This will route BIAS_ELEC (mid supply) to the negative inputs of the channels through the SRB1 pin.
4. Provide the single-ended inputs to pins 36, 32, 28, 24, 20, 16, 12, 8 of J6 for channels 1-8 respectively.

Apart from providing the option to feed inputs directly at the jumper (for general purpose data acquisition), the ADS1299 EVM provides multiple configurations specific to the EEG application. These configurations are explained in detail in Section 7.

# 5 Using the Software: ADS1299 Control Registers and GUI

Before starting to use the EVM software, there is one important feature that users should be aware of. The software GUI contains a Save tab that allows all data from any combination of channels to be saved in a given directory location with notes to describe the saved data. Figure 15 shows the Save tab options.

![img-14.jpeg](./images/img-14.jpeg)
Figure 15. File Save Option Under 'Save' Tab

# 5.1 Overview and Features

This section provides a quick overview of the various features and functions of the ADS1299EEG-FE software package.

There are four primary tabs across the left side of the GUI:

- About tab: Provides information about the EVM and software version revisions.
- ADC Register tab: Includes all of the control registers for the ADS1299, in a series of related sub-tabs:
- Channel Registers tab
- LOFF and BIAS tab
- GPIO and Other Registers tab
- Register Map tab
- Analysis tab: Provides different ways to analyze captured data in the time or frequency domain, with a series of related sub-tabs:

- Scope tab
- FFT tab
- Histogram tab

- Save tab: Provides options for saving data

## 5.2 Global Channel Registers

The first section under the Channel Registers→Global Channel Registers tab allows the user to manipulate the entire ADS1299 configuration and lead-off registers. The Global Channel Registers box includes Configuration Register 1 (controls daisy-chain/MRB mode, clock connection, and data rate); Configuration Register 2 (controls internal test source amplitude and frequency); Configuration Register 3 (controls the reference buffer power-up/-down processes, the reference voltage, the bias drive enable/disable, and the bias reference); and the Lead-Off Control Register (controls the comparator threshold and the magnitude and frequency of the lead-off signal). shows the GUI panel to manipulate these registers and the respective settings for each.

![img-15.jpeg](./images/img-15.jpeg)
Figure 16. Channel Registers GUI for Global Registers

## 5.3 Channel Control Registers

The second section under the Channel Registers tab is the Channel Control Registers box. This panel allows the user to uniquely configure the front-end MUX for each channel. Additionally, at the top of the Channel Control Registers box is the option to globally set all channels to the same setting. The channel-specific MUX is illustrated in Figure 17. The panel snapshot for the channel control registers is shown in Figure 18. Figure 19 shows the register bit to control the switches which connect all channels negative input to SRB1 pin. This bit is located in "GPIO and other registers" tab.

![img-16.jpeg](./images/img-16.jpeg)
Figure 17. Input Multiplexer for a Single Channel (MAIN = [000 or 110 or 111])

![img-17.jpeg](./images/img-17.jpeg)
Figure 18. Channel Control Registers GUI Panel

![img-18.jpeg](./images/img-18.jpeg)
Figure 19. Register Bit for SRB1 Routing

## 5.3.1 Internal Test Signals Input

Configuration Register 2 controls the signal amplitude and frequency of an internally-generated square wave test signals. The primary purpose of this test signal is to verify the functionality of the front-end MUX, the PGA, and the ADC. The test signals may be viewed on the Analysis→Scope tab, as Figure 20 shows. Detailed instructions for using the Analysis→Scope tab is provided in Section 6.1.1.

![img-19.jpeg](./images/img-19.jpeg)
Figure 20. Internal Test Signals

## 5.3.2 Temperature Sensor and the Scope Tab

The internal temperature sensor on the ADS1299 is shown in . When the internal MUX is routed to the temperature sensor input, the output voltage of the ADC may be converted to a temperature value, using Equation 1.

$$
\text{Temperature } (^{\circ}\mathrm{C}) = \left[ \frac{\text{Temperature Reading } (\mu\mathrm{V}) - 145,300\mu\mathrm{V}}{490\mu\mathrm{V}/^{\circ}\mathrm{C}} \right] + 25^{\circ}\mathrm{C} \tag{1}
$$

![img-20.jpeg](./images/img-20.jpeg)
Figure 21. Simplified Diode Arrangement

The output voltage corresponding to a given temperature can be read selecting the Temperature Sensor option on the Channel Control Registers GUI (see Figure 17) and verified using the Analysis→Scope tab as shown in Figure 22. The number 0.146V (on the y-axis) can be calculated as a temperature using Equation 1:

Temperature = (0.146 - 0.145300) / 0.00049 + 25 = 26.4°C

It should be noted that the temperature sensor input cannot be used with a gain setting of 24 as it will saturate the PGA output.

![img-21.jpeg](./images/img-21.jpeg)
Figure 22. Eight Channel Read of Internal Temperature Data

# 5.3.3 Normal Electrode Input

The Normal electrode input on the MUX routes the inputs (VINP and VINN) differentially to the internal PGA, as Figure 17 illustrates. An exception is if the SRB1 bit is set high. If channel is in Normal electrode mode and SRB1 bit is set high the signal on SRB1 pin is routed to negative inputs of all channels instead of VINN inputs.

# 5.3.4  $\mathbf{MV}_{\mathrm{DD}}$  Input and the Scope Tab

The  $\mathrm{MV}_{\mathrm{DD}}$  input option allows the measurement of the supply voltage  $V_{S} = (AV_{DD} + AV_{SS}) / 2$  for channels 1, 2, 5, 6, 7, and 8; however, the supply voltage for channel 3 and 4 will be  $DV_{DD} / 4$ . As an example, in bipolar supply mode,  $AV_{DD} = 3.0V$  and  $AV_{SS} = -2.5V$ . Therefore, with the PGA gain  $= 1$ , the output voltage measured by the ADC will be approximately  $0.25V$ .

# 5.3.5 Bias Measurement

This measurement takes the voltage at the BIASIN pin and measures it on the PGA with respect to (AVDD + AVSS)/2 or BIASREF. This option can be used to give a calibration/test signal to ADS1299 device without connecting the calibration/test signal to the electrodes. The positive signal can be applied to BIASIN pin and the negative input can be applied to the BIASREF pin. More details on this can be found in Section 7.3.

# 5.3.6 Bias Positive Electrode Drive and Bias Negative Electrode

This option can be used to have a selectable bias electrode. This option routes the signal on BIASIN pin to any of positive or negative pins of the channel inputs.

# 5.4 GPIO and Other Registers

The GPIO and Other Registers tab, located under the Analysis tab, includes controls for GPIO1 through GPIO4, SRB1 control, pulse mode control and lead off comparators power down. The GPIO registers control four general-purpose I/O pins. Figure 23 illustrates the GPIO Control Register GUI panel.

![img-22.jpeg](./images/img-22.jpeg)
Figure 23. GPIO Control Register GUI Panel

# 5.5 Lead-Off and BIAS Registers

The Lead-Off Detection and Current Control Registers and the Bias Derivation Control Registers are located under the ADC Register  $\rightarrow$  LOFF and BIAS tab.

# 5.5.1 Lead-Off Sense (LOFF_SENSP and LOFF_SENSN) Registers

These registers enable lead-off detection for both the positive and negative channels. Figure 24 describes the 4-bit DAC settings to configure the lead-off threshold. Note that the LOFF_FLIPx bits change the direction of the lead-off current if this option is selected. Figure 24 illustrates the connections from the positive and negative inputs to the lead-off comparators. Figure 25 shows the respective GUI panel on the EVM software.

![img-23.jpeg](./images/img-23.jpeg)
Figure 24. LOFF_STATP and LOFF_STATN Comparators

![img-24.jpeg](./images/img-24.jpeg)
Figure 25. LOFF_SENSP and LOFF_SENSN Registers GUI Panel

# 5.5.2 Lead-Off Status Registers (LOFF_STATP and LOFF STATN)

These registers store the output of the lead-off comparator that corresponds with each input. When a lead is disconnected, the corresponding register bit activates low. The GUI for this feature is enabled by clicking in the upper right-hand corner of the EVM software on the Show/Poll Lead-Off Status button. Pressing this button causes a pop-up box that shows the status of the lead-off registers. The GUI shows when a lead is disconnected by turning its bit from green to red. Figure 26 illustrates the Lead-Off Status Registers GUI controls.

![img-25.jpeg](./images/img-25.jpeg)
Figure 26. Lead-Off Status Indicator

## 5.5.3 Bias Drive Derivation Control Registers

The Bias Drive Derivation Control Registers enable the user to set any combination of positive and/or negative electrodes to derive the BIAS voltage that is fed to the internal bias drive amplifier. Figure 27 shows the corresponding GUI controls. The details about bias drive can be found in Section 5.5.

![img-26.jpeg](./images/img-26.jpeg)
Figure 27. BIAS_SENSP and BIAS_SENSN GUI Panel

# 5.6 Register Map

The Register Map  $\rightarrow$  Device Registers tab is a helpful debug feature that allows the user to view the state of all the internal registers. This tab is illustrated in Figure 28.

![img-27.jpeg](./images/img-27.jpeg)
Figure 28. Device Register Settings

# 6 ADS1299EEG-FE Analysis Tools

Under the Analysis tab in the ADS1299EEG-FE GUI software, there are four different analysis tools shown that enable a detailed examination of the signals selected by the front-end MUX:

- Scope
- Analysis
- Histogram
- FFT

These tools are detailed in the following subsections.

# 6.1 Scope Tab

## 6.1.1 Using the Analysis→Scope Tool

The Scope tool (available under the Analysis tab) is a very useful means of examining the exact amplitude of the measured input signals from each channel. Additionally, users can determine the noise contribution from each channel at a given resolution, and review the sampling rate, the PGA gain, and the input signal amplitude. Figure 29 illustrates the Scope tool features.

![img-28.jpeg](./images/img-28.jpeg)
Figure 29. Scope Tool Features

## 6.1.2 Waveform Examination Tool

The waveform examination tool allows the user to zoom in either on all channels simultaneously or on a single channel. Figure 30 shows an example of the waveform examination tool with the magnifying glass zoomed in on 90 samples.

![img-29.jpeg](./images/img-29.jpeg)
Figure 30. Zoom Option on the Waveform Examination Tool

## 6.2 Histogram Tool

The Histogram tool is located under the Analysis→Histogram tab.

## 6.2.1 Using the Analysis→Histogram Tool

The Analysis→Histogram tool is used primarily to view the bin separation of the different amplitudes of the EEG waveform harmonics. Figure 31 illustrates the histogram output for input short on all channels. The same Signal Zoom analysis may be used on the histogram plots for a more detailed examination of the amplitude bins. The Analysis table gives the mean of the input signal and also the rms and peak-to-peak value of the signal on each channel.

![img-30.jpeg](./images/img-30.jpeg)
Figure 31. Histogram Bins for Input Short Noise

## 6.3 FFT Tool

The FFT tool is located under the Analysis→FFT tab.

## 6.3.1 Using the Analysis→FFT Tool

The Analysis→FFT tool allows the user to examine the channel-specific spectrum as well as typical figures of merit such as SNR, THD, ENOB, and CMRR. Each feature is numbered below and described in detail in the following subsections. Figure 32 illustrates an Analysis→FFT plot for input short configuration. The explanation of different tabs is explained below.

![img-31.jpeg](./images/img-31.jpeg)
Figure 32. Analysis : FFT Graph of Input Short Test

# Coherent Frequency Calculator: 1

Coherent sampling in an FFT is defined as  $F_{\text{AIN}} / F_{\text{SAMPLE}} = N_{\text{WINDOW}} / N_{\text{TOTAL}}$ , where:

-  $F_{\text{AIN}}$  is the input frequency
- FSAMPLE is the sampling frequency of the ADS1299
-  $N_{\text{WINDOW}}$  is the number of odd integer cycles during a given sampling period
-  $N_{\text{TOTAL}}$  is the number of data points (in powers of 2) that is used to create the FFT. If the conditions for coherent sampling can be met, the FFT results for a periodic signal will be optimized. The Ideal  $A_{IN}$  Frequency is a value that is calculated based on the sampling rate, such that the coherent sampling criteria can be met.

# AC Analysis Parameters: 2

This section of the tool allows the user to dictate the number of harmonics, dc leakage bins, harmonic leakage bins, and fundamental leakage bins that are used in the creation of various histograms. Pressing the Windowing button, illustrated in Figure 33, allows the user to evaluate the FFT graph under a variety of different windows. Note that pressing the Reference button toggles between dBFS (decibels, full-scale) and dBc (decibels to carrier).

![img-32.jpeg](./images/img-32.jpeg)
Figure 33. Analysis : FFT : AC Analysis Parameters : Windowing Options

# FFT Analysis: 3

Pressing the FFT Analysis button pulls up the window shown in Figure 34. This window can be useful because the different tabulated figures of merit can show more detailed information about the channel-to-channel noise.

![img-33.jpeg](./images/img-33.jpeg)
Figure 34. Analysis : FFT : FFT Analysis : Input Short Condition

# User-Defined Dynamic Range: 4

This section enables the user to examine the SNR of a specific channel within a given frequency band defined by Low Frequency and High Frequency. The SNR displayed in this window will also show under the Dynamic Range heading as Figure 35 illustrates.

![img-34.jpeg](./images/img-34.jpeg)
Figure 35. Changing the User-Defined Dynamic Range for Channel 1

Input Amplitude: 5

This field is a user input that is important for accurately calculating the CMRR of each channel.

# 7 EEG Specific Features

This section describes some of the EEG specific features supported by the EVM, including the reference/patient bias signals, lead off detection and calibration.

# 7.1 Reference Signal and Patient Bias Signal

A typical EEG system has multiple electrodes (32 up to 256, hereby called as the "normal electrodes") connected to the scalp that are used to acquire EEG signals. In addition to these electrode signals, an EEG system also uses two additional signals, a reference signal and a patient bias signal. The reference signal is used as the reference for the single-ended EEG measurements. The patient bias signal is used for biasing the patient to set the common mode of the EEG signals (typically mid supply).

# Dedicated reference and patient bias electrodes

Many EEG systems have two dedicated electrodes, one used as the reference signal for the EEG measurement (hereby called as the "reference electrode") and the other used for the patient bias signal (hereby called as the "bias electrode"). The EVM has two signals (BIAS_ELEC, REF_ELEC) available at the connector JP25 that correspond to these two electrodes. The BIAS_DRV signal is similar to the BIAS_ELEC, but appears as a separate signal. The BIAS_DRV can be used as the BIAS_ELEC signal by installing JP1 (1-2). It can also be used as the input to the BIAS_SHD buffer by installing JP17 and the additional shield drive circuitry.

# Programmable reference and patient bias electrodes

Certain EEG systems provide the flexibility to be able to route the reference and/or the patient bias signals through any of the normal electrodes.

The internal multiplexer of the ADS1299 provides ample flexibility for

(a) Choosing the voltage applied to these electrodes (Fixed or closed loop),
(b) Being able to route the reference and patient bias signals to either the dedicated electrode or any other normal electrode.

# 7.1.1 Using the Dedicated Reference and Patient Bias Electrodes

This is the simplest option for electrode connection and is illustrated in Figure 36. One dedicated electrode is chosen as a bias electrode and a potential is applied to it to bias the patient at about mid-supply voltage. Similarly a fixed electrode is chosen as the reference electrode and all the other electrodes are measured with respect to this electrode. Below we discuss different options available on the EVM board to connect the bias electrode BIAS_ELEC/BIAS_DRV and reference electrode REF_ELEC.

![img-35.jpeg](./images/img-35.jpeg)
Figure 36. Dedicated Reference and Bias Electrode

Reference : The reference electrode (REF_ELEC) input is used to drive the negative inputs of the channel through SRB1 pin on ADS1299 device. The reference electrode is connected to the negative inputs of all the channels. This leads to increased leakage current on the reference electrode since current of all the channels gets added. The EVM provides an option to buffer the reference electrode to reduce the leakage. The disadvantage of the buffered approach is the additional noise of the buffer amplifier. The table below shows the jumper settings for the two options.

Table 9. Dedicated Reference Drive Options through REF_ELEC

|   | JP7 | JP8  |
| --- | --- | --- |
|  Un Buffered | Don’t care | 1-2  |
|  Buffered | 1-2 | 2-3  |

Bias : There is an option to provide the bias to a fixed electrode either through BIAS_ELEC or through BIAS_DRV. BIAS_ELEC option needs an external amplifier U11 to buffer the mid supply. For the BIAS_DRV option the buffer is built inside the ADS1299 chip. The BIAS_DRV option also helps in improving common mode rejection by implementing a feedback loop. The details on selecting the inputs for bias drive are discussed in Section 5.5.3 and Section 7.1.3.

## 7.1.2 Programmable Reference and Bias Electrodes

The multiplexer in ADS1299 allows any electrode to be chosen as the bias electrode or reference electrode. This is illustrated in Figure 37.

![img-36.jpeg](./images/img-36.jpeg)
Figure 37. Programmable Reference and Bias Electrode

The reference electrode selection is done using SRB2 pin. The SRB2 bit in CHxSET register is set high for the electrode chosen as reference. This reference is routed out on SRB2 pin and can be routed to SRB1 pin as a reference for all other channels. On the EVM, a jumper between pin 2 and pin 3 of JP7 and JP8 is needed for this configuration. In Figure 37, the channel 1 electrode is selected as a reference electrode and is routed out to SRB2 pin.

The bias selection is done using BIASIN pin. The voltage in this pin can be routed to positive input of any channel by writing MUX = 110 on the CHxSET register. On the EVM a jumper between pin 2 and pin 3 of JP6 is required, to route the mid supply to BIASIN. In the illustration in Figure 37 channel 7 is used as a bias electrode.

## 7.1.3 Biasing the Patient with a Feedback Loop

There are two options on the EVM board to bias the patient. First option is to use onboard BIAS_ELEC signal to drive the patient as explained in the earlier section. Second option, which is described below, is to drive the body with BIAS_DRV signal generated by ADS1299 chip. The advantage of using BIAS_DRV signal is that it takes advantage of feedback loop to get better common mode rejection. The bandwidth of the BIAS loop is determined by R8 (390kΩ) and C20 (10nF). Users can change these values to set the bandwidth based on the specific application. The stability of the loop is determined by the user's specific system. Therefore, optimization may be needed on the feedback component values to ensure stability if additional filtering components and long cables are added before the ADS1299EEG-FE.

The ADS1299 offers full flexibility by letting the user select any combination of the electrodes to generate the bias voltage. Refer to the ADS1299 data sheet (SBAS499) for more details.

The reference voltage for the on-chip right leg drive can be driven externally. The on-chip voltage is set to mid-supply. If the application requires the common mode to be set to any other voltage, this configuration can be accomplished by setting the appropriate bit in the Configuration 3 Register. The external BIASREF voltage is set by resistor R1 and adjustable resistor R2.

The following procedure needs to be applied to activate the Bias drive circuitry:

Step 1. Set the inputs to Normal Electrode, refer Figure 38

![img-37.jpeg](./images/img-37.jpeg)
Figure 38. Settings for Normal Electrode
Figure 39. Configuring BIASREF and Bias Drive Buffer
Figure 38. Select the electrodes to be chosen for the bias drive loop. In this case, the channel 1 and 2 input signals are used (as Figure 40 shows).

Step 2. Turn on the bias drive buffer and set the internal bias drive reference; refer to Figure 39.

![img-38.jpeg](./images/img-38.jpeg)
Step 3. Select the electrodes to be chosen for the bias drive loop. In this case, the channel 1 and 2 input signals are used (as Figure 40 shows).

![img-39.jpeg](./images/img-39.jpeg)
Figure 40. Setting up the Bias Drive Loop

Once these steps are completed, measure and verify that the voltage on either side of R8 is close to mid-supply. This measurement confirms whether the Bias drive loop is functional. Apart from the BIAS_DRV signal, the ADS1299EEG-FE also offers an option to drive the cable shield. The EEG cable shield signal can be connected to BIAS_SHD. The jumper (1-2) on JP17 must be shorted to enable the shield drive. The footprints for the components needed for the shield drive circuitry are available on the board. But the components are not installed at the factory.

## 7.2 Lead-Off Detection

The ADS1299 provides multiple schemes to implement the lead-off detection function. These schemes include current source at dc, at 7.8 Hz, 31.2 Hz or at $f_{\mathrm{DRi4}}$. There is also a wide range on the amplitude of currents available. Refer to the ADS1299 product data sheet (SBAS499) for additional details.

While attempting to use the lead-off detection, care must be taken to analyze the input signal. If the input signal is dc-coupled, the dc lead-off scheme can be used. If the input signal is ac-coupled, the ac lead-off scheme must be used. When using the dc lead-off scheme, be sure to bias the patient to set the input common-mode before activating lead-off detection.

## 7.2.1 DC Lead-Off

At board power-up, the firmware sets the appropriate registers so that dc lead-off is selected. In the event of a reset signal, the register values default to the device default settings. In such a scenario, follow this procedure to reactivate the lead-off circuitry.

Step 1. Make sure the input is dc-coupled and that the bias drive circuit is operational, as explained in Section 7.1.3

Step 2. Choose the lead-off scheme by setting the respective bits in the LOFF register (in the LOFF control tab). Select the DC Lead-Off Detect, 6.25 nA, Current Source scheme, and set the comparator threshold to 95%. Select the appropriate inputs for lead-off detection by clicking the bits of the LOFF_SENSP and LOFF_SENSN Registers. The LOFF tab should appear as shown in Figure 41.

![img-40.jpeg](./images/img-40.jpeg)
Figure 41. Setting the LOFF Register Bits

Step 3. Turn on the lead-off comparator by setting the bit in the Configuration 4 Register in the Global Registers control tab, as Figure 42 shows.

```txt
Configuration Register 4 (CONFIG4)
Lead-off Comparator Power-down
Comparator Enabled
```

Step 4. The software has an option where the LOFF_STATP and LOFF_STATM Registers are continuously polled (set the Read Status Registers switch as shown in shown in Figure 43). This option allows the user to see the lead-off detection scheme work in real time. Figure 44 shows a case for which only positive electrodes are connected.

![img-41.jpeg](./images/img-41.jpeg)
Figure 42. Configuring the Lead Off Comparator
Figure 43. Lead off Status Registers

## 7.2.2 AC Lead-Off Detection

AC lead off detection can be used in three ways

1. To measure electrode impedance with inband excitation for one time use at electrode placement.
2. To simultaneously measure electrode impedance with EEG, by using out of band excitation.
3. To detect if a lead is off for an AC coupled input.

These options are explained below.

## In band Electrode impedance measurement

ADS1299 provides two frequency options (7.8 Hz and 31.25 Hz) to measure the electrode impedance within the bandwidth of interest for EEG. There are four amplitude of current source (I_Leadoff) options available 6 nA, 24 nA, 6 µA, and 24 µA. The electrode impedance measurement at these frequencies cannot be done simultaneously with EEG measurements. The voltage developed at the -inputs depends on the impedance on each electrode and the current used for lead-off detection. If we denote the source impedance on INP pin as Z_inp and the source impedance on INM pin as Z_inn the peak to peak voltage developed on channel input is $2 \times (I_{\text{Leadoff}} \times Z_{\text{inp}} + I_{\text{Leadoff}} \times Z_{\text{inn}})$.

As an example Figure 44 shows the snapshot of the scope with 5K impedance on each source with $6\mu \mathrm{A}$ of lead-off current at $31.25\mathrm{Hz}$. We expect a theoretical peak to peak voltage of $120\mathrm{mV}$. The observed peak-to-peak voltage is $128\mathrm{mV}$ which is within the tolerance specification of current source. The results can also be analyzed in frequency domain using the FFT analysis tab as shown in Figure 45. The magnitude of the fundamental component will be directly proportional to the electrode impedance being measured.

![img-42.jpeg](./images/img-42.jpeg)
Figure 44. Scope tab for Impedance Measurement at 31.25 Hz

![img-43.jpeg](./images/img-43.jpeg)
Figure 45. FFT Analysis for Impedance Measurement at 31.25 Hz

# Out of band Lead off detection

ADS1299 also provides option to do electrode impedance measurement at frequencies outside the EEG bandwidth of interest. The frequency for this AC current source is set at  $f_{\mathrm{DR}} / 4$ . For example, to do an AC lead-off detection at 1 kHz, the data rate for the device must be set at 4Ksps. These measurements can be done concurrently with the EEG measurement. Figure 46 shows the fft result of AC lead off detection at fDR/4 with data rate of 4Ksps. The impedance component is present at 1 KHz and must be bandpass filtered. The EEG information is at low frequencies and the data must be low pass filtered to extract the information. It is recommended to use only nA range current sources for concurrent measurement of EEG and impedance. For μA range the noise from the current source will be too large and it may swamp the EEG signal.

![img-44.jpeg](./images/img-44.jpeg)
Figure 46. Scope Tab for Impedance Measurement at fDR/4 (DR = 4ksps)

## 7.3 External Calibration/Test Signals

ADS1299 generates a square wave test signal that can be used to check the functionality of the signal chain (Refer to the datasheet for details). It also gives the user an option to provide external test signals for calibration. For evaluation purposes with the EVM, the test signals can be provided directly to the jumpers of the corresponding signals. SRB1 (pin2 of JP8), SRB2 (pin3 of JP7), BIASIN (Pin3 of JP6), BIASREF (does not appear at a jumper, needs to be soldered to one side of R5).

## 7.3.1 Channel Inputs Disconnected

It may sometimes be required to provide a calibration or test signal to ADS1299 channel without the signal being routed to the channel input pins (or electrodes). This can be accomplished by applying the positive test signal to BIASIN pin and the negative test signal to BIASREF pin. The channel multiplexer must be set as 010, BIASREF_INT bit in Config 3 register must be set to 0 to choose external BIASREF and BIAS_MEAS bit in Config 3 must be set to 1. These multiplexer settings are illustrated in Figure 47.

![img-45.jpeg](./images/img-45.jpeg)
Figure 47. Multiplexer Setting for Calibration with Electrode Disconnected

## 7.3.2 Channel Inputs Connected

It may sometimes be required to provide a calibration or test signal to ADS1299 device with the positive input connected to the pin or electrode. This can be accomplished by connecting the positive test signal to SRB2 pin and the negative test signal to SRB1 pin. The channel input multiplexer must be set for Normal Electrode (000), SRB2 switch must be closed and SRB1 switch must be closed. This multiplexer setting is illustrated in Figure 48.

![img-46.jpeg](./images/img-46.jpeg)
Figure 48. Multiplexer Setting with Positive Electrode Connected to Test Signal

If it is desired to have both the input pins connected during calibration or test, the following connections must be made. The positive test signal must be tied to SRB2 pin and the negative test signal must tie to BIASIN pin. The channel multiplexer must be set for 111 and the SRB2 switch must be closed. This multiplexer setting is illustrated in Figure 49.

![img-47.jpeg](./images/img-47.jpeg)
Figure 49. Multiplexer Setting with Both Electrodes Connected to Test Signal

# 8 Test Options on the EVM

## 8.1 On-Chip (ADS1299) Input Short

The channel input can be shorted internally by setting the input multiplexer of the individual channel to 001. The channel control registers must be set as shown in Figure 50. This test gives the noise in the channel. It also gives the offset in the channel. The result can be seen in the analysis tab. Figure 51 shows a snapshot of the scope for internal input short with gain setting of 24. The channel offset in this example is $23\,\mu\mathrm{V}$ and noise is less than $1\,\mu\mathrm{Vpp}$. 5000pts at 500sps is taken, thereby giving data for 10 seconds.

![img-48.jpeg](./images/img-48.jpeg)
Figure 50. Channel Settings for Input Short Test

![img-49.jpeg](./images/img-49.jpeg)
Figure 51. Scope Tab for Input Short Test

# 8.2 External Input Short with 5K Resistor

There is an option on board to tie the positive and negative input of the channel to a common voltage (VCM) on BIAS_ELEC through 5K resistors. The following jumper settings are needed for this test. On JP6 short pin 1 and pin 2. On JP25 short (1-2) and (2-3). The connector J6 must have jumpers across from left to right to connect the inputs to the ADS1299 channels. The noise from U11 which is used to generate the BIAS_ELEC appears as common mode noise for this test and is rejected. Same is true for noise from resistor R10 in BIAS_ELEC path. The only noise source present are two 5K resistors in the input path and the channel noise. This test is useful to measure the effect of input bias current on noise.

The PGA in ADS1299 has CMOS input and thus has negligible current noise. The input bias current is as a result of chopping the PGA to remove flicker noise. This bias current doesn't manifest itself as noise and appears like a DC offset in presence of 5K input impedance. The Channel control registers must be programmed as shown in Figure 52. The results in the analysis tab are shown in Figure 53. The average peak-to-peak noise for this test is $1.27\mu \mathrm{V}$. The increase in noise is due to the noise from 5K resistors. The two 5K resistors contribute about $0.67\mu \mathrm{Vpp}$ in 65-Hz bandwidth.

![img-50.jpeg](./images/img-50.jpeg)

Figure 52. Channel Settings for External Input Short Test

![img-51.jpeg](./images/img-51.jpeg)
Figure 53. Scope Showing Noise for Input Short with 5k Resistors

## 8.3 Noise with Common Reference on Negative Inputs

There is an option in ADS1299 to connect all the channels negative inputs to a common reference. This can be accomplished by giving a signal on SRB1 pin and setting the bit SRB1 bit in MISC1 register. There is an option on board to test out the channel noise performance with this setting. On JP25 a jumper on (3-4) and (5-6) is needed. On JP8 a jumper (1-2) is required. These settings routes the common mode voltage VCM on BIAS_ELEC to all the positive inputs. It also connects BIAS_ELEC to REF_ELEC via R11 (5K). REF_ELEC is connected to SRB1 pin on ADS1299. The noise in this test includes noise of two 5K resistors and the channel noise. The SRB1 control switch must be set as shown in Figure 54. The snapshot of the scope in the analysis tab is shown in Figure 55. The average peak-to-peak noise for this test is $1.28\mu \mathrm{V}$.

![img-52.jpeg](./images/img-52.jpeg)
Figure 54. MISC1 Register Setting for SRB1

![img-53.jpeg](./images/img-53.jpeg)
Figure 55. Noise with Negative Input Connected to SRB1 Pin

# 8.4 Noise with Buffered Common Reference Input

Connecting all the negative inputs to one reference electrode can lead to excessive leakage current on the electrode. The typical leakage current on ADS1299 channel is 200 pA. So for a 16 channel system total leakage may be as large as 3.2 nA. This number will become progressively worse as channel count is increased. If the leakage number is not acceptable there is an option to buffer the common reference input before connected it to all the negative inputs of the channel. On JP25 jumpers (3-4) and (5-6) are required. On JP8 a jumper (2-3) is required and on JP7 a jumper (1-2) is needed. The GUI settings are same as in Figure 54. Figure 56 shows a snapshot of the noise with SRB1 driven by a buffered reference. The drawback of using the buffer in the SRB1 path is increased noise. The noise contributors in these settings are two 5-kΩ resistors, op amp U4 and ADS1299 channel. As can be seen from the Figure 56 the noise with this approach is larger than noise in previous three approaches. At present OPA376 is installed on board for U4. A lower noise op amp can be used if needed.

![img-54.jpeg](./images/img-54.jpeg)
Figure 56. Noise with OPA376 in SRB1 Path

## 8.5 Internally Generated Test Signal and Other Multiplexer Inputs

ADS1299 internally generates a test signal that can be used for signal integrity check. Also the multiplexer provides options to measure supply voltage, temperature, etc. Details of these inputs can be found in Section 5.3.

## 8.6 Arbitrary Input Signal

Any input signal can be fed to the device on connector J6 as described in Section 4.6. Figure 57 shows the results obtained when a single-ended sinusoidal signal is applied to AIN1 by following the steps described in Section 4.6.2.

![img-55.jpeg](./images/img-55.jpeg)
Figure 57. Scope Tab with Sinusoidal Inputs on AIN1

# 9 Bill of Materials, Layouts and Schematics

This section contains the complete bill of materials, printed circuit board (PCB) layouts, and schematic diagrams for the ADS1299EEG-FE.

NOTE: Board layouts are not to scale. These are intended to show how the board is laid out; do not use for manufacturing ADS1299EEG-FE PCBs.

# 9.1 ADS1299EEG-FE Front-End Board Schematics

Figure 58 through Figure 62 shown the schematic diagrams of the ADS1299EEG-FE.

![img-56.jpeg](./images/img-56.jpeg)
Figure 58. ADS1299EEG-FC Schematic

![img-57.jpeg](./images/img-57.jpeg)

Figure 59. ADS1299EEG-FC Jumper Schematic

![img-58.jpeg](./images/img-58.jpeg)

Figure 60. ECG Power Supplies

![img-59.jpeg](./images/img-59.jpeg)
Figure 61. External Reference Drivers (Not Installed)

![img-60.jpeg](./images/img-60.jpeg)
Figure 62. ECG MDK Board Interface Adapter

# 9.2 Printed Circuit Board Layout

Figure 63 through Figure 68 show the ADS1299EEG-FE PCB layout.

![img-61.jpeg](./images/img-61.jpeg)
Figure 63. ADS1299EEG-FE Top Assembly

![img-62.jpeg](./images/img-62.jpeg)
Figure 64. ADS1299EEG-FE Top Layer

![img-63.jpeg](./images/img-63.jpeg)
Figure 65. ADS1299EEG-FE Internal Layer (1)

![img-64.jpeg](./images/img-64.jpeg)
Figure 66. ADS1299EEG-FE Internal Layer (2)

![img-65.jpeg](./images/img-65.jpeg)
Figure 67. ADS1299EEG-FE Bottom Layer

![img-66.jpeg](./images/img-66.jpeg)
Figure 68. ADS1299EEG-FE Bottom Assembly

# 9.3 Bill of Materials

Table 10 lists the bill of materials for the ADS1299ECG-FE.

Table 10. Bill of Materials

|  Qty | Ref Des | Description | MFR | Part Number  |
| --- | --- | --- | --- | --- |
|  1 | NA | Printed Wiring Board | TI | 6541979  |
|  19 | C1, C2, C3, C4, C5, C6, C11, C17, C23, C24, C47, C48, C49, C58, C62, C76, C77, C97, C99 | CAP CER 1UF 25V, 10% X5R 0603 | Murata | GRM188R61E105KA12D  |
|  0 | C7, C8, C15, C19, C21, C22, C34, C38, C40, C41, C42, C43, C98 | Not Installed |  |   |
|  3 | C9, C68, C71 | CAP CER 100UF 10V, 20% X5R 1210 | Taiyo Yuden | LMK325BJ107MM-T  |
|  9 | C10, C45, C46, C50, C51, C60, C61, C65, C66 | CAP CER 10UF 10V, 10% X5R 0805 | Murata | GRM219R61A106KE44D  |
|  10 | C12, C13, C14, C16, C18, C57, C69, C70, C94, C95 | CAP CER 0.1UF 50V, 10% X7R 0603 | Murata | GRM188R71H104KA93D  |
|  2 | C20, C67 | CAP CER 10000PF 50V, 10% X7R 0603 | Murata | GRM188R71H103KA01D  |
|  0 | C33, C35 | Not Installed |  |   |
|  0 | C39 | Not Installed |  |   |
|  3 | C59, C63, C64 | CAP CER 2.2UF 6.3V, 10% X5R 0603 | Murata | GRM185R60J225KE26D  |
|  17 | C72, C73, C75, C80, C81, C82, C83, C84, C85, C86, C87, C88, C89, C90, C91, C92, C93 | CAP CER 4700PF 50V, 10% X7R 0603 | Murata | GRM188R71H472KA01D  |
|  1 | AIN1 | CONN SMA JACK STRAIGHT PCB | Amphenol | 132134  |
|   |  |  | Emerson | 142-0701-201  |
|  1 | J3 (Top) | 10 Pin, Dual Row, SM Header (20 Pos.) | Samtec | TSM-110-01-T-DV-P  |
|  2 | J2, J3 (Bottom) | 10 Pin, Dual Row, SM Header (20 Pos.) | Samtec | SSW-110-22-F-D-VS-K  |
|  1 | J4 (Bottom) | 5 Pin, Dual Row, SM Header (10 Pos.) | Samtec | SSW-105-22-F-D-VS-K  |
|  0 | J5 | Not Installed |  |   |
|  1 | J6 | 18 Pin, Dual Row, Header (36 Pos.) | Samtec | SSW-118-21-F-D  |
|  11 | JP1, JP2, JP6, JP7, JP8, JP18, JP20, JP21, JP22, JP23, JP24 | 3 Position Jumper 0.1" spacing | Samtec | TSW-103-07-T-S  |
|  0 | JP3 | Not Installed |  |   |
|  4 | JP4, JP5, JP17, JP19 | 2 Pin 0.1inch, Header | Samtec | TSW-102-07-T-S  |
|  1 | JP25 | 3 Pin, Dual Row, Header (6 Pos.) | Samtec | TSW-103-07-T-D  |
|  4 | L1, L2, L4, L5 | INDUCTOR MULTILAYER 3.3UH 0805 | TDK | MLZ2012A3R3W  |
|  1 | OSC1 | OSC 2.0480 MHZ 3.3 V, HCMOS SMT | Fox | FXO-HC735-2.048MHZ  |
|  0 | R1, R4, R5, R47, R48, R49, R50, R51, R52, R53, R56, R57, R68, R69, R70 | Not Installed |  |   |
|  0 | R2 | Not Installed |  |   |
|  6 | R3, R25, R71, R72, R73, R74 | RES 0.0 OHM 1/10W 5% 0603 SMD | Yageo | RC0603JR-070RL  |
|  4 | R6, R7, R67, R75 | RES 10.0K OHM 1/10W 1% 0603 SMD | Yageo | RC0603FR-0710KL  |
|  1 | R8 | RES 392K OHM 1/10W 1% 0603 SMD | Yageo | RC0603FR-07392KL  |
|  19 | R10, R11, R12, R80, R81, R82, R83, R84, R85, R86, R87, R88, R89, R90, R91, R92, R93, R94, R95 | RES 4.99K OHM 1/10W 1% 0603 SMD | Yageo | RC0603FR-074K99L  |
|  3 | R13, R14, R15 | RES 0.0 OHM 1/16W 0402 SMD | Yageo | RC0402JR-070RL  |
|  0 | R16, R17, R18 | Not Installed |  |   |
|  2 | R23, R24 | RES 2.0M OHM 1/10W 5% 0603 SMD | Yageo | RC0603JR-072ML  |

Table 10. Bill of Materials (continued)

|  Qty | Ref Des | Description | MFR | Part Number  |
| --- | --- | --- | --- | --- |
|  5 | TP1, TP2, TP8, TP11, TP12 | TEST POINT PC MINI .040"D BLACK | Keystone | 5001  |
|  8 | TP3, TP4, TP5, TP6, TP7, TP9, TP10, TP13 | TEST POINT PC MINI .040"D RED | Keystone | 5000  |
|  1 | U1 | ADS1299, Low-Noise, 8-Channel, 24-bit analog Front-End for Biopotential Measurements | TI | ADS1299CPAG  |
|  0 | U2 | Not Installed |  |   |
|  0 | U3, U5 | Not Installed |  |   |
|  2 | U4, U11 | IC OP AMP GP 5.5MHZ SGL 8SOIC | TI | OPA376AID  |
|  0 | U4A, U11A | Not Installed |  |   |
|  1 | U6 | IC UNREG CHRG PUMP V INV SOT23-5 | TI | TPS60403DBVT  |
|  1 | U8 | IC LDO REG NEG 200MA 2.5 V, SOT23 | TI | TPS72325DBVT  |
|  1 | U9 | IC LDO REG 250MA 2.5 V, SOT23-5 | TI | TPS73225DBVT  |
|  1 | U10 | IC EEPROM 256KBIT 400KHZ 8TSSOP | Microchip | 24AA256-I/ST  |
|  30 | NA | 0.100 Shunt - Black Shunts | 3M | 969102-0000-DA  |
|  1 | NA | MMB0 Motherboard | TI | 6462011  |

# 9.4 ADS1299EEG-FE Power-Supply Recommendations

If you chose to power the MMB0 board through the wall adapter jack, it must comply with the following requirements:

- Output voltage: 5.5 VDC to 15 VDC.
- Maximum output current: ≥ 500 mA.
- Output connector: barrel plug (positive center), 2.5-mm I.D. x 5.5-mm O.D. (9-mm insertion depth).
- Complies with applicable regional safety standards.

Figure 69 shows a +6V power-supply cable (not provided in the EVM kit) connected to a battery pack with four 1.5V batteries connected in series. Connecting to a wall-powered source makes the ADS1299EEG-FE more susceptible to 50 Hz/60 Hz noise pickup; therefore, for best performance, it is recommended to power the ADS1299EEG-FE with a battery source. This configuration minimizes the amount of noise pickup seen at the digitized output of the ADS1299.

![img-67.jpeg](./images/img-67.jpeg)
Figure 69. Recommended Power Supply for ADS1299EEG-FE

# Revision History

Changes from A Revision (October 2014) to B Revision

Page

- Deleted bullet in Section 2.4 ... "Universal AC to DC wall adapter" 6
- Added new paragraph to Section 9.4... "If you chose to power the MMB0 board through the wall adapter jack" 59
- Deleted "provided in the EVM kit" from Section 9.4. 59

NOTE: Page numbers for previous revisions may differ from page numbers in the current version.

# STANDARD TERMS AND CONDITIONS FOR EVALUATION MODULES

1. Delivery: TI delivers TI evaluation boards, kits, or modules, including any accompanying demonstration software, components, or documentation (collectively, an "EVM" or "EVMs") to the User ("User") in accordance with the terms and conditions set forth herein. Acceptance of the EVM is expressly subject to the following terms and conditions.

1.1 EVMs are intended solely for product or software developers for use in a research and development setting to facilitate feasibility evaluation, experimentation, or scientific analysis of TI semiconductors products. EVMs have no direct function and are not finished products. EVMs shall not be directly or indirectly assembled as a part or subassembly in any finished product. For clarification, any software or software tools provided with the EVM ("Software") shall not be subject to the terms and conditions set forth herein but rather shall be subject to the applicable terms and conditions that accompany such Software.

1.2 EVMs are not intended for consumer or household use. EVMs may not be sold, sublicensed, leased, rented, loaned, assigned, or otherwise distributed for commercial purposes by Users, in whole or in part, or used in any finished product or production system.

2. Limited Warranty and Related Remedies/Disclaimers:

2.1 These terms and conditions do not apply to Software. The warranty, if any, for Software is covered in the applicable Software License Agreement.

2.2 TI warrants that the TI EVM will conform to TI's published specifications for ninety (90) days after the date TI delivers such EVM to User. Notwithstanding the foregoing, TI shall not be liable for any defects that are caused by neglect, misuse or mistreatment by an entity other than TI, including improper installation or testing, or for any EVMs that have been altered or modified in any way by an entity other than TI. Moreover, TI shall not be liable for any defects that result from User's design, specifications or instructions for such EVMs. Testing and other quality control techniques are used to the extent TI deems necessary or as mandated by government requirements. TI does not test all parameters of each EVM.

2.3 If any EVM fails to conform to the warranty set forth above, TI's sole liability shall be at its option to repair or replace such EVM, or credit User's account for such EVM. TI's liability under this warranty shall be limited to EVMs that are returned during the warranty period to the address designated by TI and that are determined by TI not to conform to such warranty. If TI elects to repair or replace such EVM, TI shall have a reasonable time to repair such EVM or provide replacements. Repaired EVMs shall be warranted for the remainder of the original warranty period. Replaced EVMs shall be warranted for a new full ninety (90) day warranty period.

3. Regulatory Notices:

3.1 United States

3.1.1 Notice applicable to EVMs not FCC-Approved:

This kit is designed to allow product developers to evaluate electronic components, circuitry, or software associated with the kit to determine whether to incorporate such items in a finished product and software developers to write software applications for use with the end product. This kit is not a finished product and when assembled may not be resold or otherwise marketed unless all required FCC equipment authorizations are first obtained. Operation is subject to the condition that this product not cause harmful interference to licensed radio stations and that this product accept harmful interference. Unless the assembled kit is designed to operate under part 15, part 18 or part 95 of this chapter, the operator of the kit must operate under the authority of an FCC license holder or must secure an experimental authorization under part 5 of this chapter.

3.1.2 For EVMs annotated as FCC – FEDERAL COMMUNICATIONS COMMISSION Part 15 Compliant:

**CAUTION**

This device complies with part 15 of the FCC Rules. Operation is subject to the following two conditions: (1) This device may not cause harmful interference, and (2) this device must accept any interference received, including interference that may cause undesired operation.

Changes or modifications not expressly approved by the party responsible for compliance could void the user's authority to operate the equipment.

**FCC Interference Statement for Class A EVM devices**

NOTE: This equipment has been tested and found to comply with the limits for a Class A digital device, pursuant to part 15 of the FCC Rules. These limits are designed to provide reasonable protection against harmful interference when the equipment is operated in a commercial environment. This equipment generates, uses, and can radiate radio frequency energy and, if not installed and used in accordance with the instruction manual, may cause harmful interference to radio communications. Operation of this equipment in a residential area is likely to cause harmful interference in which case the user will be required to correct the interference at his own expense.# FCC Interference Statement for Class B EVM devices

NOTE: This equipment has been tested and found to comply with the limits for a Class B digital device, pursuant to part 15 of the FCC Rules. These limits are designed to provide reasonable protection against harmful interference in a residential installation. This equipment generates, uses and can radiate radio frequency energy and, if not installed and used in accordance with the instructions, may cause harmful interference to radio communications. However, there is no guarantee that interference will not occur in a particular installation. If this equipment does cause harmful interference to radio or television reception, which can be determined by turning the equipment off and on, the user is encouraged to try to correct the interference by one or more of the following measures:

- Reorient or relocate the receiving antenna.
- Increase the separation between the equipment and receiver.
- Connect the equipment into an outlet on a circuit different from that to which the receiver is connected.
- Consult the dealer or an experienced radio/TV technician for help.

## 3.2 Canada

### 3.2.1 For EVMs issued with an Industry Canada Certificate of Conformance to RSS-210

#### Concerning EVMs Including Radio Transmitters:

This device complies with Industry Canada license-exempt RSS standard(s). Operation is subject to the following two conditions: (1) this device may not cause interference, and (2) this device must accept any interference, including interference that may cause undesired operation of the device.

#### Concernant les EVMs avec appareils radio:

Le présent appareil est conforme aux CNR d'Industrie Canada applicables aux appareils radio exempts de licence. L'exploitation est autorisée aux deux conditions suivantes: (1) l'appareil ne doit pas produire de brouillage, et (2) l'utilisateur de l'appareil doit accepter tout brouillage radioélectrique subi, même si le brouillage est susceptible d'en compromettre le fonctionnement.

#### Concerning EVMs Including Detachable Antennas:

Under Industry Canada regulations, this radio transmitter may only operate using an antenna of a type and maximum (or lesser) gain approved for the transmitter by Industry Canada. To reduce potential radio interference to other users, the antenna type and its gain should be so chosen that the equivalent isotropically radiated power (e.i.r.p.) is not more than that necessary for successful communication. This radio transmitter has been approved by Industry Canada to operate with the antenna types listed in the user guide with the maximum permissible gain and required antenna impedance for each antenna type indicated. Antenna types not included in this list, having a gain greater than the maximum gain indicated for that type, are strictly prohibited for use with this device.

#### Concernant les EVMs avec antennes détachables

Conformément à la réglementation d'Industrie Canada, le présent émetteur radio peut fonctionner avec une antenne d'un type et d'un gain maximal (ou inférieur) approuvé pour l'émetteur par Industrie Canada. Dans le but de réduire les risques de brouillage radioélectrique à l'intention des autres utilisateurs, il faut choisir le type d'antenne et son gain de sorte que la puissance isotrope rayonnée équivalente (p.i.r.e.) ne dépasse pas l'intensité nécessaire à l'établissement d'une communication satisfaisante. Le présent émetteur radio a été approuvé par Industrie Canada pour fonctionner avec les types d'antenne énumérés dans le manuel d'usage et ayant un gain admissible maximal et l'impédance requise pour chaque type d'antenne. Les types d'antenne non inclus dans cette liste, ou dont le gain est supérieur au gain maximal indiqué, sont strictement interdits pour l'exploitation de l'émetteur

## 3.3 Japan

### 3.3.1 Notice for EVMs delivered in Japan: Please see http://www.tij.co.jp/lsds/ti_ja/general/eStore/notice_01.page

日本国内に輸入される評価用キット、ボードについては、次のところをご覧ください。

http://www.tij.co.jp/lsds/ti_ja/general/eStore/notice_01.page

### 3.3.2 Notice for Users of EVMs Considered "Radio Frequency Products" in Japan: EVMs entering Japan may not be certified by TI as conforming to Technical Regulations of Radio Law of Japan.

If User uses EVMs in Japan, not certified to Technical Regulations of Radio Law of Japan, User is required by Radio Law of Japan to follow the instructions below with respect to EVMs:

1. Use EVMs in a shielded room or any other test facility as defined in the notification #173 issued by Ministry of Internal Affairs and Communications on March 28, 2006, based on Sub-section 1.1 of Article 6 of the Ministry's Rule for Enforcement of Radio Law of Japan,
2. Use EVMs only after User obtains the license of Test Radio Station as provided in Radio Law of Japan with respect to EVMs, or
3. Use of EVMs only after User obtains the Technical Regulations Conformity Certification as provided in Radio Law of Japan with respect to EVMs. Also, do not transfer EVMs, unless User gives the same notice above to the transferee. Please note that if User does not follow the instructions above, User will be subject to penalties of Radio Law of Japan.【無線電波を送信する製品の開発キットをお使いになる際の注意事項】開発キットの中には技術基準適合証明を受けていないものがあります。技術適合証明を受けていないもののご使用に際しては、電波法遵守のため、以下のいずれかの措置を取っていただく必要がありますのでご注意ください。

1. 電波法施行規則第6条第1項第1号に基づく平成18年3月28日総務省告示第173号で定められた電波暗室等の試験設備でご使用いただく。
2. 実験局の免許を取得後ご使用いただく。
3. 技術基準適合証明を取得後ご使用いただく。

なお、本製品は、上記の「ご使用にあたっての注意」を譲渡先、移転先に通知しない限り、譲渡、移転できないものとします。
上記を遵守頂けない場合は、電波法の罰則が適用される可能性があることをご留意ください。日本テキサス・インスツルメンツ株式会社
東京都新宿区西新宿6丁目24番1号
西新宿三井ビル

3.3.3 Notice for EVMs for Power Line Communication: Please see http://www.tij.co.jp/lsds/ti_ja/general/eStore/notice_02.page
電力線搬送波通信についての開発キットをお使いになる際の注意事項については、次のところをご覧ください。http://www.tij.co.jp/lsds/ti_ja/general/eStore/notice_02.page

# 4 EVM Use Restrictions and Warnings:

4.1 EVMS ARE NOT FOR USE IN FUNCTIONAL SAFETY AND/OR SAFETY CRITICAL EVALUATIONS, INCLUDING BUT NOT LIMITED TO EVALUATIONS OF LIFE SUPPORT APPLICATIONS.
4.2 User must read and apply the user guide and other available documentation provided by TI regarding the EVM prior to handling or using the EVM, including without limitation any warning or restriction notices. The notices contain important safety information related to, for example, temperatures and voltages.

4.3 Safety-Related Warnings and Restrictions:

4.3.1 User shall operate the EVM within TI's recommended specifications and environmental considerations stated in the user guide, other available documentation provided by TI, and any other applicable requirements and employ reasonable and customary safeguards. Exceeding the specified performance ratings and specifications (including but not limited to input and output voltage, current, power, and environmental ranges) for the EVM may cause personal injury or death, or property damage. If there are questions concerning performance ratings and specifications, User should contact a TI field representative prior to connecting interface electronics including input power and intended loads. Any loads applied outside of the specified output range may also result in unintended and/or inaccurate operation and/or possible permanent damage to the EVM and/or interface electronics. Please consult the EVM user guide prior to connecting any load to the EVM output. If there is uncertainty as to the load specification, please contact a TI field representative. During normal operation, even with the inputs and outputs kept within the specified allowable ranges, some circuit components may have elevated case temperatures. These components include but are not limited to linear regulators, switching transistors, pass transistors, current sense resistors, and heat sinks, which can be identified using the information in the associated documentation. When working with the EVM, please be aware that the EVM may become very warm.

4.3.2 EVMs are intended solely for use by technically qualified, professional electronics experts who are familiar with the dangers and application risks associated with handling electrical mechanical components, systems, and subsystems. User assumes all responsibility and liability for proper and safe handling and use of the EVM by User or its employees, affiliates, contractors or designees. User assumes all responsibility and liability to ensure that any interfaces (electronic and/or mechanical) between the EVM and any human body are designed with suitable isolation and means to safely limit accessible leakage currents to minimize the risk of electrical shock hazard. User assumes all responsibility and liability for any improper or unsafe handling or use of the EVM by User or its employees, affiliates, contractors or designees.

4.4 User assumes all responsibility and liability to determine whether the EVM is subject to any applicable international, federal, state, or local laws and regulations related to User's handling and use of the EVM and, if applicable, User assumes all responsibility and liability for compliance in all respects with such laws and regulations. User assumes all responsibility and liability for proper disposal and recycling of the EVM consistent with all applicable international, federal, state, and local requirements.

5. Accuracy of Information: To the extent TI provides information on the availability and function of EVMs, TI attempts to be as accurate as possible. However, TI does not warrant the accuracy of EVM descriptions, EVM availability or other information on its websites as accurate, complete, reliable, current, or error-free.6. Disclaimers:

6.1 EXCEPT AS SET FORTH ABOVE, EVMS AND ANY WRITTEN DESIGN MATERIALS PROVIDED WITH THE EVM (AND THE DESIGN OF THE EVM ITSELF) ARE PROVIDED "AS IS" AND "WITH ALL FAULTS." TI DISCLAIMS ALL OTHER WARRANTIES, EXPRESS OR IMPLIED, REGARDING SUCH ITEMS, INCLUDING BUT NOT LIMITED TO ANY IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT OF ANY THIRD PARTY PATENTS, COPYRIGHTS, TRADE SECRETS OR OTHER INTELLECTUAL PROPERTY RIGHTS.

6.2 EXCEPT FOR THE LIMITED RIGHT TO USE THE EVM SET FORTH HEREIN, NOTHING IN THESE TERMS AND CONDITIONS SHALL BE CONSTRUED AS GRANTING OR CONFERRING ANY RIGHTS BY LICENSE, PATENT, OR ANY OTHER INDUSTRIAL OR INTELLECTUAL PROPERTY RIGHT OF TI, ITS SUPPLIERS/LICENSORS OR ANY OTHER THIRD PARTY, TO USE THE EVM IN ANY FINISHED END-USER OR READY-TO-USE FINAL PRODUCT, OR FOR ANY INVENTION, DISCOVERY OR IMPROVEMENT MADE, CONCEIVED OR ACQUIRED PRIOR TO OR AFTER DELIVERY OF THE EVM.

7. USER'S INDEMNITY OBLIGATIONS AND REPRESENTATIONS. USER WILL DEFEND, INDEMNIFY AND HOLD TI, ITS LICENSORS AND THEIR REPRESENTATIVES HARMLESS FROM AND AGAINST ANY AND ALL CLAIMS, DAMAGES, LOSSES, EXPENSES, COSTS AND LIABILITIES (COLLECTIVELY, "CLAIMS") ARISING OUT OF OR IN CONNECTION WITH ANY HANDLING OR USE OF THE EVM THAT IS NOT IN ACCORDANCE WITH THESE TERMS AND CONDITIONS. THIS OBLIGATION SHALL APPLY WHETHER CLAIMS ARISE UNDER STATUTE, REGULATION, OR THE LAW OF TORT, CONTRACT OR ANY OTHER LEGAL THEORY, AND EVEN IF THE EVM FAILS TO PERFORM AS DESCRIBED OR EXPECTED.

8. Limitations on Damages and Liability:

8.1 General Limitations. IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, COLLATERAL, INDIRECT, PUNITIVE, INCIDENTAL, CONSEQUENTIAL, OR EXEMPLARY DAMAGES IN CONNECTION WITH OR ARISING OUT OF THESE TERMS AND CONDITIONS OR THE USE OF THE EVMS PROVIDED HEREUNDER, REGARDLESS OF WHETHER TI HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES. EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF REMOVAL OR REINSTALLATION, ANCILLARY COSTS TO THE PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES, RETESTING, OUTSIDE COMPUTER TIME, LABOR COSTS, LOSS OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, LOSS OF USE, LOSS OF DATA, OR BUSINESS INTERRUPTION. NO CLAIM, SUIT OR ACTION SHALL BE BROUGHT AGAINST TI MORE THAN ONE YEAR AFTER THE RELATED CAUSE OF ACTION HAS OCCURRED.

8.2 Specific Limitations. IN NO EVENT SHALL TI'S AGGREGATE LIABILITY FROM ANY WARRANTY OR OTHER OBLIGATION ARISING OUT OF OR IN CONNECTION WITH THESE TERMS AND CONDITIONS, OR ANY USE OF ANY TI EVM PROVIDED HEREUNDER, EXCEED THE TOTAL AMOUNT PAID TO TI FOR THE PARTICULAR UNITS SOLD UNDER THESE TERMS AND CONDITIONS WITH RESPECT TO WHICH LOSSES OR DAMAGES ARE CLAIMED. THE EXISTENCE OF MORE THAN ONE CLAIM AGAINST THE PARTICULAR UNITS SOLD TO USER UNDER THESE TERMS AND CONDITIONS SHALL NOT ENLARGE OR EXTEND THIS LIMIT.

9. Return Policy. Except as otherwise provided, TI does not offer any refunds, returns, or exchanges. Furthermore, no return of EVM(s) will be accepted if the package has been opened and no return of the EVM(s) will be accepted if they are damaged or otherwise not in a resalable condition. If User feels it has been incorrectly charged for the EVM(s) it ordered or that delivery violates the applicable order, User should contact TI. All refunds will be made in full within thirty (30) working days from the return of the components(s), excluding any postage or packaging costs.

10. Governing Law: These terms and conditions shall be governed by and interpreted in accordance with the laws of the State of Texas, without reference to conflict-of-laws principles. User agrees that non-exclusive jurisdiction for any dispute arising out of or relating to these terms and conditions lies within courts located in the State of Texas and consents to venue in Dallas County, Texas. Notwithstanding the foregoing, any judgment may be enforced in any United States or foreign court, and TI may seek injunctive relief in any United States or foreign court.

Mailing Address: Texas Instruments, Post Office Box 655303, Dallas, Texas 75265
Copyright © 2015, Texas Instruments Incorporated# IMPORTANT NOTICE

Texas Instruments Incorporated and its subsidiaries (TI) reserve the right to make corrections, enhancements, improvements and other changes to its semiconductor products and services per JESD46, latest issue, and to discontinue any product or service per JESD48, latest issue. Buyers should obtain the latest relevant information before placing orders and should verify that such information is current and complete. All semiconductor products (also referred to herein as "components") are sold subject to TI's terms and conditions of sale supplied at the time of order acknowledgment.

TI warrants performance of its components to the specifications applicable at the time of sale, in accordance with the warranty in TI's terms and conditions of sale of semiconductor products. Testing and other quality control techniques are used to the extent TI deems necessary to support this warranty. Except where mandated by applicable law, testing of all parameters of each component is not necessarily performed.

TI assumes no liability for applications assistance or the design of Buyers' products. Buyers are responsible for their products and applications using TI components. To minimize the risks associated with Buyers' products and applications, Buyers should provide adequate design and operating safeguards.

TI does not warrant or represent that any license, either express or implied, is granted under any patent right, copyright, mask work right, or other intellectual property right relating to any combination, machine, or process in which TI components or services are used. Information published by TI regarding third-party products or services does not constitute a license to use such products or services or a warranty or endorsement thereof. Use of such information may require a license from a third party under the patents or other intellectual property of the third party, or a license from TI under the patents or other intellectual property of TI.

Reproduction of significant portions of TI information in TI data books or data sheets is permissible only if reproduction is without alteration and is accompanied by all associated warranties, conditions, limitations, and notices. TI is not responsible or liable for such altered documentation. Information of third parties may be subject to additional restrictions.

Resale of TI components or services with statements different from or beyond the parameters stated by TI for that component or service voids all express and any implied warranties for the associated TI component or service and is an unfair and deceptive business practice. TI is not responsible or liable for any such statements.

Buyer acknowledges and agrees that it is solely responsible for compliance with all legal, regulatory and safety-related requirements concerning its products, and any use of TI components in its applications, notwithstanding any applications-related information or support that may be provided by TI. Buyer represents and agrees that it has all the necessary expertise to create and implement safeguards which anticipate dangerous consequences of failures, monitor failures and their consequences, lessen the likelihood of failures that might cause harm and take appropriate remedial actions. Buyer will fully indemnify TI and its representatives against any damages arising out of the use of any TI components in safety-critical applications.

In some cases, TI components may be promoted specifically to facilitate safety-related applications. With such components, TI's goal is to help enable customers to design and create their own end-product solutions that meet applicable functional safety standards and requirements. Nonetheless, such components are subject to these terms.

No TI components are authorized for use in FDA Class III (or similar life-critical medical equipment) unless authorized officers of the parties have executed a special agreement specifically governing such use.

Only those TI components which TI has specifically designated as military grade or "enhanced plastic" are designed and intended for use in military/aerospace applications or environments. Buyer acknowledges and agrees that any military or aerospace use of TI components which have not been so designated is solely at the Buyer's risk, and that Buyer is solely responsible for compliance with all legal and regulatory requirements in connection with such use.

TI has specifically designated certain components as meeting ISO/TS16949 requirements, mainly for automotive use. In any case of use of non-designated products, TI will not be responsible for any failure to meet ISO/TS16949.

|  Products |  | Applications |   |
| --- | --- | --- | --- |
|  Audio | www.ti.com/audio | Automotive and Transportation | www.ti.com/automotive  |
|  Amplifiers | amplifier.ti.com | Communications and Telecom | www.ti.com/communications  |
|  Data Converters | dataconverter.ti.com | Computers and Peripherals | www.ti.com/computers  |
|  DLP® Products | www.dlp.com | Consumer Electronics | www.ti.com/consumer-apps  |
|  DSP | dsp.ti.com | Energy and Lighting | www.ti.com/energy  |
|  Clocks and Timers | www.ti.com/clocks | Industrial | www.ti.com/industrial  |
|  Interface | interface.ti.com | Medical | www.ti.com/medical  |
|  Logic | logic.ti.com | Security | www.ti.com/security  |
|  Power Mgmt | power.ti.com | Space, Avionics and Defense | www.ti.com/space-avionics-defense  |
|  Microcontrollers | microcontroller.ti.com | Video and Imaging | www.ti.com/video  |
|  RFID | www.ti-rfid.com |  |   |
|  OMAP Applications Processors | www.ti.com/omap | TI E2E Community | e2e.ti.com  |
|  Wireless Connectivity | www.ti.com/wirelessconnectivity |  |   |

