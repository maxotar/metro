# Metro - Vibrating Metronome Wearable

A compact wearable metronome device that provides haptic feedback for keeping time. Perfect for musicians, dancers, and other performers who need portable tempo guidance.

## Project Metadata

- **Project Name:** Metro
- **Type:** Wearable Device
- **Primary Function:** Vibrating Metronome
- **Target MCU:** ATtiny1616
- **CPU Frequency:** 4 MHz
- **Battery:** 302025 LiPo
- **Charging:** Smart ring magnetic pogo pin connector

## Project Structure

### `/electrical`
PCB design files, schematics, and component information
- KiCad project files (.kicad_sch, .kicad_pcb)
- Component library with footprints and datasheets
- Fabrication and production files (BOM, designators, netlist)

### `/src`
Firmware for the ATtiny1616 microcontroller
- `main.cpp` - Main program entry point
- `i2c.cpp` - I2C communication library

### `/mechanical`
Enclosure and mechanical design
- `Case.step` - 3D model of the device case
- [Onshape CAD](https://cad.onshape.com/documents/80829a8af30c2a989428d6ff/w/00026b4066893ecc4d2da063/e/4930d11ba89da00962c1ae8d?renderMode=0&uiState=69bccf33683487e6bda76e25) - Full source CAD file

**3D Printing Specifications:**
- Printer: Bambu Lab A1
- Material: Matte white PLA
- Bed: Smooth PEI surface
- Nozzle: 0.2 mm
- Layer height: 0.06 mm (high quality)
- Print method: Parts printed individually (case, back cap, buttons) for quality assurance
