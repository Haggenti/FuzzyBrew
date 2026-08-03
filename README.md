# FuzzyBrew - Brewing Temperature Control System

An intelligent temperature control system for beer brewing, based on Arduino with fuzzy logic and complete user interface.

## System Specifications
- Originally designed for 50L BIAB (Brew In A Bag) setup
- Compatible with other volumes (20-100L recommended)
- Heating element control only
- Pump control not included (manual operation required)
- Tested with 3500W heating element

## ⚠️ Safety Warning

This project involves:
- High voltage (220V/110V) connections
- Heating elements
- Liquids near electronics
- Food safety considerations

Required safety measures:
- All electrical work must be done by qualified personnel
- Use proper electrical isolation and grounding
- Install adequate circuit breakers
- Use waterproof enclosures (minimum IP54)
- Install emergency stop button
- Keep electronics and high voltage separated
- Use food-grade temperature probe
- Regular safety checks of all components

## Disclaimer

This project is provided "as is", without warranty of any kind, express or implied. By using this system, you agree that the author shall not be liable for any damages, including but not limited to direct, indirect, special, incidental or consequential damages or losses arising out of the use or inability to use this system.

The user assumes all responsibility for:
- Proper installation and wiring
- Safe operation
- Regular maintenance
- Food safety compliance
- Compliance with local regulations

## Features

### Temperature Control
- Fuzzy logic control with 4 distinct zones
- Logarithmic power curve for maintenance
- Real-time LCD 20x4 display
- TM1637 LED display for current temperature
- Visual heating indication via RGB LED (red)
- Probe error protection

### Operating Modes
- **Manual Mode**: Direct temperature control with fine adjustment
- **Auto Mode**: Automated brewing steps sequence
- **Preset Mode**: Quick temperature presets selection
- **Standby Mode**: Temperature monitoring without heating

## Calibration script
The repository includes `calibrate.py`, which can fit several models to your measured temperature/PWM data, choose the best curve automatically, and show fit quality metrics like RMSE and R². Run it interactively or with `python calibrate.py --file measurements.csv`.