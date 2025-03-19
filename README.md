# QShift32 - ESP32 based timing retard quickshifter module for motorcycles
This project is basically a big revision of my other project "quickshiftuino".
The differences are pretty big though, so I decided to create a new repository.

"**quickshiftuino**" had 2 channels that could cut the power to the ignition coils on the high side (P-Mos).
That topology works great, is fairly simple and reliable, but doesn't allow for more complex features.

"**QShift32**" uses all the thoroughly tested sub-circuits of "quickshiftuino" except the power switching components.
It instead offers 4 independent channels switched by IGBTs on the low side.
They can withstand high voltages and high current and are configured to pull down the low side of the ignition coil.
This can either be used to fully prevent sparks from firing or delay them by a precise time to achieve an ignition retard.

[**Demonstration Video**](https://www.youtube.com/watch?v=TwFzDqyh4EY)


R1 PCB 3D model:

<img src='images/pcb_r1.png' width='350'>

## New possibilities
- Either ignition timing retard or full spark cut to allow clutchless shifting
- Ignition timing retard will smoothly go back to normal operation after shift is done
- Smooth control over engine power transitions -> extremely smooth shifts
- RPM measurement even during power cut (not during full spark cut, only ignition retard)
- During upshift period fuel gets ignited in the exhaust instead of remaining unburnt (-> pops and flames)
- Pit limiter and launch control (RPM limiter) with spark cut and ignition retard (-> pops and flames)
- Precise dwell time and time per revolution measurement for each channel (10 µs precision)
- Expandable to support shift light, handlebar buttons, gear indicator etc. in the future

## Development progress
### 19.03.2025
- 1000 km of successful road testing done
- still working on perfecting the different configuration options
  to reduce complexity and improve shift quality
- experimented with really low and really high retard values and smoothness factors:
  low retard -> very smooth, nice shift sound that's not too loud
  high retard -> a bit less smooth, very fast shifts, loud explosions at medium throttle and high rpm
- no electronic failures
### 30.01.2025
- 300 km of successful road testing done
- no electronic failures
### 04.01.2025
- New option to do a full ignition cut instead of retarding timing
- Still smoothly brings back the engine power after shifting

  <img src='images/cut_retard_smooth_waveform.png' width='350'>

### 28.12.2024
- R1 pcb assembled
- first successful tests of retarding ignition pulses (only on the bench)

## Next steps
- Find a way of controlling multiple channels simultaneously with a single timer
- Implement rear wheel speed sensor reading (Gear indicator and automatic Launch mode exit)
- Upload Gerber files and assembly info for PCB R2
- Design a case that holds the PCB and connectors (ideally somewhat water splash proof)

The pulse measurement and IGBT switching schematic and some parts of the code were inspired by the "DIY Quickshifter" series by Gil Vargas on YouTube.
He explains all the steps in great detail.
