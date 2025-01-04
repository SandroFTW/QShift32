# QShift32 - ESP32 based timing retard quickshifter module for motorcycles
This project is basically a big revision of my other project "quickshiftuino".
The differences are pretty big though, so I decided to create a new repository.

"quickshiftuino" had 2 channels that could cut the power to the ignition coils on the high side (P-Mos).
That topology works great, is fairly simple and reliable, but doesn't allow for more complex features.

"QShift32" uses all the thoroughly tested sub-circuits of "quickshiftuino" except the power switching components.
It instead offers 4 independent channels switched by IGBTs on the low side.
They can withstand high voltages and high current and are configured to pull down the low side of the ignition coil.
This can either be used to fully prevent sparks from firing or delay them by a precise time to achieve an ignition retard.

<ins>Demonstration Video (YouTube, click image):</ins>

[![Demonstration video](https://img.youtube.com/vi/TwFzDqyh4EY/0.jpg)](https://www.youtube.com/watch?v=TwFzDqyh4EY)

<ins>R1 PCB 3D model</ins>

<img src='images/pcb_r1.png' width='350'>

## New possibilities
- RPM measurement even during power cut (not during spark cut, only ignition retard)
- Either ignition timing retard or full spark cut to allow clutchless shifting
- Smooth control over engine power transitions -> extremely smooth shifts
- Ignition timing retard will smoothly go back to normal operation after shift is done
- During upshift period fuel gets ignited in the exhaust instead of remaining unburnt (-> pops and flames)
- Pit limiter and launch control possible in the future
- Precise dwell time measurement for each channel

## Development progress
### 28.12.2024
- R1 pcb assembled
- first successful tests of retarding ignition pulses (only on the bench)
### 04.01.2025
- New option to do a full ignition cut instead of retarding timing
- Still smoothly brings back the engine power after shifting
<img src='images/cut_retard_smooth_waveform.png' width='350'>

## Next steps
- Find a way of controlling multiple channels simultaneously with a single timer
- Gain more road experience for different settings
- Fix a small error on R1 pcb so R2 can be released

The pulse measurement and IGBT switching schematic and some parts of the code were inspired by the "DIY Quickshifter" series by Gil Vargas on YouTube.
He explains all the steps in great detail.
