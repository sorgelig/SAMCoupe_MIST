# [SAM Coupe](https://en.wikipedia.org/wiki/SAM_Coup%C3%A9) for [MIST Board](https://github.com/mist-devel/mist-board/wiki)

### Features:
- Fully functional SAM Coupe with precise CPU and Video timings.
- Real CPU frequency for ZX mode and full speed (6MHz) for other modes.
- 512KB of original internal memory.
- Emulation for 4MB of extended memory.
- Two disk drives.
- Support for disk formats: EDSK, MGT, IMG.
- Write support (Drive 1).
- Original SAM joysticks (same as Sinclair 1 and 2).
- Kempston joystick (useful for some ZX games).
- SAA1099 sound chip.
- SID MOS6581 sound chip.
- Stereo SAM DAC on LPT1.
- Mouse.


### Installation:
Copy the *.rbf file at the root of the SD card. You can rename the file to core.rbf if you want the MiST to load it automatically at startup.
Copy [samcoupe.rom](https://github.com/sorgelig/SAMCoupe_MIST/tree/master/releases) file to the root of SD card.

For PAL mode (RGBS output) you need to put [mist.ini](https://github.com/sorgelig/ZX_Spectrum-128K_MIST/tree/master/releases/mist.ini) file to the root of SD card. Set the option **scandoubler_disable** for desired video output.

### Notes about supported formats:
**MGT** is simple sector dump of SAM disks. All disks have the same size 819200 (for 80 track disks).

**IMG** is the same as **MGT** but uses different layout. It's used on +D FDC add-on (ZX Spectrum).

**EDSK** is format for non-standard disks. Sizes up to 1024kb are supported.
There is only basic support for EDSK format. If application has strong copy protection then it may not work.

**MGT and EDSK formats may have common file extension - DSK. IMG format should use extension IMG for correct detection**.

Other formats like **SAD,SDF,TD0, etc.** can be converted to one of supported format by [SAMdisk](http://simonowen.com/samdisk/) utility.

Write feature isn't mature yet. Make backup of important disks before write to them.

**MiST firmware must be updated to release 2016-06-19 or later for correct disk functionality!**

Core provides autostart for newly inserted disk into drive 1. Autostart works only on startup screen with stripes. Press any key after restart before loading the disk if you don't want autostart.

### Keyboard:
Most PC keys are mapped to the same SAM Coupe keys.
**F1-F10** mapped to **F1-F0**, thus reduced keyboards like Logitech K400r can be used. **Alt** is mapped to **Symbol shift**. **Left Shift** is mapped to **Shift**, **Right Shift** doesn't map to SAM Coupe key but used as modifier for unfitted keys to represent original SAM Coupe keys. For example RShift + 6...0 prodice keys as written on PC keyboards. Suitable for other close to RShift PC keys. **RShift+Ctrl** switches to layout more suitable for ZX (where cursor keys are mapped to CAPS 5-8) although most SAM Coupe ZX emulators already provide good mapping and thus usually you don't need to switch layout.

* F11 - NMI key
* Ctrl-F11 - reset.
* Alt-F11 - reset and unload disk images.
* F12 - show OSD

### Other info:
**CPU Throttle** option allow to turn on/off CPU slowdown. Originally SAM Coupe's CPU doesn't work at full speed when accessing ports and internal memory. If you switch it off then CPU will work at full speed. ZX mode will work at 6MHz as well if this option is off.

**ZX mode speed** allows to switch between original SAM Coupe's ZX speed emulation through contention and real 3.5MHz frequency. It's useful for ZX import where beeper used for sound. In emulation mode the sound is garbled in these games while real frequency provides clean sound. (Arkanoid 48k is a good example).

**External RAM** enables/disables support for additional 4MB of RAM. Disabling external RAM reduces startup time of some apps. Some apps can gain advantages if this option is enabled. Actual change of this option happens upon reset.

### Download precompiled binaries and system ROMs:
Go to [releases](https://github.com/sorgelig/SAMCoupe_MIST/tree/master/releases) folder.
