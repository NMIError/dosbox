/*
 *  Copyright (C) 2002-2020  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <string.h>

#include "dosbox.h"
#include "inout.h"
#include "mixer.h"
#include "dma.h"
#include "hardware.h"
#include "setup.h"
#include "support.h"
#include "logging.h"
#include "adlib.h"

// Block 0 (0x388): FM

// Block 1 (0x788): Dual FM, parallel mixer
#define PAS_IO_B1_PARALLEL_INTERFACE_AUDIO_MIXER_1 0x78A
#define PAS_IO_B1_PARALLEL_INTERFACE_AUDIO_MIXER_2 0x78B

// Block 2 (0xB88): mixer / interrupts / filter
#define PAS_IO_B2_AUDIO_MIXER_CONTROL 0xB88
#define PAS_IO_B2_INTERRUPT_STATUS 0xB89
#define PAS_IO_B2_AUDIO_FILTER_CONTROL 0xB8A
#define PAS_IO_B2_INTERRUPT_CONTROL 0xB8B

// Block 3 (0xF88): PCM data / cross channel / reserved
#define PAS_IO_B3_PCM_DATA 0xF88
#define PAS_IO_B3_PCM_DATA_HIGH 0xF89
#define PAS_IO_B3_CROSS_CHANNEL_CONTROL 0xF8A
#define PAS_IO_B3_RESERVED_5 0xF8B

// Block 4 (0x1388): timers / PCM buffer count
#define PAS_IO_B4_SAMPLE_RATE_TIMER 0x1388
#define PAS_IO_B4_SAMPLE_BUFFER_COUNT 0x1389
#define PAS_IO_B4_LOCAL_SPEAKER_TIMER_COUNT 0x138A
#define PAS_IO_B4_LOCAL_TIMER_CONTROL 0x138B

// Block 5 (0x1788): MIDI
// Block 6 (0x1B88): MIDI

// PAS16 specific

#define PAS16_IO_LEFT_VU_READ 0x2388
#define PAS16_IO_RIGHT_VU_READ 0x2389

#define PAS16_IO_CDROM_TIMEOUT_COUNTER 0x4388
#define PAS16_IO_CDROM_TIMEOUT_STATUS 0x4389

#define PAS16_IO_SCSI_1 0x7F88
#define PAS16_IO_SCSI_DETECT 0x7F89
#define PAS16_IO_SCSI_3 0x7F8A
#define PAS16_IO_SCSI_4 0x7F8B

#define PAS16_IO_B7_SYSTEM_CONFIG_1 0x8388
#define PAS16_IO_B7_SYSTEM_CONFIG_2 0x8389
#define PAS16_IO_B7_SYSTEM_CONFIG_3 0x838A
#define PAS16_IO_B7_SYSTEM_CONFIG_4 0x838B

#define PAS16_IO_MASTER_ADDRESS_POINTER 0x9A01

#define PAS16_IO_WAIT_STATE 0xBF88
#define PAS16_IO_ANALOG_CHIP_SERIAL_DATA 0xBF89
#define PAS16_IO_OVERSAMPLE_PRESCALE 0xBF8A

#define PAS16_IO_SECONDARY_PUSH_BUTTON 0xE388
#define PAS16_IO_AUX_INTERRUPT_STATUS 0xE38A
#define PAS16_IO_AUX_INTERRUPT_ENABLE 0xE38B

#define PAS16_IO_SECONDARY_UP_DATA 0xEB88
#define PAS16_IO_SECONDARY_UP_COMMAND 0xEB89
#define PAS16_IO_SECONDARY_UP_RESTART 0xEB8A

#define PAS16_IO_SECONDARY_CHIP_REVISION 0xEF88
#define PAS16_IO_SECONDARY_READ 0xEF8B

#define PAS16_IO_IO_CONFIG_1 0xF388
#define PAS16_IO_IO_CONFIG_2 0xF389
#define PAS16_IO_IO_CONFIG_3 0xF38A
#define PAS16_IO_IO_CONFIG_4 0xF38B

#define PAS16_IO_COMPATIBILITY_REGISTER_ENABLE 0xF788
#define PAS16_IO_EMULATION_ADDRESS_POINTER 0xF789

#define PAS16_IO_PRIMARY_UP_DATA 0xFB88
#define PAS16_IO_PRIMARY_UP_COMMAND 0xFB89
#define PAS16_IO_PRIMARY_UP_RESTART 0xFB8A
#define PAS16_IO_INTERRUPT_WATCH_DOG 0xFB8B

#define PAS16_IO_PRIMARY_CHIP_REVISION 0xFF88
#define PAS16_IO_PRIMARY_READ 0xFF8B

enum PAS_TYPES { PAS_NONE, PAS_1, PAS_16 };
enum PAS_FILTER_FREQS { 
	PAS_FILTER_NONE, 
	PAS_FILTER_3KHZ, 
	PAS_FILTER_6KHZ, 
	PAS_FILTER_9KHZ, 
	PAS_FILTER_12KHZ, 
	PAS_FILTER_16KHZ, // "15.909 kHz"; actually 14.909?
	PAS_FILTER_18KHZ
};

struct PAS_INFO {
	PAS_TYPES type;
	bool enabled;
	struct {
		bool enabled;
	} dma;
	struct {
		bool pending;
		bool fm_left_enabled;
		bool fm_right_enabled;
		bool sample_timer_enabled;
		bool sample_buffer_enabled;
		bool midi_enabled;
		Bit8u interrupt_control_upper_bits;
	} irq;
	struct {
		bool enabled;
		bool stereo;
		bool dac;

		Bit8u wait_states;
		Bit8u oversample_prescale;
	} dsp;
	struct {
		Bitu base;
		Bitu irq;
		Bit8u dma;
	} hw;
	struct {
		bool enabled;
		bool fmMono;
		bool righttoright;
		bool lefttoright;
		bool righttoleft;
		bool lefttoleft;
		PAS_FILTER_FREQS filter_freq;
	} mixer;
	struct {
		bool sample_rate_enabled;
		bool buffer_counter_enabled;
	} timer;
	struct {
		bool shadow;
		bool pas1_pcm_emu;
		bool clock_28mhz;
		bool invert_com;
		bool pc_speaker_stereo;
		bool real_sound;
		bool sc1d6;
		bool master_reset;

		bool pcm_rate_28mhz;
		bool sb_sample_rate_timer_1mhz;
		bool invert_vco;
		bool dac_invert_bclk;
		bool left_right_sync_pulse;

		bool cd_dma_active_high;
		bool cd_dma_ack_active_high;
		bool cd_irq_active_high;
		bool cd_dma_valid;
		bool com_interrupt_enabled;
		bool scsi_interrupt_enabled;
		Bit8u dma_timing_pointer;
	} sysconfig;
	struct {
		bool ps2_enabled;
		Bit8u com_decode_pointer;
		Bit8u com_interrupt_pointer;
		bool joystick_enabled;
		bool warm_boot_reset_enabled;
	} ioconfig;
	MixerChannel *chan;
	Adlib::Module *oplModule;
};

static Bit8u dma_to_ioconfig[] = { 4, 1, 2, 3, 0, 5, 6, 7 };
static Bit8u irq_to_ioconfig[] = { 0, 0, 1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 0, 10, 11 };
static Bit8u ioconfig_to_irq[] = { 0xFF, 2, 3, 4, 5, 6, 7, 10, 11, 12, 14, 15 };

static PAS_INFO pas;

static void PAS_Reset() {
	pas.enabled = false;

	pas.dma.enabled = false;

	pas.irq.fm_left_enabled = false;
	pas.irq.fm_right_enabled = false;
	pas.irq.sample_timer_enabled = false;
	pas.irq.sample_buffer_enabled = false;
	pas.irq.midi_enabled = false;

	pas.dsp.enabled = false;
	pas.dsp.stereo = false;
	pas.dsp.dac = false;
	pas.dsp.wait_states = 0;
	pas.dsp.oversample_prescale = 0;

	pas.mixer.fmMono = false;
	pas.mixer.righttoright = 0;
	pas.mixer.lefttoright = 0;
	pas.mixer.righttoleft = 0;
	pas.mixer.lefttoleft = 0;
	pas.mixer.filter_freq = PAS_FILTER_NONE;

	pas.timer.sample_rate_enabled = false;
	pas.timer.buffer_counter_enabled = false;

	pas.sysconfig.shadow = false;
	pas.sysconfig.pas1_pcm_emu = false;
	pas.sysconfig.clock_28mhz = false;
	pas.sysconfig.invert_com = false;
	pas.sysconfig.pc_speaker_stereo = false;
	pas.sysconfig.real_sound = false;
	pas.sysconfig.sc1d6 = false;
	pas.sysconfig.master_reset = false;

	pas.sysconfig.pcm_rate_28mhz = false;
	pas.sysconfig.sb_sample_rate_timer_1mhz = false;
	pas.sysconfig.invert_vco = false;
	pas.sysconfig.dac_invert_bclk = false;
	pas.sysconfig.left_right_sync_pulse = false;

	pas.sysconfig.cd_dma_active_high = false;
	pas.sysconfig.cd_dma_ack_active_high = false;
	pas.sysconfig.cd_irq_active_high = false;
	pas.sysconfig.cd_dma_valid = false;
	pas.sysconfig.com_interrupt_enabled = false;
	pas.sysconfig.scsi_interrupt_enabled = false;
	pas.sysconfig.dma_timing_pointer = 0;

	pas.ioconfig.ps2_enabled = false;
	pas.ioconfig.com_decode_pointer = 0;
	pas.ioconfig.com_interrupt_pointer = 0;
	pas.ioconfig.joystick_enabled = false;
	pas.ioconfig.warm_boot_reset_enabled = false;
}

static Bitu read_pas(Bitu port, Bitu /*iolen*/) {
	Bit16u retVal = 0;

	if (port == PAS16_IO_MASTER_ADDRESS_POINTER)
		// This register is write-only
		return 0;

	Bit16u standardizedPort = port + (pas.hw.base - 0x388);
	switch (standardizedPort) {
	case PAS_IO_B2_AUDIO_MIXER_CONTROL:
		retVal = 0;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (audio mixer control) value %4X", port, retVal);
		break;
	case PAS_IO_B2_AUDIO_FILTER_CONTROL:
		switch (pas.mixer.filter_freq) {
		case PAS_FILTER_18KHZ:
			retVal = 0x01;
			break;
		case PAS_FILTER_16KHZ:
			retVal = 0x02;
			break;
		case PAS_FILTER_12KHZ:
			retVal = 0x09;
			break;
		case PAS_FILTER_9KHZ:
			retVal = 0x11;
			break;
		case PAS_FILTER_6KHZ:
			retVal = 0x19;
			break;
		case PAS_FILTER_3KHZ:
			retVal = 0x04;
			break;
		case PAS_FILTER_NONE:
			retVal = 0x00;
			break;
		}
		retVal |= (pas.enabled << 5) |
			(pas.timer.sample_rate_enabled << 6) |
			(pas.timer.buffer_counter_enabled << 7);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (audio filter control) value %4X", port, retVal);
		break;
	case PAS_IO_B2_INTERRUPT_CONTROL:
		retVal = (pas.irq.fm_left_enabled ? 1 : 0) |
			(pas.irq.fm_right_enabled << 1) |
			(pas.irq.sample_timer_enabled << 2) |
			(pas.irq.sample_buffer_enabled << 3) |
			(pas.irq.midi_enabled << 4);
		if (pas.type == PAS_16)
			// Upper 3 bits are the board revision.
			// This must be 0 for original PAS.
			// For PAS16 this should apparently be 1 or 7.
			retVal |= 0x20;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (interrupt control) value %4X", port, retVal);
		break;
	case PAS16_IO_SCSI_DETECT:
		// Detection routines use this port to check for the presence 
		// of an enhanced SCSI device. This is used to determine the 
		// PAS model. The complete SCSI interface is not implemented.
		retVal = 0;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (SCSI detect) value %4X", port, retVal);
		break;
	case PAS16_IO_B7_SYSTEM_CONFIG_1:
		retVal = (pas.sysconfig.shadow ? 1 : 0) |
			(pas.sysconfig.pas1_pcm_emu << 1) |
			(pas.sysconfig.clock_28mhz << 2) |
			(pas.sysconfig.invert_com << 3) |
			(pas.sysconfig.pc_speaker_stereo << 4) |
			(pas.sysconfig.real_sound << 5) |
			(pas.sysconfig.sc1d6 << 6) |
			(pas.sysconfig.master_reset << 7);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (system config 1) value %4X", port, retVal);
		break;
	case PAS16_IO_B7_SYSTEM_CONFIG_3:
		retVal = (pas.sysconfig.pcm_rate_28mhz ? 1 : 0) |
			(pas.sysconfig.sb_sample_rate_timer_1mhz << 1) |
			(pas.sysconfig.invert_vco << 2) |
			(pas.sysconfig.dac_invert_bclk << 3) |
			(pas.sysconfig.left_right_sync_pulse << 4);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (system config 3) value %4X", port, retVal);
		break;
	case PAS16_IO_B7_SYSTEM_CONFIG_4:
		retVal = (pas.sysconfig.cd_dma_active_high ? 1 : 0) |
			(pas.sysconfig.cd_dma_ack_active_high << 1) |
			(pas.sysconfig.cd_irq_active_high << 2) |
			(pas.sysconfig.cd_dma_valid << 3) |
			(pas.sysconfig.com_interrupt_enabled << 4) |
			(pas.sysconfig.scsi_interrupt_enabled << 5) |
			(pas.sysconfig.dma_timing_pointer << 6);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (system config 4) value %4X", port, retVal);
		break;
	case PAS16_IO_WAIT_STATE:
		retVal = pas.dsp.wait_states;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (wait state) value %4X", port, retVal);
		break;
	case PAS16_IO_OVERSAMPLE_PRESCALE:
		retVal = pas.dsp.oversample_prescale;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (oversample prescale) value %4X", port, retVal);
		break;
	case PAS16_IO_IO_CONFIG_1:
		retVal = (pas.ioconfig.ps2_enabled ? 1 : 0) |
			(pas.ioconfig.com_decode_pointer << 1) |
			(pas.ioconfig.com_interrupt_pointer << 3) |
			(pas.ioconfig.joystick_enabled << 6) |
			(pas.ioconfig.warm_boot_reset_enabled << 7);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (I/O config 1) value %4X", port, retVal);
		break;
	case PAS16_IO_IO_CONFIG_2:
		retVal = dma_to_ioconfig[pas.hw.dma];
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (I/O config 2) value %4X", port, retVal);
		break;
	case PAS16_IO_IO_CONFIG_3:
		retVal = irq_to_ioconfig[pas.hw.irq];
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (I/O config 3) value %4X", port, retVal);
		break;
	case PAS16_IO_COMPATIBILITY_REGISTER_ENABLE:
		// PAS16 SoundBlaster and MPU-401 emulation is not implemented
		retVal = 0;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (compatibility register enable) value %4X", port, retVal);
		break;
	case PAS16_IO_EMULATION_ADDRESS_POINTER:
		// PAS16 SoundBlaster and MPU-401 emulation is not implemented
		retVal = 0;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (emulation address pointer) value %4X", port, retVal);
		break;
	case PAS16_IO_PRIMARY_CHIP_REVISION:
		retVal = 0x03; // rev C or later for PAS16
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (primary chip revision) value %4X", port, retVal);
		break;
	case PAS16_IO_PRIMARY_READ:
		// Bit 0: AT/PS2 bus (1 - AT)
		// Bit 1: Timer emulation (0 - disabled)
		// Bit 2: Primary/secondary (0 - primary)
		// Bit 3: Secondary present (0 - no)
		// Bit 4: XT/AT timing (1 - AT)
		// Bit 5-7: chip revision (3 - rev C)
		retVal = 0x71;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (primary read) value %4X", port, retVal);
		break;
	case PAS16_IO_SECONDARY_READ:
		// Bit 0-1: drive interface (3 - enhanced SCSI)
		// Bit 2: FM chip type (1 - OPL3)
		// Bit 3: DAC type (1 - 16 bit)
		// Bit 4: use internal MIDI (0 - no)
		// Bit 7: auto-repeating switch (0 - no)
		retVal = 0x0F;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (secondary read) value %4X", port, retVal);
		break;
	case PAS_IO_B3_CROSS_CHANNEL_CONTROL:
		// Port is write-only
		LOG(LOG_PAS, LOG_NORMAL)("Attempt to read from write-only PAS port %4X", port, retVal);
		retVal = 0;
		break;
	default:
		LOG(LOG_PAS, LOG_NORMAL)("Unhandled read from PAS port %4X", port);
		break;
	}
	return retVal;
}

static void write_pas(Bitu port, Bitu val, Bitu /*iolen*/) {
	if (port == PAS16_IO_MASTER_ADDRESS_POINTER) {
		if (val >= 0xBC && val <= 0xBF)
			// This activates one of four installed PAS boards.
			return;
		if (val << 2 != pas.hw.base)
			LOG_MSG("Attempt to activate PAS16 on I/O %4X while configured as %4X", val << 2, pas.hw.base);
		return;
	}

	Bit16u standardizedPort = port + (pas.hw.base - 0x388);
	switch (standardizedPort) {
	case PAS_IO_B2_AUDIO_MIXER_CONTROL:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (audio mixer control) value %4X", port, val);
		if (pas.type == PAS_1) {
			Adlib::Module::pasFmMono = ((val & 0x80) > 0);
		} else if (pas.type == PAS_16) {
			if (val & 0x1) {
				// OPL3 reset
			}
			if (val & 0x2) {
				// DAC reset
			}
			if (val & 0x4) {
				// Emulated SB reset (not implemented)
			}
			if (val & 0x10) {
				// MVA508 mixer reset
			}
		}
		break;
	case PAS_IO_B2_AUDIO_FILTER_CONTROL:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (audio filter control) value %4X", port, val);
		switch (val & 0x1F) {
		case 0x01:
			pas.mixer.filter_freq = PAS_FILTER_18KHZ;
			break;
		case 0x02:
			pas.mixer.filter_freq = PAS_FILTER_16KHZ;
			break;
		case 0x09:
			pas.mixer.filter_freq = PAS_FILTER_12KHZ;
			break;
		case 0x11:
			pas.mixer.filter_freq = PAS_FILTER_9KHZ;
			break;
		case 0x19:
			pas.mixer.filter_freq = PAS_FILTER_6KHZ;
			break;
		case 0x04:
			pas.mixer.filter_freq = PAS_FILTER_3KHZ;
			break;
		case 0x00:
		default:
			pas.mixer.filter_freq = PAS_FILTER_NONE;
			break;
		}
		pas.enabled = val & 0x20;
		pas.timer.sample_rate_enabled = val & 0x40;
		pas.timer.buffer_counter_enabled = val & 0x80;
		break;
	case PAS_IO_B2_INTERRUPT_CONTROL:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (interrupt control) value %4X", port, val);
		pas.irq.fm_left_enabled = val & 0x1;
		pas.irq.fm_right_enabled = val & 0x2;
		pas.irq.sample_timer_enabled = val & 0x4;
		pas.irq.sample_buffer_enabled = val & 0x8;
		pas.irq.midi_enabled = val & 0x10;
		break;
	case PAS_IO_B3_CROSS_CHANNEL_CONTROL:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (cross channel control) value %4X", port, val);
		pas.mixer.righttoright = val & 0x1;
		pas.mixer.lefttoright = val & 0x2;
		pas.mixer.righttoleft = val & 0x4;
		pas.mixer.lefttoleft = val & 0x8;
		pas.dsp.dac = val & 0x10;
		pas.dsp.stereo = val & 0x20;
		pas.dsp.enabled = val & 0x40;
		pas.dma.enabled = val & 0x80;
		break;
	case PAS16_IO_B7_SYSTEM_CONFIG_1:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (system config 1) value %4X", port, val);
		pas.sysconfig.shadow = val & 0x1;
		pas.sysconfig.pas1_pcm_emu = val & 0x2;
		pas.sysconfig.clock_28mhz = val & 0x4;
		pas.sysconfig.invert_com = val & 0x8;
		pas.sysconfig.pc_speaker_stereo = val & 0x10;
		pas.sysconfig.real_sound = val & 0x20;
		pas.sysconfig.sc1d6 = val & 0x40;
		pas.sysconfig.master_reset = val & 0x80;
		break;
	case PAS16_IO_B7_SYSTEM_CONFIG_3:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (system config 3) value %4X", port, val);
		pas.sysconfig.pcm_rate_28mhz = val & 0x1;
		pas.sysconfig.sb_sample_rate_timer_1mhz = val & 0x2;
		pas.sysconfig.invert_vco = val & 0x4;
		pas.sysconfig.dac_invert_bclk = val & 0x8;
		pas.sysconfig.left_right_sync_pulse = val & 0x10;
		break;
	case PAS16_IO_B7_SYSTEM_CONFIG_4:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (system config 4) value %4X", port, val);
		pas.sysconfig.cd_dma_active_high = val & 0x1;
		pas.sysconfig.cd_dma_ack_active_high = val & 0x2;
		pas.sysconfig.cd_irq_active_high = val & 0x4;
		pas.sysconfig.cd_dma_valid = val & 0x8;
		pas.sysconfig.com_interrupt_enabled = val & 0x10;
		pas.sysconfig.scsi_interrupt_enabled = val & 0x20;
		pas.sysconfig.dma_timing_pointer = val & 0xC0 >> 6;
		break;
	case PAS16_IO_WAIT_STATE:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (wait state) value %4X", port, val);
		pas.dsp.wait_states = val;
		break;
	case PAS16_IO_OVERSAMPLE_PRESCALE:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (oversample prescale) value %4X", port, val);
		pas.dsp.oversample_prescale = val;
		break;
	case PAS16_IO_IO_CONFIG_1:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (I/O config 1) value %4X", port, val);
		pas.ioconfig.ps2_enabled = val & 0x1;
		pas.ioconfig.com_decode_pointer = val & 0x6 >> 1;
		pas.ioconfig.com_interrupt_pointer = val & 0x38 >> 3;
		pas.ioconfig.joystick_enabled = val & 0x40;
		pas.ioconfig.warm_boot_reset_enabled = val & 0x80;
		break;
	case PAS16_IO_IO_CONFIG_2:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (I/O config 2) value %4X", port, val);
		if (pas.hw.dma != dma_to_ioconfig[val & 0x7])
			LOG_MSG("Attempt to set PAS16 DMA to %d while configured as %d", dma_to_ioconfig[val & 0x7], pas.hw.dma);
		break;
	case PAS16_IO_IO_CONFIG_3:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (I/O config 3) value %4X", port, val);
		if (pas.hw.irq != ioconfig_to_irq[val & 0xF])
			LOG_MSG("Attempt to set PAS16 IRQ to %d while configured as %d", ioconfig_to_irq[val & 0xF], pas.hw.irq);
		break;
	case PAS16_IO_COMPATIBILITY_REGISTER_ENABLE:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (compatibility register enable) value %4X", port, val);
		if (val & 0x1)
			LOG_MSG("Attempt to enable PAS16 MPU-401 emulation");
		if (val & 0x2)
			LOG_MSG("Attempt to enable PAS16 SoundBlaster emulation");
		break;
	case PAS16_IO_EMULATION_ADDRESS_POINTER:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (emulation address pointer) value %4X", port, val);
		if (val & 0xF)
			LOG_MSG("Attempt to set PAS16 SoundBlaster emulation address pointer to %1X", val);
		if (val & 0xF0)
			LOG_MSG("Attempt to set PAS16 MPU-401 emulation address pointer to %1X", val >> 4);
		break;
	default:
		LOG(LOG_PAS, LOG_NORMAL)("Unhandled write to PAS port %4X value %4X", port, val);
		break;
	}
}

static void PAS_CallBack(Bitu len) {
	// TODO Implement this
}

class PAS : public Module_base {
private:
	/* Data */
	IO_ReadHandleObject ReadHandler[13];
	IO_WriteHandleObject WriteHandler[13];
	//AutoexecObject autoexecline;
	MixerObject MixerChan;
	OPL_Mode oplmode;

public:
	PAS(Section *configuration) : Module_base(configuration) {
		Section_prop *section = static_cast<Section_prop *>(configuration);

		pas.hw.base = section->Get_hex("pasbase");
		pas.hw.irq = section->Get_int("pasirq");
		Bitu dma = section->Get_int("pasdma");
		if (dma > 0xff) dma = 0xff;
		pas.hw.dma = (Bit8u)(dma & 0xff);

		pas.mixer.enabled = section->Get_bool("pasmixer");

		const char *pastype = section->Get_string("pastype");
		if (!strcasecmp(pastype, "pas")) pas.type = PAS_1;
		else if (!strcasecmp(pastype, "pas16")) pas.type = PAS_16;
		else pas.type = PAS_NONE;

		if (pas.type == PAS_16 && (!IS_EGAVGA_ARCH || !SecondDMAControllerAvailable()))
			pas.type = PAS_1;

		const char *omode = section->Get_string("pasoplmode");
		if (!strcasecmp(omode, "none")) oplmode = OPL_none;
		else if (!strcasecmp(omode, "dualopl2")) oplmode = OPL_dualopl2;
		else if (!strcasecmp(omode, "opl3")) oplmode = OPL_opl3;
		/* Else assume auto */
		else {
			switch (pas.type) {
			case PAS_NONE:
				oplmode = OPL_none;
				break;
			case PAS_1:
				oplmode = OPL_dualopl2;
				break;
			case PAS_16:
				oplmode = OPL_opl3;
				break;
			}
		}

		if (oplmode == OPL_none) {
			//WriteHandler[0].Install(0x388, adlib_gusforward, IO_MB);
		} else {
			pas.oplModule = OPL_Init(section, oplmode);
		}

		if (pas.type == PAS_NONE) return;

		pas.chan = MixerChan.Install(&PAS_CallBack, 22050, "PAS");

		for (Bit8u i = 0; i < (pas.type == PAS_1 ? 3 : 11); i++) {
			Bit16u port;
			switch (i) {
			case 0:
				port = PAS_IO_B2_AUDIO_MIXER_CONTROL;
				break;
			case 1:
				port = PAS_IO_B3_PCM_DATA;
				break;
			case 2:
				port = PAS_IO_B4_SAMPLE_RATE_TIMER;
				break;
			case 3:
				port = PAS16_IO_SCSI_1;
				break;
			case 4:
				port = PAS16_IO_B7_SYSTEM_CONFIG_1;
				break;
			case 5:
				port = PAS16_IO_WAIT_STATE;
				break;
			case 6:
				port = PAS16_IO_IO_CONFIG_1;
				break;
			case 7:
				port = PAS16_IO_COMPATIBILITY_REGISTER_ENABLE;
				break;
			case 8:
				port = PAS16_IO_PRIMARY_UP_DATA;
				break;
			case 9:
				port = PAS16_IO_PRIMARY_CHIP_REVISION;
				break;
			case 10:
				port = PAS16_IO_SECONDARY_CHIP_REVISION;
				break;
			}

			port += (pas.hw.base - 0x388);

			ReadHandler[i].Install(port, read_pas, IO_MB, 4);
			WriteHandler[i].Install(port, write_pas, IO_MB, 4);
		}

		ReadHandler[pas.type == PAS_1 ? 3 : 11].Install(PAS_IO_B1_PARALLEL_INTERFACE_AUDIO_MIXER_1 + (pas.hw.base - 0x388), read_pas, pas.type == PAS_1 ? IO_MB : (IO_MB | IO_MW), 2);
		WriteHandler[pas.type == PAS_1 ? 3 : 11].Install(PAS_IO_B1_PARALLEL_INTERFACE_AUDIO_MIXER_1 + (pas.hw.base - 0x388), write_pas, pas.type == PAS_1 ? IO_MB : (IO_MB | IO_MW), 2);
		if (pas.type == PAS_16) {
			ReadHandler[12].Install(PAS16_IO_MASTER_ADDRESS_POINTER, read_pas, IO_MB);
			WriteHandler[12].Install(PAS16_IO_MASTER_ADDRESS_POINTER, write_pas, IO_MB);
		}

		PAS_Reset();
	}

	~PAS() {
		if (oplmode != OPL_none)
			OPL_ShutDown(m_configuration);
		if (pas.type != PAS_NONE)
			// TODO Shut this thing down
			;
	}
};

static PAS *pasInstance;
void PAS_ShutDown(Section * /*sec*/) {
	delete pasInstance;
}

void PAS_Init(Section *sec) {
	pasInstance = new PAS(sec);
	sec->AddDestroyFunction(&PAS_ShutDown, true);
}