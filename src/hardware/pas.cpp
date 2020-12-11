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

#define DMA_BUFSIZE 1024

#define PAS_SH	14
#define PAS_SH_MASK	((1 << PAS_SH)-1)

enum PAS_TYPES { PAS_NONE, PAS_1, PAS_16 };
enum PAS_FILTER_FREQS { 
	PAS_FILTER_NONE, 
	PAS_FILTER_3KHZ, 
	PAS_FILTER_6KHZ, 
	PAS_FILTER_9KHZ, 
	PAS_FILTER_12KHZ, 
	PAS_FILTER_16KHZ,
	PAS_FILTER_18KHZ
};
enum PAS_DSP_BIT_DEPTH {
	PAS_DSP_BIT_DEPTH_8,
	PAS_DSP_BIT_DEPTH_12,
	PAS_DSP_BIT_DEPTH_16
};
enum PAS_OVERSAMPLING {
	PAS_OVERSAMPLING_0,
	PAS_OVERSAMPLING_1,
	PAS_OVERSAMPLING_2,
	PAS_OVERSAMPLING_4
};

enum PAS_INTERRUPT_TYPES {
	PAS_INTERRUPT_LEFT_FM,
	PAS_INTERRUPT_RIGHT_FM,
	PAS_INTERRUPT_SAMPLE_RATE_TIMER,
	PAS_INTERRUPT_SAMPLE_BUFFER_COUNTER,
	PAS_INTERRUPT_MIDI
};

struct PAS_INFO {
	PAS_TYPES type;
	bool enabled;
	struct {
		bool enabled;
		bool active;
		bool masked;

		bool bit16;
		Bitu rate, mul;
		Bitu min;
		union {
			Bit8u  b8[DMA_BUFSIZE];
			Bit16s b16[DMA_BUFSIZE];
		} buf;
		DmaChannel *chan;
		Bitu remain_size;
	} dma;
	struct {
		bool fm_left_pending;
		bool fm_right_pending;
		bool sample_timer_pending;
		bool sample_buffer_pending;
		bool midi_pending;

		bool fm_left_enabled;
		bool fm_right_enabled;
		bool sample_timer_enabled;
		bool sample_buffer_enabled;
		bool midi_enabled;
	} irq;
	struct {
		bool enabled;
		bool mono;
		bool dac;
		PAS_DSP_BIT_DEPTH bit_depth;

		bool right_active; // 0 if left channel or mono is active; 1 if right
		bool sample_clipping; // 1 if recording has clipped

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

		Bit8u mixer_control_last_write;

		bool righttoright;
		bool lefttoright;
		bool righttoleft;
		bool lefttoleft;
		PAS_FILTER_FREQS filter_freq;
		bool output; // true if the mixer should output audio; false if it is muted
	} mixer;
	struct {
		bool sample_rate_enabled;
		bool sample_rate_bcd_counting;
		bool sample_rate_square_wave;
		bool sample_rate_16_bit;
		bool sample_rate_flipflop;
		Bit16u sample_rate_interval; // interval between samples (1193180 / sample rate)

		bool buffer_counter_enabled;
		bool buffer_counter_bcd_counting;
		bool buffer_counter_square_wave;
		bool buffer_counter_16_bit;
		bool buffer_counter_flipflop;
		Bit16u buffer_counter_count; // number of samples? to count down

		Bit16u buffer_counter_current;
	} timer;
	struct {
		// System config 1
		bool shadow;
		bool pas1_pcm_emu;
		bool clock_28mhz;
		bool invert_com;
		bool pc_speaker_stereo;
		bool real_sound;
		bool sc1d6;
		bool master_reset;

		// System config 2
		PAS_OVERSAMPLING oversampling;
		bool msb_invert;
		Bit8u secondary_port_bits;
		bool vco_lock;

		// System config 3
		bool pcm_rate_28mhz;
		bool sb_sample_rate_timer_1mhz;
		bool invert_vco;
		bool dac_invert_bclk;
		bool left_right_sync_pulse;

		// System config 4
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

static const Bit8u dma_to_ioconfig[] = { 4, 1, 2, 3, 0, 5, 6, 7 };
static const Bit8u irq_to_ioconfig[] = { 0, 0, 1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 0, 10, 11 };
static const Bit8u ioconfig_to_irq[] = { 0xFF, 2, 3, 4, 5, 6, 7, 10, 11, 12, 14, 15 };

static PAS_INFO pas;

static void PAS_DSP_StartDMATransfer();
static void PAS_DSP_GenerateDMASound(Bitu size);
static void PAS_DSP_DMACallBack(DmaChannel *chan, DMAEvent event);
static void PAS_DSP_EndDMATransfer();

static Bitu correct_DMA_overrun(Bitu read);

static void PAS_Reset() {
	//pas.enabled = (pas.type != PAS_1); // Original PAS starts up disabled
	pas.enabled = true;

	pas.dma.enabled = false;
	pas.dma.active = false;
	pas.dma.masked = true;

	pas.dma.bit16 = false;
	pas.dma.rate = 0;
	pas.dma.mul = 0;
	pas.dma.min = 0;
	memset(pas.dma.buf.b8, 0, sizeof(pas.dma.buf.b8));
	memset(pas.dma.buf.b16, 0, sizeof(pas.dma.buf.b16));
	pas.dma.chan = NULL;
	pas.dma.remain_size = 0;

	pas.irq.fm_left_pending = false;
	pas.irq.fm_right_pending = false;
	pas.irq.sample_timer_pending = false;
	pas.irq.sample_buffer_pending = false;
	pas.irq.midi_pending = false;

	pas.irq.fm_left_enabled = false;
	pas.irq.fm_right_enabled = false;
	pas.irq.sample_timer_enabled = false;
	pas.irq.sample_buffer_enabled = false;
	pas.irq.midi_enabled = false;

	pas.dsp.enabled = false;
	pas.dsp.mono = false;
	pas.dsp.dac = false;
	pas.dsp.bit_depth = PAS_DSP_BIT_DEPTH_8;

	pas.dsp.right_active = false;
	pas.dsp.sample_clipping = false;

	pas.dsp.wait_states = 0;
	pas.dsp.oversample_prescale = 0;

	pas.mixer.mixer_control_last_write = 0;
	pas.mixer.righttoright = 0;
	pas.mixer.lefttoright = 0;
	pas.mixer.righttoleft = 0;
	pas.mixer.lefttoleft = 0;
	pas.mixer.filter_freq = PAS_FILTER_NONE;
	pas.mixer.output = false;

	pas.timer.sample_rate_enabled = false;
	pas.timer.sample_rate_bcd_counting = false;
	pas.timer.sample_rate_square_wave = false;
	pas.timer.sample_rate_16_bit = false;
	pas.timer.sample_rate_flipflop = false;
	pas.timer.sample_rate_interval = 0;

	pas.timer.buffer_counter_enabled = false;
	pas.timer.buffer_counter_bcd_counting = false;
	pas.timer.buffer_counter_square_wave = false;
	pas.timer.buffer_counter_16_bit = false;
	pas.timer.buffer_counter_flipflop = false;
	pas.timer.buffer_counter_count = 0;
	pas.timer.buffer_counter_current = 0;

	pas.sysconfig.shadow = false;
	pas.sysconfig.pas1_pcm_emu = false;
	pas.sysconfig.clock_28mhz = false;
	pas.sysconfig.invert_com = false;
	pas.sysconfig.pc_speaker_stereo = false;
	pas.sysconfig.real_sound = false;
	pas.sysconfig.sc1d6 = false;
	pas.sysconfig.master_reset = false;

	pas.sysconfig.oversampling = PAS_OVERSAMPLING_0;
	pas.sysconfig.msb_invert = false;
	pas.sysconfig.secondary_port_bits = 0;
	pas.sysconfig.vco_lock = false;

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

	pas.chan->Enable(pas.enabled && pas.mixer.output);
}

static Bitu read_pas(Bitu port, Bitu /*iolen*/) {
	Bit16u retVal = 0;

	if (port == PAS16_IO_MASTER_ADDRESS_POINTER)
		// This register is write-only
		return 0;

	Bit16u standardizedPort = port + (0x388 - pas.hw.base);
	switch (standardizedPort) {
	case PAS_IO_B2_AUDIO_MIXER_CONTROL:
		retVal = pas.mixer.mixer_control_last_write;
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (audio mixer control) value %4X", port, retVal);
		break;
	case PAS_IO_B2_INTERRUPT_STATUS:
		retVal = (pas.irq.fm_left_pending ? 1 : 0) |
			(pas.irq.fm_right_pending << 1) |
			(pas.irq.sample_timer_pending << 2) |
			(pas.irq.sample_buffer_pending << 3) |
			(pas.irq.midi_pending << 4) |
			(pas.dsp.right_active << 5) |
			(pas.enabled << 6) |
			(pas.dsp.sample_clipping << 7);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (interrupt status) value %4X", port, retVal);
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
		retVal |= (pas.mixer.output << 5) |
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
	case PAS_IO_B3_CROSS_CHANNEL_CONTROL:
		retVal = (pas.mixer.righttoright ? 1 : 0) |
			(pas.mixer.lefttoright << 1) |
			(pas.mixer.righttoleft << 2) |
			(pas.mixer.lefttoleft << 3) |
			(pas.dsp.dac << 4) |
			(pas.dsp.mono << 5) |
			(pas.dsp.enabled << 6) |
			(pas.dma.enabled << 7);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (cross channel control) value %4X", port, retVal);
		break;
	case PAS16_IO_SCSI_DETECT:
		// Detection routines use this port to check for the presence 
		// of an enhanced SCSI interface. This is used to determine the 
		// PAS type. The complete SCSI interface is not implemented.
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
	case PAS16_IO_B7_SYSTEM_CONFIG_2:
		switch (pas.sysconfig.oversampling) {
		case PAS_OVERSAMPLING_0:
			retVal = 0x0;
			break;
		case PAS_OVERSAMPLING_1:
			retVal = 0x1;
			break;
		case PAS_OVERSAMPLING_2:
			retVal = 0x2;
			break;
		case PAS_OVERSAMPLING_4:
			retVal = 0x3;
			break;
		}
		switch (pas.dsp.bit_depth) {
		case PAS_DSP_BIT_DEPTH_8:
			retVal |= (0x0 << 2);
			break;
		case PAS_DSP_BIT_DEPTH_12:
			retVal |= (0x3 << 2);
			break;
		case PAS_DSP_BIT_DEPTH_16:
			retVal |= (0x1 << 2);
			break;
		}
		retVal |= (pas.sysconfig.msb_invert << 4) |
			(pas.sysconfig.secondary_port_bits << 5) |
			(pas.sysconfig.vco_lock << 7);
		LOG(LOG_PAS, LOG_NORMAL)("Read from PAS port %4X (system config 2) value %4X", port, retVal);
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
	case PAS_IO_B3_PCM_DATA:
	case PAS_IO_B3_PCM_DATA_HIGH:
	case PAS_IO_B4_SAMPLE_RATE_TIMER:
	case PAS_IO_B4_SAMPLE_BUFFER_COUNT:
	case PAS_IO_B4_LOCAL_SPEAKER_TIMER_COUNT:
	case PAS_IO_B4_LOCAL_TIMER_CONTROL:
		// Port is write-only
		retVal = 0;
		LOG(LOG_PAS, LOG_NORMAL)("Attempt to read from write-only PAS port %4X", port, retVal);
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
			LOG(LOG_PAS, LOG_WARN)("Attempt to activate PAS16 on I/O %4X while configured as %4X", val << 2, pas.hw.base);
		return;
	}

	Bit16u standardizedPort = port + (0x388 - pas.hw.base);
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
		pas.mixer.mixer_control_last_write = val;
		break;
	case PAS_IO_B2_INTERRUPT_STATUS:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (interrupt status) value %4X", port, val);
		// A write to this registers clears any interrupts and brings the PAS out of reset.
		pas.irq.fm_left_pending = false;
		pas.irq.fm_right_pending = false;
		pas.irq.sample_timer_pending = false;
		pas.irq.sample_buffer_pending = false;
		pas.irq.midi_pending = false;
		if (pas.type == PAS_1 && !pas.enabled) {
			pas.enabled = true;
			pas.chan->Enable(pas.enabled && pas.mixer.output);
		}
		pas.dsp.sample_clipping = false;

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
		bool output;
		output = val & 0x20;
		if (output != pas.mixer.output) {
			pas.mixer.output = output;
			pas.chan->Enable(pas.enabled && pas.mixer.output);
		}

		bool sampleBufferCounterEnabled;
		sampleBufferCounterEnabled = val & 0x80;
		if (sampleBufferCounterEnabled && !pas.timer.buffer_counter_enabled) {
			pas.timer.buffer_counter_current = pas.timer.buffer_counter_count;
			pas.timer.buffer_counter_enabled = true;
		} else if (!sampleBufferCounterEnabled && pas.timer.buffer_counter_enabled) {
			pas.timer.buffer_counter_enabled = false;
		}

		bool sampleRateTimerEnabled;
		sampleRateTimerEnabled = val & 0x40;
		if (sampleRateTimerEnabled && !pas.timer.sample_rate_enabled) {
			// Start DMA transfer
			pas.timer.sample_rate_enabled = true;
			if (pas.irq.sample_timer_enabled)
				LOG(LOG_PAS, LOG_WARN)("Sample timer enabled with interrupt (not implemented)!");
			PAS_DSP_StartDMATransfer();
		} else if (!sampleRateTimerEnabled && pas.timer.sample_rate_enabled) {
			// Stop DMA transfer
			PAS_DSP_EndDMATransfer();
			pas.dma.active = false;
			pas.timer.sample_rate_enabled = false;
		}
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
		pas.dsp.mono = val & 0x20;

		bool dspEnabled;
		dspEnabled = val & 0x40;

		if (dspEnabled && !pas.dsp.enabled) {
			pas.dsp.enabled = true;
			PAS_DSP_StartDMATransfer();
		} else if (!dspEnabled && pas.dsp.enabled) {
			// Disable PCM
			PAS_DSP_EndDMATransfer();
			pas.dma.active = false;
			pas.timer.sample_rate_enabled = false;
			pas.timer.buffer_counter_enabled = false;
			pas.dsp.enabled = false;
		}

		bool dmaEnabled;
		dmaEnabled = val & 0x80;

		if (dmaEnabled && !pas.dma.enabled) {
			// Activate DMA
			pas.dma.chan = GetDMAChannel(pas.hw.dma);
			pas.dma.bit16 = (pas.dma.chan->DMA16 > 0);
			pas.dma.enabled = true;
			PAS_DSP_StartDMATransfer();
		} else if (!dmaEnabled && pas.dma.enabled) {
			// Deactivate DMA
			PAS_DSP_EndDMATransfer();
			pas.dma.chan->Register_Callback(NULL);
			pas.dma.chan = NULL;
			pas.dma.active = false;
			pas.dma.enabled = false;
		}

		break;
	case PAS_IO_B4_SAMPLE_RATE_TIMER:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (sample rate timer) value %4X", port, val);
		if (pas.timer.sample_rate_16_bit && pas.timer.sample_rate_flipflop) {
			pas.timer.sample_rate_interval |= (val << 8);
			pas.timer.sample_rate_flipflop = false;
		} else {
			pas.timer.sample_rate_interval = val;
			if (pas.timer.sample_rate_16_bit)
				pas.timer.sample_rate_flipflop = true;
		}
		break;
	case PAS_IO_B4_SAMPLE_BUFFER_COUNT:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (sample buffer count) value %4X", port, val);
		if (pas.timer.buffer_counter_16_bit && pas.timer.buffer_counter_flipflop) {
			pas.timer.buffer_counter_count |= (val << 8);
			pas.timer.buffer_counter_flipflop = false;
			pas.timer.buffer_counter_current = pas.timer.buffer_counter_count;
		} else {
			pas.timer.buffer_counter_count = val;
			if (pas.timer.buffer_counter_16_bit) {
				pas.timer.buffer_counter_flipflop = true;
			} else {
				pas.timer.buffer_counter_current = pas.timer.buffer_counter_count;
			}
		}
		break;
	case PAS_IO_B4_LOCAL_SPEAKER_TIMER_COUNT:
		LOG(LOG_PAS, LOG_NORMAL)("Unhandled write to PAS port %4X (speaker timer count) value %4X", port, val);
		break;
	case PAS_IO_B4_LOCAL_TIMER_CONTROL:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (local timer control) value %4X", port, val);
		bool sampleBufferCounter;
		switch ((val & 0xC0) >> 6) {
		case 0x1:
			sampleBufferCounter = true;
			break;
		case 0x0: // sample rate timer
			sampleBufferCounter = false;
			break;
		case 0x2: // local speaker timer (not supported)
		default:
			LOG(LOG_PAS, LOG_WARN)("Attempt to set unsupported timer type %X", (val & 0xC0) >> 6);
			return;
		}
		if (sampleBufferCounter) {
			pas.timer.buffer_counter_bcd_counting = val & 0x1;
			if (pas.timer.buffer_counter_bcd_counting)
				LOG(LOG_PAS, LOG_WARN)("Attempt to use sample buffer counter with BCD counting (not implemented)");
			pas.timer.buffer_counter_square_wave = (val & 0xE) >> 1 == 0x3;
			if (pas.timer.buffer_counter_square_wave)
				LOG(LOG_PAS, LOG_WARN)("Attempt to use sample buffer counter with square wave generator (not implemented)");
			pas.timer.buffer_counter_16_bit = (val & 0x30) >> 4 == 0x3;
			pas.timer.buffer_counter_flipflop = false;
		} else {
			pas.timer.sample_rate_bcd_counting = val & 0x1;
			if (pas.timer.sample_rate_bcd_counting)
				LOG(LOG_PAS, LOG_WARN)("Attempt to use sample rate timer with BCD counting (not implemented)");
			pas.timer.sample_rate_square_wave = (val & 0xE) >> 1 == 0x3;
			if (!pas.timer.sample_rate_square_wave)
				LOG(LOG_PAS, LOG_WARN)("Attempt to use sample rate timer with rate generator (not implemented)");
			pas.timer.sample_rate_16_bit = (val & 0x30) >> 4 == 0x3;
			pas.timer.sample_rate_flipflop = false;
		}
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
	case PAS16_IO_B7_SYSTEM_CONFIG_2:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (system config 2) value %4X", port, val);
		switch (val & 0x3) {
		case 0x0:
			pas.sysconfig.oversampling = PAS_OVERSAMPLING_0;
			break;
		case 0x1:
			pas.sysconfig.oversampling = PAS_OVERSAMPLING_1;
			break;
		case 0x2:
			pas.sysconfig.oversampling = PAS_OVERSAMPLING_2;
			break;
		case 0x3:
			pas.sysconfig.oversampling = PAS_OVERSAMPLING_4;
			break;
		}
		switch ((val & 0xC) >> 2) {
		case 0x0:
		case 0x2: // both 12 bit and 16 bit must be set for 12 bit audio
			pas.dsp.bit_depth = PAS_DSP_BIT_DEPTH_8;
			break;
		case 0x1:
			pas.dsp.bit_depth = PAS_DSP_BIT_DEPTH_16;
			break;
		case 0x3:
			pas.dsp.bit_depth = PAS_DSP_BIT_DEPTH_12;
			break;
		}
		pas.sysconfig.msb_invert = val & 0x10;
		pas.sysconfig.secondary_port_bits = (val & 0x60) >> 5;
		pas.sysconfig.vco_lock = val & 0x80;
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
			LOG(LOG_PAS, LOG_WARN)("Attempt to set PAS16 DMA to %d while configured as %d", dma_to_ioconfig[val & 0x7], pas.hw.dma);
		break;
	case PAS16_IO_IO_CONFIG_3:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (I/O config 3) value %4X", port, val);
		if (pas.hw.irq != ioconfig_to_irq[val & 0xF])
			LOG(LOG_PAS, LOG_WARN)("Attempt to set PAS16 IRQ to %d while configured as %d", ioconfig_to_irq[val & 0xF], pas.hw.irq);
		break;
	case PAS16_IO_COMPATIBILITY_REGISTER_ENABLE:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (compatibility register enable) value %4X", port, val);
		if (val & 0x1)
			LOG(LOG_PAS, LOG_WARN)("Attempt to enable PAS16 MPU-401 emulation");
		if (val & 0x2)
			LOG(LOG_PAS, LOG_WARN)("Attempt to enable PAS16 SoundBlaster emulation");
		break;
	case PAS16_IO_EMULATION_ADDRESS_POINTER:
		LOG(LOG_PAS, LOG_NORMAL)("Write to PAS port %4X (emulation address pointer) value %4X", port, val);
		if (val & 0xF)
			LOG(LOG_PAS, LOG_WARN)("Attempt to set PAS16 SoundBlaster emulation address pointer to %1X", val);
		if (val & 0xF0)
			LOG(LOG_PAS, LOG_WARN)("Attempt to set PAS16 MPU-401 emulation address pointer to %1X", val >> 4);
		break;
	default:
		LOG(LOG_PAS, LOG_NORMAL)("Unhandled write to PAS port %4X value %4X", port, val);
		break;
	}
}

static INLINE void PAS_RaiseIRQ(PAS_INTERRUPT_TYPES interruptType) {
	bool raiseIrq = true;
	switch (interruptType) {
	case PAS_INTERRUPT_LEFT_FM:
		raiseIrq = (pas.irq.fm_left_enabled && !pas.irq.fm_left_pending);
		if (raiseIrq) {
			LOG(LOG_PAS, LOG_NORMAL)("Raising FM left IRQ");
			pas.irq.fm_left_pending = true;
		}
		break;
	case PAS_INTERRUPT_RIGHT_FM:
		raiseIrq = (pas.irq.fm_right_enabled && !pas.irq.fm_right_pending);
		if (raiseIrq) {
			LOG(LOG_PAS, LOG_NORMAL)("Raising FM right IRQ");
			pas.irq.fm_right_pending = true;
		}
		break;
	case PAS_INTERRUPT_SAMPLE_RATE_TIMER:
		raiseIrq = (pas.irq.sample_timer_enabled && !pas.irq.sample_timer_pending);
		if (raiseIrq) {
			LOG(LOG_PAS, LOG_NORMAL)("Raising sample rate timer IRQ");
			pas.irq.sample_timer_pending = true;
		}
		break;
	case PAS_INTERRUPT_SAMPLE_BUFFER_COUNTER:
		raiseIrq = (pas.irq.sample_buffer_enabled && !pas.irq.sample_buffer_pending);
		if (raiseIrq) {
			LOG(LOG_PAS, LOG_NORMAL)("Raising sample buffer counter IRQ");
			pas.irq.sample_buffer_pending = true;
		}
		break;
	case PAS_INTERRUPT_MIDI:
		raiseIrq = (pas.irq.midi_enabled && !pas.irq.midi_pending);
		if (raiseIrq) {
			LOG(LOG_PAS, LOG_NORMAL)("Raising MIDI IRQ");
			pas.irq.midi_pending = true;
		}
		break;
	}
	if (raiseIrq) {
		PIC_ActivateIRQ(pas.hw.irq);
	} else {
		LOG(LOG_PAS, LOG_NORMAL)("Not raising requested IRQ type %d", interruptType);
	}
}

static void PAS_DSP_StartDMATransfer() {
	if (!pas.dsp.enabled || !pas.timer.sample_rate_enabled || 
		!pas.dma.enabled || pas.dma.active)
		return;

	pas.chan->FillUp();

	switch (pas.dsp.bit_depth) {
	case PAS_DSP_BIT_DEPTH_8:
		pas.dma.mul = (1 << PAS_SH);
		break;
	case PAS_DSP_BIT_DEPTH_12:
		LOG(LOG_PAS, LOG_WARN)("Attempt to start DMA transfer with 12 bit audio (not implemented)");
		pas.dma.mul = (1 << PAS_SH) + (1 << (PAS_SH - 1)); // * 1.5
		break;
	case PAS_DSP_BIT_DEPTH_16:
		pas.dma.mul = (1 << PAS_SH) * 2;
		break;
	}

	// Interval for stereo is set to half the mono interval,
	// so the calculated frequency is double the actual frequency.
	Bitu freq = 1193180 / (pas.timer.sample_rate_interval * (!pas.dsp.mono ? 2 : 1));
	if (!pas.dsp.mono) {
		pas.dma.mul *= 2;
	}
	pas.dma.rate = (freq * pas.dma.mul) >> PAS_SH;
	pas.dma.min = (pas.dma.rate * 3) / 1000;
	pas.dma.remain_size = 0;
	pas.chan->SetFreq(freq);

	pas.dma.chan->Register_Callback(PAS_DSP_DMACallBack);

	pas.dma.active = true;

	if (pas.timer.buffer_counter_enabled && (pas.timer.buffer_counter_count < pas.dma.min)) {
		float delay = (pas.timer.buffer_counter_count * 1000.0f) / pas.dma.rate;
		LOG(LOG_PAS, LOG_NORMAL)("Short transfer scheduling IRQ in %.3f milliseconds", delay);
		PIC_AddEvent(PAS_DSP_GenerateDMASound, delay, pas.timer.buffer_counter_count);
	}

	LOG(LOG_PAS, LOG_NORMAL)("Started DMA transfer. autoinit: %d, frequency: %d, bitdepth: %d, stereo: %d", 
		pas.dma.chan->autoinit ? 1 : 0, freq, pas.dsp.bit_depth, pas.dsp.mono ? 0 : 1);
}

static double pas_last_dma_callback = 0.0f;

static void PAS_DSP_DMACallBack(DmaChannel *chan, DMAEvent event) {
	if (chan != pas.dma.chan) return;
	switch (event) {
	case DMA_MASKED:
		LOG(LOG_PAS, LOG_NORMAL)("DMA masked, stopping output, left %d", chan->currcnt);
		PAS_DSP_EndDMATransfer();
		pas.dma.masked = true;
		break;
	case DMA_UNMASKED:
		LOG(LOG_PAS, LOG_NORMAL)("DMA unmasked");
		pas.dma.masked = false;
		break;
	case DMA_REACHED_TC:
		// No need to do anything.
		break;
	default:
		E_Exit("Unknown pas dma event");
	}
}

static void PAS_DSP_EndDMATransfer() {
	if (pas.timer.sample_rate_enabled && pas.dsp.enabled && 
		pas.dma.enabled && pas.dma.active && !pas.dma.masked) {
		double t = PIC_FullIndex() - pas_last_dma_callback;
		LOG(LOG_PAS, LOG_NORMAL)("PAS_DSP_EndDMATransfer: time passed since last DMA callback: %f", t);
		Bitu s = static_cast<Bitu>(pas.dma.rate * t / 1000.0f);
		if (s > pas.dma.min) {
			LOG(LOG_PAS, LOG_NORMAL)("PAS_DSP_EndDMATransfer: limiting DMA end amount to pas.dma.min");
			s = pas.dma.min;
		}
		if (s) {
			LOG(LOG_PAS, LOG_NORMAL)("PAS_DSP_EndDMATransfer: generating %d bytes", s);
			PAS_DSP_GenerateDMASound(s);
		}
	}
}

static void PAS_DSP_GenerateDMASound(Bitu size) {
	Bitu read = 0; Bitu done = 0; Bitu i = 0;
	pas_last_dma_callback = PIC_FullIndex();

	//Determine how much you should read
	if (pas.dma.masked || pas.dma.chan->autoinit) {
		if (pas.timer.buffer_counter_enabled && (pas.timer.buffer_counter_current < size)) {
			LOG(LOG_PAS, LOG_NORMAL)("Limiting generated sound to buffer counter value %d", pas.timer.buffer_counter_current);
			size = pas.timer.buffer_counter_current;
		}
	} else {
		if (pas.timer.buffer_counter_enabled && (pas.timer.buffer_counter_current < pas.dma.min)) {
			LOG(LOG_PAS, LOG_NORMAL)("Setting generated sound to current buffer counter value %d (below min value)", pas.timer.buffer_counter_current);
			size = pas.timer.buffer_counter_current;
		}
	}

	if (pas.dma.masked) {
		pas.chan->AddSilence();
		read = size;
	} else {
		//Read the actual data, process it and send it off to the mixer
		if (pas.dsp.bit_depth == PAS_DSP_BIT_DEPTH_8) {
			if (!pas.dsp.mono) {
				read = pas.dma.chan->Read(size, &pas.dma.buf.b8[pas.dma.remain_size]);
				if (read != size) {
					LOG(LOG_PAS, LOG_NORMAL)("Requested %d DMA bytes, got %d", size, read);
				}
				Bitu used = correct_DMA_overrun(read);
				Bitu total = used * (pas.dma.bit16 ? 2 : 1) + pas.dma.remain_size;
				pas.chan->AddSamples_s8(total >> 1, pas.dma.buf.b8);
				if (total & 1) {
					pas.dma.remain_size = 1;
					pas.dma.buf.b8[0] = pas.dma.buf.b8[total - 1];
				} else pas.dma.remain_size = 0;
			} else {
				read = pas.dma.chan->Read(size, pas.dma.buf.b8);
				if (read != size) {
					LOG(LOG_PAS, LOG_NORMAL)("Requested %d DMA bytes, got %d", size, read);
				}
				Bitu used = correct_DMA_overrun(read);
				pas.chan->AddSamples_m8(used * (pas.dma.bit16 ? 2 : 1), pas.dma.buf.b8);
			}
		} else if (pas.dsp.bit_depth == PAS_DSP_BIT_DEPTH_12) {
			// TODO Implement this
			pas.chan->AddSilence();
			read = size;
		} else {
			if (!pas.dsp.mono) {
				read = pas.dma.chan->Read(size, (Bit8u *)&pas.dma.buf.b16[pas.dma.remain_size]);
				if (read != size) {
					LOG(LOG_PAS, LOG_NORMAL)("Requested %d DMA bytes, got %d", size, read);
				}
				Bitu used = correct_DMA_overrun(read);
				Bitu total = used / (pas.dma.bit16 ? 1 : 2) + pas.dma.remain_size;
#if defined(WORDS_BIGENDIAN)
				pas.chan->AddSamples_s16_nonnative(total >> 1, pas.dma.buf.b16);
#else
				pas.chan->AddSamples_s16(total >> 1, (Bit16s *)pas.dma.buf.b16);
#endif
				if (total & 1) {
					pas.dma.remain_size = 1;
					pas.dma.buf.b16[0] = pas.dma.buf.b16[total - 1];
				} else pas.dma.remain_size = 0;
			} else {
				read = pas.dma.chan->Read(size, (Bit8u *)pas.dma.buf.b16);
				if (read != size) {
					LOG(LOG_PAS, LOG_NORMAL)("Requested %d DMA bytes, got %d", size, read);
				}
				Bitu used = correct_DMA_overrun(read);
#if defined(WORDS_BIGENDIAN)
				pas.chan->AddSamples_m16_nonnative(used / (pas.dma.bit16 ? 1 : 2), pas.dma.buf.b16);
#else
				pas.chan->AddSamples_m16(used / (pas.dma.bit16 ? 1 : 2), (Bit16s *)pas.dma.buf.b16);
#endif
			}
		}
	}

	//Check how many bytes were actually read
	if (pas.timer.buffer_counter_enabled) {
		pas.timer.buffer_counter_current -= 
			((pas.timer.buffer_counter_current < read) ? pas.timer.buffer_counter_current : read);
		if (pas.timer.buffer_counter_current == 0) {
			PAS_RaiseIRQ(PAS_INTERRUPT_SAMPLE_BUFFER_COUNTER);
			pas.timer.buffer_counter_current = pas.timer.buffer_counter_count;
		}
	}
}

static Bitu correct_DMA_overrun(Bitu read) {
	if (pas.dma.masked) {
		// Channel got masked due to the DMA buffer being empty. For some reason 
		// it seems an extra "trash" byte is read in this case. These must be 
		// removed, otherwise they will cause clicks in the audio. (PQ3)
		// Possibly this is a Sierra sound driver bug, not a PAS issue...
		if (read > 0) {
			read--;
		}
		LOG(LOG_PAS, LOG_NORMAL)("DMA got masked; using only %d bytes", read);
	}
	return read;
}

static void PAS_CallBack(Bitu len) {
	if (pas.timer.sample_rate_enabled && pas.dsp.enabled && 
		pas.dma.enabled && pas.dma.active) {
		len *= pas.dma.mul;
		if (len & PAS_SH_MASK) len += 1 << PAS_SH;
		len >>= PAS_SH;
		PAS_DSP_GenerateDMASound(len);
	}
	else {
		pas.chan->AddSilence();
	}
}

static void PAS_OPLIRQCallback(bool right) {
	PAS_RaiseIRQ(right ? PAS_INTERRUPT_RIGHT_FM : PAS_INTERRUPT_LEFT_FM);
}

class PAS : public Module_base {
private:
	/* Data */
	IO_ReadHandleObject ReadHandler[13];
	IO_WriteHandleObject WriteHandler[13];
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
			case PAS_1:
				oplmode = OPL_dualopl2;
				break;
			case PAS_16:
				oplmode = OPL_opl3;
				break;
			case PAS_NONE:
			default:
				oplmode = OPL_none;
				break;
			}
		}

		if (oplmode == OPL_none) {
			//WriteHandler[0].Install(0x388, adlib_gusforward, IO_MB);
		} else {
			pas.oplModule = OPL_Init(section, oplmode, &PAS_OPLIRQCallback);
		}

		if (pas.type == PAS_NONE) return;

		pas.chan = MixerChan.Install(&PAS_CallBack, 22050, "PAS");
		pas.dma.chan = NULL;

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
		if (pas.type != PAS_NONE) {
			PIC_DeActivateIRQ(pas.hw.irq);
			PIC_RemoveEvents(PAS_DSP_GenerateDMASound);
			if (pas.dma.chan) {
				pas.dma.chan->Clear_Request();
				pas.dma.chan->Register_Callback(NULL);
			}
		}
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