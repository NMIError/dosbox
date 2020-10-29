/*
 *  Copyright (C) 2002-2017  The DOSBox Team
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <ieee1284.h>
#include <math.h>
#include <unistd.h>

#include "adlib.h"
#include "dosbox.h"
#include "cpu.h"
#include "opl2lpt.h"

#if C_OPL2LPT

static int opl2lpt_thread(void *ptr) {
	OPL2LPT::Handler *self = static_cast<OPL2LPT::Handler *>(ptr);
	return self->WriteThread();
}

namespace OPL2LPT {

	static const unsigned char OPL2LPTRegisterSelect[] = {
		(C1284_NSELECTIN | C1284_NSTROBE | C1284_NINIT) ^ C1284_INVERTED,
		(C1284_NSELECTIN | C1284_NSTROBE) ^ C1284_INVERTED,
		(C1284_NSELECTIN | C1284_NSTROBE | C1284_NINIT) ^ C1284_INVERTED
	};

	static const unsigned char OPL3LPTRegisterSelect[] = {
		(C1284_NSTROBE | C1284_NINIT) ^ C1284_INVERTED,
		C1284_NSTROBE ^ C1284_INVERTED,
		(C1284_NSTROBE | C1284_NINIT) ^ C1284_INVERTED
	};

	static const unsigned char OPL2LPTRegisterWrite[] = {
		(C1284_NSELECTIN | C1284_NINIT) ^ C1284_INVERTED,
		C1284_NSELECTIN ^ C1284_INVERTED,
		(C1284_NSELECTIN | C1284_NINIT) ^ C1284_INVERTED
	};

	struct parport *Handler::opl2lpt_init(std::string name) {
		struct parport_list parports = {};
		struct parport *pport;

		// Look for available parallel ports
		if (ieee1284_find_ports(&parports, 0) != E1284_OK) {
			LOG_MSG("OPL2LPT: cannot find parallel ports");
			return nullptr;
		}
		for (int i = 0; i < parports.portc; i++) {
			if (name == "" ||
				name == parports.portv[i]->name) {
				int caps = CAP1284_RAW;
				pport = parports.portv[i];
				if (ieee1284_open(pport, NULL, &caps) != E1284_OK) {
					LOG_MSG("OPL2LPT: cannot open parallel port %s", pport->name);
				}
				if (ieee1284_claim(pport) != E1284_OK) {
					LOG_MSG("OPL2LPT: cannot claim parallel port %s", pport->name);
					ieee1284_close(pport);
					continue;
				}
				opl2lpt_reset(pport);
				LOG_MSG("OPL2LPT: found parallel port %s", pport->name);
				ieee1284_free_ports(&parports);
				return pport;
			}
		}
		ieee1284_free_ports(&parports);
		LOG_MSG("OPL2LPT: cannot find parallel port %s", name.c_str());
		return nullptr;
	}

	void Handler::opl2lpt_shutdown(struct parport *pport) {
		if (pport) {
			ieee1284_close(pport);
		}
	}

	void Handler::opl2lpt_write_register(struct parport *pport, Bit16u regAddr, Bit8u value) {
		if (!pport) return;

		//LOG_MSG("OPL2LPT: writing reg %X val %X", regAddr, value);

		ieee1284_write_data(pport, regAddr);
		if (regAddr < 0x100) {
			ieee1284_write_control(pport, OPL2LPTRegisterSelect[0]);
			ieee1284_write_control(pport, OPL2LPTRegisterSelect[1]);
			ieee1284_write_control(pport, OPL2LPTRegisterSelect[2]);
		} else {
			ieee1284_write_control(pport, OPL3LPTRegisterSelect[0]);
			ieee1284_write_control(pport, OPL3LPTRegisterSelect[1]);
			ieee1284_write_control(pport, OPL3LPTRegisterSelect[2]);
		}
		// Required delay between index and data write for OPL3 is negligible
		usleep(isOpl3 ? 1 : 4);		// 3.3 us

		ieee1284_write_data(pport, value);
		ieee1284_write_control(pport, OPL2LPTRegisterWrite[0]);
		ieee1284_write_control(pport, OPL2LPTRegisterWrite[1]);
		ieee1284_write_control(pport, OPL2LPTRegisterWrite[2]);
		usleep(isOpl3 ? 1 : 23); // OPL3 requires 0.26us delay after data write; OPL2 requires 23us
	}

	void Handler::opl2lpt_reset(struct parport *pport) {
		LOG_MSG("OPL2LPT: reset OPL%d chip", isOpl3 ? 3 : 2);

		for (int i = 0; i < (isOpl3 ? 512 : 256); i++) {
			opl2lpt_write_register(pport, i, 0);
		}
	}

	void Handler::WriteReg(Bit32u reg, Bit8u val) {
#if C_DEBUG
		LOG_MSG("OPL2LPT: cycles %" PRId32 ", on reg 0x%" PRIx32 ", write 0x%" PRIx8,
			CPU_Cycles, reg, val);
#endif
		if (reg == 0xFF)
			// Write should be discarded.
			return;

		if (mode == OPL_opl2) {
			reg &= 0xFF;
		} else {
			// OPL3, OPL3GOLD, DUAL_OPL2
			reg &= 0x1FF;
		}

		if (mode == OPL_dualopl2) {
			if (reg == 0x01 || reg == 0x101) // Enable waveform selection - unecessary on OPL3

				return;

			if (reg >= 0x102 && reg <= 0x104)
				// Timer functionality - not present in register set 2
				// Redirect to register set 1 (might cause problems if a program 
				// uses both sets of timers - this is unlikely however)
				reg -= 0x100;

			if (reg == 0x108)
				// Note select / key scale keyboard split - not present in register set 2
				// Redirect to register set 1.
				// Note that OPL3 only supports 1 global setting, while 2 OPL2 chips 
				// can have 2 separate settings. The last value set on either OPL2 chip 
				// is used as the OPL3 global value.
				reg = 0x08;

			if (reg == 0xBD || reg == 0x1BD) {
				// Check if rhythm mode value has changed
				bool rhythmMode = (val & 0x20);
				bool left = reg == 0xBD;
				if (rhythmMode != (left ? rhythmModeLeft : rhythmModeRight)) {
					// Set new rhythm mode value
					if (left)
						rhythmModeLeft = rhythmMode;
					else
						rhythmModeRight = rhythmMode;

					LOG_MSG("Rhythm mode %s %sactivated", left ? "left" : "right", rhythmMode ? "" : "de");

					// Pan channels 6-8 according to which OPL2 chip(s) have rhythm mode active
					// When neither chip has rhythm mode active, pan them left (for left chip instrument use)
					c6 = (c6 & ~0x30) | ((rhythmModeLeft || !rhythmModeRight) << 4) | (rhythmModeRight << 5);
					eventQueue.push(RegisterWrite(0xC6, c6));
					c7 = (c7 & ~0x30) | ((rhythmModeLeft || !rhythmModeRight) << 4) | (rhythmModeRight << 5);
					eventQueue.push(RegisterWrite(0xC7, c7));
					c8 = (c8 & ~0x30) | ((rhythmModeLeft || !rhythmModeRight) << 4) | (rhythmModeRight << 5);
					eventQueue.push(RegisterWrite(0xC8, c8));
				}

				// Set the rhythm mode flag if one of the OPL2 chips has rhythm mode active
				val = (val & ~0x20) | ((rhythmModeLeft || rhythmModeRight) << 5);

				if ((val & 0x1F) && !(left ? rhythmModeLeft : rhythmModeRight))
					// A drum sound is triggered, but rhythm mode is not active for this OPL2 chip
					// Don't trigger the drum sound
					val |= 0xE0;

				if (reg == 0x1BD) {
					// Register BD does not exist in OPL3 register set 2.
					// Redirect this write to register set 1.
					// Note that bit 6 and 7 control vibrato and modulation depth.
					// These can only be set to 1 global value on OPL3, while they can 
					// be set separately on 2 OPL2 chips. The last value set on either OPL2 
					// chip is set as the global value on OPL3.
					reg = 0xBD;
				}
			}

			if (!rhythmModeLeft && rhythmModeRight && (
				(reg >= 0xA6 && reg <= 0xA8)
				|| (reg >= 0xB6 && reg <= 0xB8)
				|| (reg >= 0xC6 && reg <= 0xC8) // Channels 7-9
				|| (reg >= 0x30 && reg <= 0x36)
				|| (reg >= 0x50 && reg <= 0x56)
				|| (reg >= 0x70 && reg <= 0x76)
				|| (reg >= 0x90 && reg <= 0x96)
				|| (reg >= 0xF0 && reg <= 0xF6) // Operators 13-18
				)) {
				// Rhythm mode is active only on the right OPL2 chip and 
				// registers affecting channels 7-9 are written on the 
				// left OPL2 chip. These are used for the OPL3 rhythm mode.
				// Writes on these channels cannot be supported in this 
				// situation.
				return;
			}

			if (rhythmModeRight && (
				(reg >= 0x1A6 && reg <= 0x1A8)
				|| (reg >= 0x1B6 && reg <= 0x1B8)
				|| (reg >= 0x1C6 && reg <= 0x1C8) // Channels 16-18
				|| (reg >= 0x130 && reg <= 0x136)
				|| (reg >= 0x150 && reg <= 0x156)
				|| (reg >= 0x170 && reg <= 0x176)
				|| (reg >= 0x190 && reg <= 0x196)
				|| (reg >= 0x1F0 && reg <= 0x1F6) // Operators 31-36
				)) {
				// Rhythm mode is active on the right OPL2 chip and registers 
				// affecting the drum sounds are written on this chip.
				// Copy register writes to set 1 to have these writes 
				// affect the drum sounds on OPL3.
				Bit8u copiedVal = val;
				if (reg >= 0x1C6 && reg <= 0x1C8) {
					// Set panning according to rhythm mode
					copiedVal = (copiedVal & ~0x30) | ((rhythmModeLeft || !rhythmModeRight) << 4) | (rhythmModeRight << 5);
				}
				eventQueue.push(RegisterWrite(reg - 0x100, copiedVal));
			}

			if (reg >= 0xC6 && reg <= 0xC8) {
				// Set panning according to rhythm mode
				val = (val & ~0x30) | ((rhythmModeLeft || !rhythmModeRight) << 4) | (rhythmModeRight << 5);
				// Track register values
				if (reg == 0xC6)
					c6 = val;
				if (reg == 0xC7)
					c7 = val;
				if (reg == 0xC8)
					c8 = val;
			}
		}

		SDL_LockMutex(lock);
		eventQueue.push(RegisterWrite(reg, val));
		SDL_CondSignal(cond);
		SDL_UnlockMutex(lock);
	}

	Bit32u Handler::WriteAddr(Bit32u port, Bit8u val) {
#if C_DEBUG
		LOG_MSG("OPL2LPT: cycles %" PRId32 ", on port 0x%" PRIx32 ", write 0x%" PRIx8,
			CPU_Cycles, port, val);
#endif
		Bit32u retVal = val;
		if ((port & 3) != 0) {
			// Write to second register set
			if (mode == OPL_opl3 || mode == OPL_opl3gold) {
				// Indicate second register set
				retVal |= 0x100;
			} else if (mode == OPL_opl2) {
				// Discard second register set write
				retVal = 0xFF;
			}
			// Second register set for dual OPL2 is handled by Module::DualWrite
		}

		return retVal;
	}

	void Handler::Generate(MixerChannel* chan, Bitu samples) {
		/* No need to generate sound */
		chan->enabled = false;
		return;
	}

	int Handler::WriteThread() {
		struct parport *pport = opl2lpt_init(pportName);
		if (mode == OPL_dualopl2) {
			for (int i = 0; i < 9; i++) {
				opl2lpt_write_register(pport, 0xC0 + i, 0x10);
				opl2lpt_write_register(pport, 0x1C0 + i, 0x20);
			}
		}

		RegisterWrite event;

		while (true) {
			SDL_LockMutex(lock);
			while (eventQueue.empty()) {
				SDL_CondWait(cond, lock);
			}
			event = eventQueue.front();
			eventQueue.pop();
			SDL_UnlockMutex(lock);

			if (event.quit) {
				LOG_MSG("OPL2LPT: quit sound thread");
				break;
			}
			opl2lpt_write_register(pport, event.regAddr, event.value);
		}

		if (pport) {
			opl2lpt_reset(pport);
			opl2lpt_shutdown(pport);
		}
		return 0;
	}

	void Handler::Init(Bitu rate) {
		thread = SDL_CreateThread(opl2lpt_thread, this);
		if (!thread) {
			LOG_MSG("OPL2LPT: unable to create thread: %s", SDL_GetError());
		}
	}

	Handler::Handler(OPL_Mode oplMode, std::string name, bool opl3) : 
		mode(oplMode), pportName(name), isOpl3(opl3), 
		rhythmModeLeft(false), rhythmModeRight(false),
		c6(0), c7(0), c8(0),
		lock(SDL_CreateMutex()), cond(SDL_CreateCond()) {
	}

	Handler::~Handler() {
		SDL_LockMutex(lock);
		eventQueue.push(RegisterWrite(true));
		SDL_CondSignal(cond);
		SDL_UnlockMutex(lock);
		SDL_WaitThread(thread, nullptr);
	}

}; // namespace OPL2LPT

#endif // C_OPL2LPT
