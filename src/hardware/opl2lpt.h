/* -*- c++ -*- */
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

#if C_OPL2LPT

#include <queue>

#include "SDL_thread.h"
#include "adlib.h"
#include "dosbox.h"

namespace OPL2LPT {

	struct RegisterWrite {
		bool quit;
		Bit16u regAddr;
		Bit8u value;

		RegisterWrite() : quit(false), regAddr(0), value(0) { };
		RegisterWrite(bool quit) : quit(quit), regAddr(0), value(0) { };
		RegisterWrite(Bit16u regAddr, Bit8u value) : quit(false), regAddr(regAddr), value(value) { };
	};

	struct Handler : public Adlib::Handler {
	private:
		OPL_Mode mode; // The OPL configuration for which the data sent to the handler is intended
		std::string pportName;
		bool isOpl3; // OPL2 or OPL3 chip on LPT device?
		/* Thread management for OPL2LPT */
		SDL_Thread *thread;
		SDL_mutex *lock;
		SDL_cond *cond;
		std::queue<RegisterWrite> eventQueue;

		bool rhythmModeLeft;
		bool rhythmModeRight;
		Bit8u c6, c7, c8;

		struct parport *opl2lpt_init(std::string name);
		void opl2lpt_shutdown(struct parport *pport);
		void opl2lpt_write_register(struct parport *pport, Bit16u regAddr, Bit8u value);
		void opl2lpt_reset(struct parport *pport);

	public:
		virtual Bit32u WriteAddr( Bit32u port, Bit8u val );
		virtual void WriteReg( Bit32u addr, Bit8u val );
		virtual void Generate( MixerChannel* chan, Bitu samples );
		virtual void Init( Bitu rate );
		int WriteThread();
		explicit Handler(OPL_Mode oplmode, std::string name, bool opl3);
		~Handler();
	};

};		//Namespace

#endif
