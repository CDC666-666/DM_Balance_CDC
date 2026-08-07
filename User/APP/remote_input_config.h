#ifndef REMOTE_INPUT_CONFIG_H
#define REMOTE_INPUT_CONFIG_H

/* Change only this selection to switch the active remote controller. */
#define RC_PS2                       1
#define RC_GAMESIR_NOVA_LITE        2

#define __RC_TYPE RC_GAMESIR_NOVA_LITE

#if ((__RC_TYPE != RC_PS2) && (__RC_TYPE != RC_GAMESIR_NOVA_LITE))
#error "Unsupported __RC_TYPE"
#endif

#endif
