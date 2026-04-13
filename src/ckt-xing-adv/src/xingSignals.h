#ifndef _XING_SIGNALS_H_
#define _XING_SIGNALS_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	SIGNAL_STATE_IDLE = 0,
	SIGNAL_STATE_START_SETUP,
	SIGNAL_STATE_START,
	SIGNAL_STATE_MAIN_GATE_DOWN_SETUP,
	SIGNAL_STATE_MAIN_GATE_DOWN,
	SIGNAL_STATE_4Q_GATE_WAIT_SETUP,
	SIGNAL_STATE_4Q_GATE_WAIT,
	SIGNAL_STATE_4Q_GATE_DOWN_SETUP,
	SIGNAL_STATE_4Q_GATE_DOWN,
	SIGNAL_STATE_4Q_GATE_UP_SETUP,
	SIGNAL_STATE_4Q_GATE_UP,
	SIGNAL_STATE_MAIN_GATE_UP_SETUP,
	SIGNAL_STATE_MAIN_GATE_UP,
	SIGNAL_STATE_ACTIVE,
	SIGNAL_STATE_ACTIVE_2Q,
	SIGNAL_STATE_ACTIVE_4Q,
    SIGNAL_STATE_END
} SignalState_t;


typedef enum
{
	GATES_DISABLED            = 1,
	GATES_2Q_ONLY             = 2,
	GATES_4Q_SIMULTANEOUS_UP  = 3,
	GATES_4Q_DELAYED_UP       = 4,
    GATES_END
} GateConfiguration_t;

typedef struct
{
    SignalState_t signalState;
    uint8_t stateTimer;
    bool lightsActive;
    bool bellActive;
    bool mainGatesActive;
    bool auxGatesActive;

    GateConfiguration_t gateConfig;
    bool bellOnGateDropOnly;

} CrossingSignalState_t;

void initCrossingSignals(CrossingSignalState_t* xingState, GateConfiguration_t gateConfig, bool bellOnGateDropOnly);
void runCrossingSignalStateMachine(CrossingSignalState_t* xingState, bool active);

#endif