# Driver\_-\_Talon-Auxiliary
Driver interface for the Aux Talon

| **Key Name** | **Definition** | **Parent Key** | **Device** | **Range, Min** | **Range, Max** | **Expected Value** |
|---|---|---|---|---|---|---|
| 0 | Analog voltage reading, instantaneous [mV] | `PORT_x` | `Talon-Aux` | 0 | 5000 | N/A |
| 1 | Analog voltage reading, average [^1] [mV] | `PORT_x` | `Talon-Aux` | 0 | 5000 | N/A |
| 2 | Pulse count since last reading [count] | `PORT_x` | `Talon-Aux` | 0 | 65536 | N/A |
| 3 | Pulse frequency over course of last reading period [Hz] | `PORT_x` | `Talon-Aux` | 0 | 65536/LogPeriod[s] | N/A |
| `START` | Unix time [s] when the associated reading period was started | `Talon-Aux` | `Talon-Aux` | N/A | N/A | N/A |
| `STOP` | Unix time [s] when the associated reading period was ended | `Talon-Aux` | `Talon-Aux` | N/A | N/A | N/A |

[^1]: 8 sample average, 128 samples per second - approximately 62.5 ms