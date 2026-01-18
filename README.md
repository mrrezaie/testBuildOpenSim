### A custom-built OpeSim (JAM) API

Modifications:
1. COMAK with optional `use_muscle_volume` property
2. OpenSim API without ezc3d

Notes:
1. Both OpenSim and Ezc3d must have the same version; otherwise they cannot be imported simultaneously.
2. Current Ezc3d version: 1.6.3
3. If Opensim is compiled with the same Ezc3d version, Ezc3d must be imported first; otherwise it doesn't function well.
4. The easy solution is to compile OpenSim without Ezc3d (this is the case for all API only builds)
5. OpenSim GUI cannot be compiled without Ezc3d
