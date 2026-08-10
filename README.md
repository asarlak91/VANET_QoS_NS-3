# IQDN VANET Simulation Components (NS-3.26)

C++/NS-3 simulation components associated with the paper:

**“An Approach to Improve the Quality of Service in DTN and Non-DTN based VANET.”**
https://jist.ir/en/Article/15660

This repository contains cleaned and revalidated versions of two simulation components originally developed for the study:

- **LTE / Delay-Intolerant Data (DID)**
- **IEEE 802.11p / Delay-Tolerant Data (DTD)**

The code was tested successfully with **NS-3.26**.

## Components

### LTE DID
`src/lte-did-component.cc`

- 80 UEs (4 groups * 20)
- 1 eNodeB
- 160-byte UDP packets every 20 ms
- Vehicle speed: 20 m/s

### IEEE 802.11p DTD
`src/ieee80211p-dtd-component.cc`

- 40 mobile nodes
- IEEE 802.11p, 6 Mbps PHY mode
- 1000-byte packets at 1 Mbps
- Vehicle speed: 20 m/s

Both simulations use NS-3 `FlowMonitor` to measure packet loss, delay, jitter, and throughput.

## Tested Environment

- NS-3 3.26
- Ubuntu 24.04.1 LTS
- GCC/G++ 13.3.0
- Python 3.10.20

See `docs/tested-environment.txt` for setup and compatibility details.

## Note
These files contain the available LTE and IEEE 802.11p simulation components; the complete integrated SDN controller and handoff implementation described in the paper is not included.

## Running the Simulations

Place the source files in the `scratch/` directory of NS-3.26 and run:

```bash
python ./waf --run scratch/lte-did-component
python ./waf --run scratch/ieee80211p-dtd-component
