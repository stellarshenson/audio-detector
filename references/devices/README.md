# devices

A device reference is one markdown file per device that states what the device is, what it speaks and how a host program drives it through the device bridge. One uniform format documents the device side and the host side of every device.

- **File** - `references/devices/<device>.md`, `<device>` in lowercase words joined by hyphens, one file per device
- **Datasheets** - the datasheets a reference cites go to `references/datasheets/`, named `<part>-<document>.pdf`
- **Procedure** - the device bridge agent skill `jupyterlab-device-bridge-extension` holds the steps to research and write a reference, in its `references/device-reference.md`
- **Example** - the device bridge agent skill `jupyterlab-device-bridge-extension` carries the worked example of the format, in its `references/devices/turmo-usb-monitor.md`

## Format

Every reference has these nine sections, as the headings `## 1. Identity` to `## 9. Sources`, in this order. A section with nothing to state says `None known`.

| Section | Side | Holds |
| --- | --- | --- |
| Identity | device | What the device is and how to recognise it in `jupyterlab-device list`: VID:PID, product, manufacturer and serial strings |
| Connection | bridge | The path under `/run/devices`, the serial settings and the action on open, or the USB interface and endpoints |
| Device protocol | device | What the device speaks: commands, bytes in hex, replies, timing, what the device acknowledges |
| Device code | device | The code that runs on the device: the firmware files of the project and what each implements, or the vendor firmware that is not available |
| Host protocol | host | How the host program drives the device through the bridge: sequence, framing, timing, recovery |
| Host code | host | The host program, as code blocks or as file paths in the project, with the commands that run it |
| Traps and recovery | both | Each failure, its effect and the recovery step |
| Observed and untested | both | The facts observed on the unit, each with the date and the evidence; every other statement marked as from a source or untested |
| Sources | both | The URLs, and the datasheets in `references/datasheets/`, each cited also in the section that uses it |

- **Facts** - only behaviour observed on the unit is stated as fact; a statement from a document names its source, any other statement says untested
- **Bytes** - in hex, lowercase, one space between bytes: `45 45 45 45 45 45`
