To ensure the FUSB302B receive FIFO (RxFIFO) fills up with data when a USB-C PD-capable device is plugged in, the device must be configured to power its internal decoding blocks, detect an attachment, and enable the reception of specific packet types.

### 1. Power Up Essential Analog Blocks
The FUSB302B receiver requires several internal blocks to be active to decode Baseband Mark Coded (BMC) signals. You must configure the **Power (0x0B)** register to enable the following,:
*   **Internal Oscillator (`PWR`)**: Required to clock the BMC receiver,.
*   **Measure Block (`PWR`)**: Required for CC line voltage monitoring,.
*   **Receiver and Current References (`PWR`)**: Powers the analog receiver,.
*   **Bandgap and Wake Circuit (`PWR`)**: Essential for initial detection,.
*   **Setting:** Write **`0x0F`** to the **Power (0x0B)** register to turn all blocks on.

### 2. Enable Attachment Detection (Autonomous Toggle)
The FUSB302B will not fill the RxFIFO unless it detects a valid attachment and determines its role (Source or Sink). The most efficient way is to enable **Autonomous DRP Toggle**,:
*   **Control2 (0x08)**: Set the **`TOGGLE`** bit (Bit 0) to **1**,.
*   **Mode Setting**: Set **`MODE[1:0]`** (Bits 2:1) in **Control2** to **`01b`** for Dual-Role Port (DRP) polling,.
*   **Host Current**: Configure **Control0 (0x06)** bits **`HOST_CUR[1:0]`** to **`01b`** (80 $\mu$A default) to provide the initial pull-up current if acting as a source,.

### 3. Configure CC Line Switches
The device must be "listening" on the correct Configuration Channel (CC) pin. In autonomous mode, the FUSB302B handles these internally, but for sniffing or manual control, you use the **Switches0 (0x02)** register,:
*   **Measurement**: Ensure **`MEAS_CC1`** (Bit 2) and **`MEAS_CC2`** (Bit 3) are active so the comparators can monitor activity,.
*   **Sniffing Note**: For passive sniffing, ensure all pull-ups (**`PU_EN1/2`**) and pull-downs (**`PDWN1/2`**) are disabled to avoid interfering with the existing connection,,.

### 4. Enable SOP* Packet Reception
The FUSB302B ignores all PD packets by default unless specifically told which types to accept. Use the **Control1 (0x07)** register to enable packet types,:
*   **SOP**: Enabled by default for Port-to-Port communication.
*   **SOP' / SOP''**: Set **`ENSOP1`** (Bit 0) and **`ENSOP2`** (Bit 1) to **1** to allow packets destined for cable plugs to enter the FIFO,.
*   **Debug Packets**: Set **`ENSOP1DB`** (Bit 5) and **`ENSOP2DB`** (Bit 6) to **1** for maximum visibility during development,.

### 5. Interrupt Management (Notification of FIFO Filling)
To know when the 80-byte RxFIFO has data, the host processor (STM32F072) must monitor the **`INT_N`** pin (PB8 on the schematic),:
*   **Unmask Activity**: In the **Mask (0x0A)** register, ensure **`M_ACTIVITY`** (Bit 6) is **0** to be notified of CC transitions,.
*   **Unmask CRC Check**: Ensure **`M_CRC_CHK`** (Bit 4) is **0** to be notified when a valid packet with a good CRC is loaded into the FIFO,.
*   **Status Monitoring**: When an interrupt occurs, check **Status1 (0x41)** bit **`RX_EMPTY`** (Bit 5); if it is **0**, the FIFO contains data,.

### Summary Setup Table (C Header Constants)
Using the provided `fusb302.h` (from `fusb302.txt`), the setup sequence would look like this:

| Step | Register | Value | Effect |
| :--- | :--- | :--- | :--- |
| **Reset** | `FUSB302_REG_RESET` | `FUSB302_RESET_SW` | Clear existing configs. |
| **Power** | `FUSB302_REG_POWER` | `FUSB302_POWER_ALL_ON` | Enable Rx analog/oscillator. |
| **SOP Rx** | `FUSB302_REG_CONTROL1` | `0x67` | Enable all SOP types + flush Rx. |
| **Toggle** | `FUSB302_REG_CONTROL2` | `0x03` | Enable DRP polling. |
| **Interrupt** | `FUSB302_REG_MASK` | `~0x50` | Unmask Activity and CRC Check. |

The **RxFIFO (0x43)** will then automatically load data and the received 32-bit CRC whenever a valid packet is detected on the CC line,.

***