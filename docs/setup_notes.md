To ensure that the **Rx FIFO** of the FUSB302B fills with data from a USB PD-capable device, you must configure the device to power its BMC physical layer, identify the connection orientation, and enable the appropriate packet recognition.

The following register bits and configuration steps are required:

### 1. Enable Power and Internal Logic
Before the FUSB302B can receive BMC (Biphase Mark Coding) data, its internal blocks must be powered.
*   **Register `POWER` (0x0Bh):** Set bits `PWR[3:0]` to **0x0F** (or at least 0x07).
    *   **PWR (Enable Internal Oscillator):** Necessary for the BMC receiver to clock data.
    *   **PWR (Receiver Powered):** Enables the current references and receiver circuitry required for the BMC physical layer.
    *   **PWR (Bandgap and Wake):** Enables core internal references.

### 2. Establish Orientation and Connection
The FUSB302B must know which of the two Configuration Channel (CC) pins (CC1 or CC2) to listen to.
*   **Detection:** Enable the toggle logic by setting the **TOGGLE (bit 0)** in the **`Control2` (0x08h)** register. 
*   **Orientation Check:** Once an attachment is detected (signaled by the `I_TOGDONE` interrupt), read the **TOGSS[3:1]** status bits in the **`Status1a` (0x3Dh)** register to determine which CC pin is active.
*   **Pin Selection:** In the **`Switches0` (0x02h)** register, set **MEAS_CC1 (bit 2)** or **MEAS_CC2 (bit 3)** to match the active pin. This physically connects the BMC logic to the correct CC wire.

### 3. Enable Packet Recognition (SOP*)
To allow different types of PD packets into the FIFO, you must configure the "Enable SOP" bits.
*   **Register `Control1` (0x07h):**
    *   Standard **SOP** packets are typically enabled by the receiver logic when powered.
    *   To receive packets from cables (important for full PD capability), set **ENSOP1 (bit 0)** and **ENSOP2 (bit 1)** to enable detection of **SOP’** and **SOP’’** packets.

### 4. Enable Automatic GoodCRC Response
USB PD protocol requires that every valid message be acknowledged with a `GoodCRC` packet within a strict time window (`tTransmit`). If the FUSB302B does not acknowledge, the port partner will stop sending data and may initiate a reset.
*   **Register `Switches1` (0x03h):** Set **AUTO_CRC (bit 2)** to 1. This instructs the FUSB302B to automatically calculate the CRC and send the `GoodCRC` response without waiting for the host processor, preventing the partner from timing out.

### 5. Clear the FIFO for New Data
*   **Register `Control1` (0x07h):** Set **RX_FLUSH (bit 2)** to 1. This is a self-clearing bit that flushes any stale data from the receive FIFO, ensuring that the 80-byte buffer is ready for the incoming PD message.

### 6. Monitor Interrupts
To know when data has arrived in the FIFO, you should unmask and monitor specific interrupts in the **`Mask1` (0x0Ah)** register:
*   **M_CRC_CHK (bit 4):** Set to 0 (unmask) to be notified when a message with a valid CRC is received.
*   **M_ACTIVITY (bit 6):** Set to 0 (unmask) to be notified of any signal transitions on the CC line.

***