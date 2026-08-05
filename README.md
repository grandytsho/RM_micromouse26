## 🚀 About the Project

This project features an autonomous maze-solving robot designed for the **Technoxian 2026 Maze Solver Challenge**. The overarching project encompasses three robots designed completely in-house: one veroboard prototype for logic testing, and two custom-designed bots built specifically for the competition.

> **Note:** This current branch is dedicated to the **1st Veroboard Prototype**. We utilized this build to test our core software logic. Ultimately, it was not used in the final competition due to physical dimensional constraints and hardware instabilities. However, the foundational software tested here led to the creation of two podium-winning robots utilizing fully in-house designed PCBs.

![Veroboard Bot Prototype](Insert_Image_Link_Here)

---

## ⚡ PCB Maze Robots

The branches containing complete, detailed information about the main competition robots we used can be found below:

*   **PCB1 Maze Solver Robot**  
    ![PCB1 Maze Solver Robot](Insert_PCB1_Image_Link_Here)  
    [🔗 Link to the PCB1 Repository](https://github.com/grandytsho/RM_micromouse26/tree/pcb)

*   **PCB2 Maze Solver Robot**  
    ![PCB2 Maze Solver Robot](Insert_PCB2_Image_Link_Here)  
    [🔗 Link to the PCB2 Repository](https://github.com/grandytsho/RM_micromouse26/tree/pcb2)

---

## 🛠️ What Could Have Been Done Better?

Looking back at the development cycle, here are key areas identified for future optimization:

*   **IR Sensor Calibration:** Infrared sensors proved to be highly sensitive to environmental factors. Implementing a more robust calibration method (perhaps an isolated, dedicated calibration function) would yield better stability.
*   **Testing Environment Authenticity:** A cardboard test maze does not accurately reflect the maze we come across in real competitions. Constructing a reusable, standardized wooden maze would improve real-world tuning.
*   **Encoder Overshoot Mitigation:** The hardware encoders occasionally experienced overshooting at high speeds. This could be resolved by writing a custom function that actively decelerates based on the number of encoder ticks covered.
