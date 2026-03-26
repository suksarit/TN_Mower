# 🚜 TN_Mower (โปรแกรมควบคุมรถตัดหญ้าบังคับ)

![C++](https://img.shields.io/badge/Language-C++-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Hardware](https://img.shields.io/badge/Hardware-Microcontroller-orange.svg)

 ซอฟต์แวร์ควบคุม **รถตัดหญ้าบังคับวิทยุ (Remote-Controlled Lawn Mower)** พัฒนาด้วยภาษา C/C++ โดยแยกโมดูล (Modular Design) เพื่อประสิทธิภาพสูงสุด ง่ายต่อการบำรุงรักษา และเน้นความปลอดภัยในการทำงาน
รองรับการควบคุมด้วยรีโมท การจัดการความปลอดภัยของฮาร์ดแวร์ และการเชื่อมต่อกับอุปกรณ์เสริม เช่น แอป TN Mower และ STorM32 Gimbal

## ✨ ฟีเจอร์หลัก (Key Features)

* **🏗️ Modular Architecture:** โครงสร้างโค้ดแยกอิสระตามหน้าที่การทำงาน (เช่น `DriveController`, `InputMixer`, `CommsManager`)
* **🛡️ Advanced Safety System:** * `SafetyManager` & `FaultManager`: ตรวจจับและจัดการข้อผิดพลาดของระบบแบบเรียลไทม์
  * `ThermalManager`: ป้องกันความร้อนสะสมในระบบ
  * `VoltageManager` & `CurrentCalibration`: ตรวจสอบกระแสไฟและแรงดัน ป้องกันมอเตอร์และแบตเตอรี่เสียหาย
  * `DriveProtection` & `AutoReverse`: ระบบป้องกันการชนและถอยหลังอัตโนมัติ
* **⚙️ Smart Drive Control:** * `TractionControl`: ควบคุมการยึดเกาะถนน ลดอาการล้อหมุนฟรี
  * `SpeedController` & `DriveRamp`: ควบคุมการเร่งและเบรกอย่างนุ่มนวล ลดภาระของเกียร์และมอเตอร์
* **📡 Telemetry & Comms:** ระบบส่งข้อมูลสถานะตัวรถกลับไปยังผู้ควบคุม
* **🎥 Gimbal Integration:** รองรับการควบคุมผ่าน `Storm32Controller` สำหรับกล้องหรือเซนเซอร์

---

## 🛠️ ฮาร์ดแวร์ที่แนะนำ (Hardware Recommendations)

 ถูกออกแบบมาให้ยืดหยุ่นและรองรับฮาร์ดแวร์ประสิทธิภาพสูง:

* **Microcontroller  :** รองรับบอร์ด Arduino Mega 2560  
* **Motor Drivers:** รองรับโมดูลขับมอเตอร์กระแสสูง เช่น **BTS7960**, **HC160** หรือที่เทียบเท่า
* **RC Receiver Protocols:** รองรับโปรโตคอล iBUS** หรืออาจปรับไปใช้ CRSF (ต้องแก้โค๊ด)**
* **Gimbal Controller:** **STorM32 BGC** สำหรับการรักษาระดับกล้องหรือเซนเซอร์นำทาง

---

## 📂 โครงสร้างซอฟต์แวร์ (Software Architecture)

| โมดูล | หน้าที่การทำงาน |
| :--- | :--- |
| `TN_Mower.ino` | ไฟล์หลัก (Main loop) สำหรับการประมวลผลระบบทั้งหมด |
| `GlobalState` | จัดการตัวแปรและสถานะโดยรวมของรถตัดหญ้า |
| `InputMixer` | ประมวลผลและผสมสัญญาณจากรีโมท (RC Channel) |
| `MotorDriver` / `MotorOutput` | แปลงคำสั่งควบคุมเป็นสัญญาณ PWM ส่งไปยัง Motor Driver |
| `EngineManager` / `FanManager` | ควบคุมการทำงานของเครื่องยนต์ตัดหญ้าและระบบระบายความร้อน |
| `HardwareConfig.h` | ไฟล์รวมการตั้งค่าพิน (Pinout) และพารามิเตอร์ของฮาร์ดแวร์ |

---

## 🚀 การติดตั้งและใช้งาน (Getting Started)

1. **โคลนโปรเจกต์ (Clone the repository):**
   ```bash
   git clone [https://github.com/suksarit/TN_Mower.git](https://github.com/suksarit/TN_Mower.git)
2. เปิดโปรเจกต์: เปิดไฟล์ TN_Mower.ino ด้วย Arduino IDE หรือ PlatformIO

3. ตั้งค่าฮาร์ดแวร์: ตรวจสอบและแก้ไขการกำหนดพิน (Pin Assignments) ในไฟล์ HardwareConfig.h ให้ตรงกับบอร์ดและ Wiring ของคุณ

4. คอมไพล์และอัปโหลด: เลือกบอร์ดที่ใช้งานและทำการอัปโหลดโค้ดลงไมโครคอนโทรลเลอร์

5. คู่มือเพิ่มเติม: อ่านการตั้งค่าเชิงลึกได้ที่ไฟล์ คู่มือ.txt 
 
7. 
