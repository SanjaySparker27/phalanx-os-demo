# 360 Camera BOM - Insta360 X3 Style
## Bill of Materials for DIY 360° Action Camera

---

## 1. IMAGE SENSORS (Dual)

| Component | Specs | Supplier | Est. Price (USD) | Qty | Subtotal |
|-----------|-------|----------|------------------|-----|----------|
| Sony IMX577 | 12.3MP, 1/2.3", 4056x3040, MIPI CSI-2 | Alibaba/AliExpress | $25-35 | 2 | $60 |
| **Alternative**: Sony IMX477 | 12.3MP, 1/2.3", Raspberry Pi HQ Cam compatible | $15-25 | 2 | $40 |
| **Alternative**: OV12A10 | 12MP OmniVision | $12-18 | 2 | $30 |

**Recommendation**: Sony IMX577 for best image quality

---

## 2. FISHEYE LENSES (Dual)

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| 220° Fisheye Lens | M12 mount, F2.8, 1/2.3" format | AliExpress/Taobao | $15-25 | 2 | $40 |
| 200° Ultra-wide Lens | For 360° stitching | Alibaba | $20-30 | 2 | $50 |
| Lens holders/adapters | M12 mount | - | $2-5 | 2 | $8 |

**Total Lens Cost**: $48-58

---

## 3. MAIN PROCESSOR (ISP/SoC)

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| **Ambarella CV25M** | 4K60 + AI, 360° video processing | DigiKey/Mouser | $45-65 | 1 | $55 |
| **Alternative**: Ambarella H22 | 4K60 ISP | $30-45 | 1 | $40 |
| **Alternative**: Novatek NT96670 | Budget 4K ISP | $15-25 | 1 | $20 |
| **Alternative**: Allwinner V3S | Budget option | $8-12 | 1 | $10 |

**Recommendation**: Ambarella CV25M for professional quality

---

## 4. MEMORY & STORAGE

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| LPDDR4 RAM | 2GB (for ISP) | DigiKey | $8-12 | 1 | $10 |
| eMMC 5.1 | 32GB internal storage | AliExpress | $12-18 | 1 | $15 |
| MicroSD Slot | Push-push type | LCSC | $0.50 | 1 | $0.50 |
| Flash Memory | 128MB NOR (firmware) | $3-5 | 1 | $4 |

**Total Storage**: $29.50

---

## 5. DISPLAY

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| 2.29" Touchscreen | 480x640 IPS LCD + Cap Touch | Alibaba | $12-18 | 1 | $15 |
| **Alternative**: 2.0" LCD | 240x320 | $8-12 | 1 | $10 |
| Touch Controller | FT6336U/GT911 | $2-3 | 1 | $2.50 |
| Tempered Glass | Screen protection | $1-2 | 1 | $1.50 |

**Total Display**: $19

---

## 6. SENSORS (IMU/Gyro)

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| ICM-20948 | 9-axis IMU (Accel+Gyro+Mag) | DigiKey | $8-12 | 1 | $10 |
| **Alternative**: MPU-6050 | 6-axis | $2-4 | 1 | $3 |
| **Alternative**: BMI270 | Bosch 6-axis | $4-6 | 1 | $5 |
| Temperature Sensor | For thermal management | $0.50 | 1 | $0.50 |

**Total IMU**: $10.50

---

## 7. CONNECTIVITY

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| WiFi 6 Module | RTL8821CU or ESP32-C6 | $8-15 | 1 | $12 |
| Bluetooth 5.2 | Integrated with WiFi | - | - | - | - |
| USB-C Connector | 24-pin, USB 3.0 + PD | LCSC | $1.50 | 1 | $1.50 |
| USB-C Port PCB | Charge + Data | $2-3 | 1 | $2.50 |

**Total Connectivity**: $16

---

## 8. POWER MANAGEMENT

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| Battery Charger IC | BQ24295 or TP4056 | $2-4 | 1 | $3 |
| Buck-Boost Converter | TPS63020 (3.3V/1.8V rails) | $3-5 | 2 | $8 |
| Li-Ion Protection IC | DW01A + FS8205A | $1 | 1 | $1 |
| **Battery** | 3.7V 1800mAh Li-ion | AliExpress | $8-12 | 1 | $10 |
| **Alternative Battery** | 3.7V 2500mAh (extended) | $12-18 | 1 | $15 |
| Battery Connector | JST PH 2.0 | $0.20 | 1 | $0.20 |

**Total Power**: $22.20 - $27.20

---

## 9. MECHANICAL/HOUSING

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| Main Housing | ABS/PC injection molded | Custom | $15-25 | 1 | $20 |
| Lens Covers | Optical glass, waterproof | $8-12 | 2 | $20 |
| Button Caps | Silicone rubber | $1-2 | 3 | $3 |
| Screws Kit | M1.4, M2 stainless | $2-3 | 1 | $2.50 |
| Waterproof Seals | O-rings, gaskets | $3-5 | 1 | $4 |
| Mounting Bracket | 1/4" tripod mount | $2-3 | 1 | $2.50 |

**Total Housing**: $52

---

## 10. PCB & COMPONENTS

| Component | Specs | Supplier | Est. Price | Qty | Subtotal |
|-----------|-------|----------|------------|-----|----------|
| Main PCB | 6-layer HDI, 40x60mm | JLCPCB | $15-25 | 1 | $20 |
| Flex PCBs | For camera modules | $8-12 | 2 | $20 |
| Passive Components | Resistors, caps, inductors | $5-10 | 1 | $8 |
| LEDs | Status indicators | $1 | 3 | $3 |
| Microphone | MEMS mic, noise-canceling | $1-2 | 2 | $3 |
| Speaker | Mini 8Ω | $0.50 | 1 | $0.50 |
| Crystal Oscillators | 24MHz, 32.768kHz | $1-2 | 2 | $3 |

**Total PCB**: $57.50

---

## 11. ACCESSORIES (Optional)

| Component | Specs | Est. Price | Qty | Subtotal |
|-----------|-------|------------|-----|----------|
| Invisible Selfie Stick | Carbon fiber | $15-25 | 1 | $20 |
| Carry Case | EVA hard case | $5-10 | 1 | $8 |
| Mounting Accessories | Various adapters | $10-20 | 1 | $15 |
| Extra Battery | 1800mAh | $8-12 | 1 | $10 |

---

# TOTAL COST BREAKDOWN

## Budget Build (~$250-300)
| Component Category | Cost |
|-------------------|------|
| Sensors (IMX477 x2) | $40 |
| Lenses | $40 |
| Processor (Novatek) | $20 |
| Memory/Storage | $25 |
| Display | $10 |
| IMU | $5 |
| Connectivity | $10 |
| Power/Battery | $20 |
| Housing | $40 |
| PCB/Components | $40 |
| **SUBTOTAL** | **~$250** |

## Mid-Range Build (~$400-500)
| Component Category | Cost |
|-------------------|------|
| Sensors (IMX577 x2) | $60 |
| Lenses (Quality) | $50 |
| Processor (Ambarella H22) | $40 |
| Memory/Storage | $30 |
| Display (Touch) | $15 |
| IMU (ICM-20948) | $10 |
| Connectivity | $16 |
| Power/Battery | $25 |
| Housing (Better quality) | $50 |
| PCB/Components | $60 |
| **SUBTOTAL** | **~$356** |

## Premium Build (~$600-800)
| Component Category | Cost |
|-------------------|------|
| Sensors (IMX577 x2) | $70 |
| Lenses (Premium 220°) | $60 |
| Processor (Ambarella CV25) | $55 |
| Memory/Storage (64GB) | $40 |
| Display (2.3" IPS Touch) | $18 |
| IMU (ICM-20948) + Baro | $15 |
| Connectivity (WiFi6) | $20 |
| Power/Battery (2500mAh) | $30 |
| Housing (IP68 rated) | $70 |
| PCB/Components | $70 |
| **SUBTOTAL** | **~$448** |

---

# WHERE TO BUY

## China Suppliers (Alibaba/AliExpress/Taobao)
- **Camera Modules**: Shenzhen Xinhuamei Electronics
- **Lenses**: Hangzhou ToupTek Photonics
- **PCB Assembly**: JLCPCB, PCBGOGO
- **Housing**: Custom mold via 3ERP or Xometry

## Distributors
- **DigiKey**: IMU, power ICs, passives
- **Mouser**: Ambarella processors
- **Arrow**: Memory, storage

## Assembly
- **JLCPCB**: PCB + SMT assembly (~$100-150 for prototypes)

---

# SOFTWARE REQUIREMENTS

| Item | Cost |
|------|------|
| Linux BSP for Ambarella | Free (SDK) |
| 360° Video Stitching Algorithm | Open source or $5000+ license |
| Mobile App Development | Custom (Flutter/React Native) |
| Firmware Development | DIY or hire embedded dev |

---

# SUMMARY

**Minimum viable 360 camera**: ~$250-300 (basic quality)
**Good quality DIY 360 camera**: ~$400-500
**Premium DIY 360 camera**: ~$600-800

**Compare to Insta360 X3**: $449 retail

---

*Last Updated: February 2025*
*Prices are estimates and subject to change*
