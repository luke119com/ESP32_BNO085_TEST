# ESP32 + BNO085 即時姿態 Demo

一個 PlatformIO 專案,把 ESP32 變成 BNO085 九軸感測器的即時展示平台。

## 功能

| 功能 | 說明 |
|------|------|
| **BNO085 全功能** | 四元數、加速度、陀螺儀、磁力計、線性加速度、重力、計步、敲擊、搖晃、穩定度分類 |
| **即時反應** | WebSocket 串流(~50Hz),網頁即時更新無延遲 |
| **3D 姿態** | 純 CSS 3D 立方體,依感測器四元數即時轉動(離線可用,免載入外部函式庫) |
| **指南針** | Canvas 繪製,依地磁方位轉動 |
| **網頁 OTA** | 上傳 `firmware.bin` 遠端更新韌體,免接線 |
| **WiFi 設定頁** | 掃描、選擇、輸入密碼後儲存並自動連線 |
| **AP fallback** | 連不到 AP 時自動開啟 `BNO085-Setup` 熱點 + Captive Portal 直連 |

## 硬體接線 (SPI)

| BNO085 | ESP32 | 說明 |
|--------|-------|------|
| VIN | 3V3 | 電源 |
| GND | GND | 接地 |
| SCK | GPIO18 | SPI 時脈 |
| MISO / SDA / DO | GPIO19 | SPI 主收 |
| MOSI / DI | GPIO23 | SPI 主送 |
| CS | GPIO5 | 晶片選擇(函式庫自動控制) |
| INT | GPIO4 | HINTN 資料就緒(低態有效,**必接**) |
| RST | GPIO16 | NRST 硬體重置 |
| P0 / PS0 | GPIO17 | 協定選擇(SPI 需拉高) |
| P1 / PS1 | GPIO25 | 協定選擇(SPI 需拉高) |

> **協定選擇**:`PS1=HIGH 且 PS0=HIGH` 才會在 reset 當下鎖定為 SPI 模式。
> 韌體會在 `begin_SPI` 前先把 PS0/PS1 拉高並保持。
> 函式庫固定使用 **SPI_MODE3 @ 1MHz**,以 INT 中斷驅動流控(不需輪詢空轉)。
> 接腳可在 [src/main.cpp](src/main.cpp) 最上方修改。

> 💡 若你的 Adafruit 模組出廠預設走 I2C(P0/P1 內建下拉),把 P0、P1 接到本表 GPIO 由韌體拉高即可切換;
> 不需要動模組上的焊接跳線。

## 建置與燒錄

需先安裝 [PlatformIO](https://platformio.org/)(VS Code 擴充套件或 CLI)。

```bash
# 1. 燒錄韌體
pio run -t upload

# 2. 上傳網頁檔案到 LittleFS(data/ 資料夾)
pio run -t uploadfs

# 3. 開啟序列埠看 IP 位址
pio device monitor
```

## 使用流程

1. 第一次開機沒有 WiFi 設定 → 自動進入 **AP 模式**。
2. 手機/電腦連上熱點 **`BNO085-Setup`**(密碼 `12345678`)。
3. 通常會自動彈出設定頁;若無,瀏覽器開 `http://192.168.4.1`。
4. 進 **WiFi** 分頁 → 掃描 → 選網路 → 輸入密碼 → 儲存,裝置重開機並連上你的 WiFi。
5. 之後用序列埠顯示的 IP(或 `http://<IP>`)即可開啟主控台。

## 網頁分頁

- **姿態**:3D 立方體 + 指南針 + Roll/Pitch/Yaw + 更新率
- **感測器**:所有原始三軸數值與校正等級(0–3)
- **事件**:計步、穩定度、敲擊、搖晃
- **WiFi**:目前連線資訊與設定
- **韌體**:OTA 上傳(選 `.pio/build/esp32dev/firmware.bin`)、系統資訊、重開機

## 校正說明

BNO085 內建自動校正,校正等級 0–3(越高越準):
- **磁力計**:拿著裝置在空中畫「8」字。
- **陀螺儀**:靜置數秒。
- **加速度**:朝不同方向靜置。

指南針準確度取決於磁力計校正等級,請先讓 `磁力校正` 達到 2–3。

## 專案結構

```
platformio.ini      # 專案設定與函式庫相依
src/main.cpp        # 韌體:感測器 / WiFi / WebSocket / OTA
data/index.html     # 網頁介面
data/style.css      # 樣式
data/app.js         # 前端邏輯(WebSocket、3D、指南針、OTA)
```
