# 雙發射器子系統架構規格書 (Dual Shooter Subsystem Specification)

本文件詳細記載了 2026-2027 賽季雙發射器 (Dual Shooter) 的系統架構、數學模型、變數定義以及現場調適的標準作業程序 (SOP)。
本架構專為「單飛輪 + 弧形軌道 (Hooded Shooter)」設計，將混亂的物理現實與純淨的數學模型分離，並支援**雙發射器 (Small / Big 或 Shooter 1 / Shooter 2) 同時運行**，且**所有需要調適的物理、動態擬合與硬體參數完全獨立分開**。

---

## 1. 系統架構總覽 (Architecture Overview)

系統被嚴格劃分為三個層級，確保職責分離：

1. **擬合物理層 (Physical Fitting Layer)**
   負責各式單位轉換 (例如 Ticks <-> RPM <-> in/sec)。透過現場實測的「雙參數 (轉換率與損失率)」來隱含並抵銷空氣阻力、摩擦打滑、Magnus 效應等難以純數學解析的現實物理因素。
2. **數學模型層 (Mathematical Model Layer)**
   負責純粹的拋物線運算。利用物理層提供的虛擬參數，計算出理想的發射仰角與所需的射彈初速。本層被完全獨立在 `ShooterCalculator.java` 類別中，所有演算法皆參數化傳入 `ShooterConfig`，雙發射器共用純淨數學邏輯而互不干涉。
3. **反饋控制層 (Feedback Control Layer)**
   負責硬體對接 (`Shooter.java`)。支援同時註冊並執行雙發射器實例 (`Shooter.small` / `Shooter.big` 或 `Shooter.shooter1` / `Shooter.shooter2`)。實時讀取感測器 (Encoder) 的真實反饋，根據當下的真實轉速，動態微調最終的伺服馬達 (仰角) 輸出。

---

## 2. 雙發射器獨立調適參數結構 (Separated Tuning Parameters)

所有調適數據集中於 `Config.java` 內的 `Config.shooter.small` 與 `Config.shooter.big`，支援透過 Bylazar Fullpanels / FTC Dashboard 進行實時微調 (Live Tuning)。

```
Config
 └── shooter
      ├── small (發射器 1 調適樹)
      │    ├── ENERGY_CONVERSION_RATE (能量轉換率)
      │    ├── ENERGY_LOSS_RATE (能量損失率)
      │    ├── INTAKE_INITIAL_VELOCITY_IN_PER_SEC (進球初速)
      │    ├── IDEAL_INCIDENT_ANGLE_DEG (理想入射角)
      │    ├── RPM_ERROR_THRESHOLD (轉速容忍誤差)
      │    ├── PITCH_OFFSET_DEG (仰角偏移)
      │    ├── TURRET_FORWARD_IN (出球口前後偏移)
      │    ├── TURRET_LEFT_IN (出球口左右偏移)
      │    ├── H (目標高度差)
      │    └── flywheelPID (飛輪 PIDF 係數)
      └── big (發射器 2 調適樹)
           ├── ENERGY_CONVERSION_RATE
           ├── ENERGY_LOSS_RATE
           ├── ... (獨立於 small)
           └── flywheelPID
```

### 2.1 雙參數架構 - 場地動態擬合常數 (Two-Parameter Architecture)
*   **`ENERGY_CONVERSION_RATE` (能量轉換率)**:
    *   **意義**：飛輪轉速轉換為球體出膛初速的真實效率。
    *   **特性**：通常介於 0.5 ~ 0.9 之間。於**近距離無阻力測試**中求出。
*   **`ENERGY_LOSS_RATE` (能量損失率)**:
    *   **意義**：球體飛行過程中因空氣阻力與旋轉 (Magnus 效應) 造成的動能流失與提早下墜。
    *   **特性**：通常為微小的正數 (如 0.0002)。透過平移虛擬目標點 B' 來補償動能流失。於**遠距離測試**中求出。

### 2.2 控制與硬體常數
*   **`IDEAL_INCIDENT_ANGLE_DEG`**: 期望球體落入目標籃框時的完美角度。
*   **`RPM_ERROR_THRESHOLD`**: 當真實轉速與目標轉速的誤差小於此值時，系統判定 `isStable()` 為 true 並啟用閉迴路仰角微調；提速期間則維持於預設仰角，防止伺服馬達抖動。
*   **`TURRET_FORWARD_IN` / `TURRET_LEFT_IN`**: 發射器出球口相對於機器底盤中心的物理安裝偏移量 (英吋)。

---

## 3. 核心運算流程 (5-Stage Pipeline)

每個發射器在 `Loop()` 中嚴格執行以下五個階段：

### 階段 I：物理層轉換 (RPM <-> 初速)
$$V_{theoretical\_exit} = \frac{V_{surface}}{2} = \frac{\frac{RPM}{60} \times 2\pi r}{2}$$
$$V_{actual} = V_{theoretical\_exit} \times \text{ENERGY\_CONVERSION\_RATE} + V_{intake\_init}$$

### 階段 II：移動速度超前補償 (Motion Compensation)
藉由機器人當前速度向量 $\vec{V}_{robot}$ 與垂直飛行時間 $t_{flight}$，計算動態預測目標點：
$$\vec{P}_{predict} = \vec{P}_{target} - \vec{V}_{robot} \times t_{flight}$$

### 階段 III：虛擬目標點變換 (補償空氣阻力)
$$d' = d \times (1 + \text{ENERGY\_LOSS\_RATE} \times d)$$
$$h' = h + (\text{ENERGY\_LOSS\_RATE} \times d^2)$$

### 階段 IV：計算理想初速與前饋目標 RPM
$$V_{ideal} = \sqrt{ \frac{g \cdot (d')^2}{2 \cos^2(\theta_{ideal}) (d' \tan(\theta_{ideal}) - h')} }$$
並結合防撞矮牆檢查 (Front Wall Clearance) 確保拋物線越過障礙物。

### 階段 V：反饋控制與真實仰角計算
當飛輪達到穩定轉速時，依據當下真實轉速反解拋物線二次方程式，求出當下最佳仰角 $\theta_{real}$：
$$h' = d' \tan(\theta_{real}) - \frac{g (d')^2}{2 V_{actual}^2} (1 + \tan^2(\theta_{real}))$$

---

## 4. 現場調適標準作業程序 (Calibration SOP)

每當更換發射器硬體、飛輪、皮帶或更換球種時，請依序對兩個發射器分別進行校正：

### SOP 步驟 1：近距離測試 (求解 ENERGY_CONVERSION_RATE)
1. 將機器人置於離籃框水平距離約 24 吋處。
2. 固定仰角為 45 度。
3. 調整飛輪 RPM 直至完美進球，記下此時的轉速 `perfect_rpm`。
4. 呼叫 `ShooterCalculator.calibrateEnergyConversionRate(distance, height, perfect_rpm, config)`。
5. 將計算結果分別填入 `Config.shooter.small.ENERGY_CONVERSION_RATE` 或 `Config.shooter.big.ENERGY_CONVERSION_RATE`。

### SOP 步驟 2：遠距離測試 (求解 ENERGY_LOSS_RATE)
1. 確保步驟 1 已完成。
2. 將機器人移至場地遠端發射。
3. 若球體提早下墜，在 Dashboard 中逐步調大該發射器的 `ENERGY_LOSS_RATE` (例如每次 +0.0001)，直到遠距離亦能穩定進球。

---

## 5. 場地中心 4 個 Hive 目標與上下半場自動切換

場地中心設定為 $(0, 0)$，四個 Hive 目標座標定義於 `Config.field.targetPose`：

* **紅方聯盟 (Red Alliance, $X < 0$)**：
  * **上方 Hive (`redPositive`)**：$(-12.9, +19.89)$
  * **下方 Hive (`redNegative`)**：$(-12.9, -19.89)$
* **藍方聯盟 (Blue Alliance, $X > 0$)**：
  * **上方 Hive (`bluePositive`)**：$(+12.9, +19.89)$
  * **下方 Hive (`blueNegative`)**：$(+12.9, -19.89)$

### 自動目標切換機制
* 當機器人位於場地**上半場 ($Y > 0$)** 時：發射器自動切換瞄準**上方 Hive**。
* 當機器人位於場地**下半場 ($Y < 0$)** 時：發射器自動切換瞄準**下方 Hive**。
* 在邊界 $Y \in [-2.0, 2.0]$ 設置了磁滯緩衝區，防止機器人橫越中線時砲台產生高頻抖動。

---

## 6. 常態持續瞄準與 Intake 控制發射架構 (Continuous Aiming & Intake Trigger)

* **持續常態瞄準 (Always-On Aiming)**：
  發射器子系統在開場啟動 (`Start()`) 後即預設處於啟用與瞄準就緒狀態 (`isEnable = true`, `enableAiming = true`)：
  * **Turret 砲台**：每個 Loop 持續根據 PedroPathing 定位與機器人運動速度向量，即時旋轉對準目標籃框。
  * **Pitch 仰角**：每個 Loop 持續計算當前距離的理想/真實反饋仰角並驅動伺服舵機。
  * **Flywheel 飛輪**：持續維持前饋目標轉速 `targetRPM`，保持備便。
* **發射觸發權限 (Triggered Exclusively by Intake)**：
  發射器本身不再綁死進球機構。是否真正進行射擊，完全由 `Intake` 子系統的狀態決定：
  * **射擊時**：`Intake.setMode(Mode.shoot)`，進球馬達轉動將球推送進已瞄準穩定的發射器完成射擊。
  * **未射擊時**：`Intake.setMode(Mode.intake)` 吸球或 `Mode.brake` 停止，發射器依然保持持續瞄準與轉速。

---

## 6. 對外簡潔介面 (Public Minimal API)

發射器子系統將所有內部運算與變數嚴格私有封裝，外部僅提供 `isEnablePredict` 控制與各項狀態數據的 `get...` 介面：

| 方法名稱 | 說明 |
| :--- | :--- |
| `isEnablePredict()` | 查詢是否啟用機器人移動速度超前補償 |
| `setEnablePredict(boolean)` | 開啟或關閉移動速度超前補償 |
| `Shooter.setEnablePredictAll(boolean)` | 靜態批次設定雙發射器之移動補償 |
| `getTargetRPM()` | 取得當前距離計算出的目標轉速 (RPM) |
| `getActualRPM()` | 取得滑動均值濾波後的真實轉速 (RPM) |
| `isStable()` / `getIsStable()` | 轉速誤差是否小於閾值 (`isStable`) |
| `Shooter.isAllStable()` | 兩組發射器是否皆已穩定達標 |
| `getCurrentPosition()` | 取得出球口之場地實時座標 (`ShooterPose`) |
| `getTargetPosition()` | 取得目標籃框座標 (`ShooterPose`) |
| `getPredictTargetPosition()` | 取得移動補償後之虛擬瞄準座標 (`ShooterPose`) |
| `getLastTargetVelocity()` | 取得理想計算線速度初速 (in/s) |
| `getLastTargetPitch()` | 取得當前目標發射仰角角度 (度) |
| `getConfig()` | 取得該發射器專屬的調適配置物件 |
| `Shooter.getSmall()` / `getBig()` | 取得發射器實例 (`Shooter.small` / `Shooter.big`) |

---

## 7. 程式調用範例 (Usage Examples)

```java
// 1. 查詢發射器狀態 (透過 get 介面)
double targetRpm = Shooter.small.getTargetRPM();
double actualRpm = Shooter.small.getActualRPM();
boolean stable   = Shooter.small.isStable();

// 2. 移動超前補償控制
Shooter.small.setEnablePredict(true);

// 3. 鏈式呼叫配置參數 (Fluent API / Method Chaining)
ShooterConfig.SMALL
        .setIdealIncidentAngleDeg(58.0)
        .setEnergyLossRate(0.00022)
        .setRpmErrorThreshold(120.0);

// 4. ManualControl (手動模式)：發射器開場自動持續瞄準，按鍵決定是否進球發射
if (gamepad1.left_bumper) {
    Intake.setMode(Mode.shoot); // 射擊：推球進入發射器
} else {
    Intake.setMode(Mode.intake); // 吸球，發射器持續瞄準
}
```
