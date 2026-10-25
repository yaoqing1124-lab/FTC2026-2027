### SOP 步驟 1：近距離測試 (求解 ENERGY_CONVERSION_RATE)
**目標：在忽略空氣阻力的近距離下，找出純粹的硬體傳輸效率。**
1. 將機器人移至距離目標極近的地方 (例如 24 吋)。
2. 暫時關閉自動仰角，將伺服馬達固定在 **45度角**。
3. 手動微調飛輪的 RPM 發射，直到球**剛好完美掉入目標**。記下這個轉速 `perfect_rpm`。
4. 呼叫 `ShooterCalculator.calibrateEnergyConversionRate(distance, height, perfect_rpm)`。
5. 將回傳的值填入 `Config.java` 的 `ENERGY_CONVERSION_RATE`。

### SOP 步驟 2：遠距離測試 (求解 ENERGY_LOSS_RATE)
**目標：在遠距離下，利用虛擬平移來補償空氣阻力與 Magnus 下墜效應。**
1. **必須確保 SOP 步驟 1 已經完成**，且轉換率已填寫正確。
2. 將機器人退至遠距離 (例如 72 吋以上)。
3. 開啟自動化數學模型 (恢復正常 Loop 運作) 並發射。
4. 觀察落點：
    * 若球**提早下墜 (落在目標前方)**，代表空氣阻力影響較大。
    * 請至 Dashboard 手動將 `ENERGY_LOSS_RATE` 逐步**調大** (建議每次增加 `0.001`)。
5. 反覆測試直到球在遠距離也能完美命中。將最終數值填入 `Config.java`。