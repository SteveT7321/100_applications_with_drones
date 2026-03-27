# 專案 AI 協作約定

## 專案概覽

**主題**：無人機模擬應用 100 選
**目的**：系統性研究無人機在5大領域的模擬應用
**技術棧**：gym-pybullet-drones + Python 3.10 + Matplotlib 3D
**conda 環境**：`drones`

## 執行所有 Python 時

```bash
conda activate drones
python src/pursuit/s001_basic_intercept.py
```

## 場景卡片格式

每個場景 MD 卡片包含：問題定義 / 數學模型 / 實作方法 / 關鍵參數 / 預期輸出 / 延伸方向

詳見：`scenarios/01_pursuit_evasion/S001_basic_intercept.md` 為完整範本。

## 視覺化約定

- 全部用 **Matplotlib 3D**（`mpl_toolkits.mplot3d`）
- 追逐者：紅色，逃跑者：藍色，目標：綠色，障礙物：灰色
- 圖片存放：`outputs/<scenario_id>/`

## 資料夾命名規則

`scenarios/` 與 `domains/` 子資料夾使用相同命名：

```
scenarios/
├── 01_pursuit_evasion/        S001–S020
├── 02_logistics_delivery/     S021–S040
├── 03_environmental_sar/      S041–S060
├── 04_industrial_agriculture/ S061–S080
└── 05_special_entertainment/  S081–S100
```

`src/` 子資料夾使用簡短名稱（`pursuit/`, `logistics/` 等），與上面無關。

## 完成一個場景後的固定流程

每次完成一個場景（simulation 跑通、圖片產出後），**必須依序執行**：

1. **更新場景卡片狀態**
   - 把 `scenarios/<domain>/S0xx_xxx.md` 的 Status 從 `[ ]` 改為 `[x]`
   - 更新 `progress.md`，將對應場景的進度標記為完成

2. **寫 outputs README**
   - 在 `outputs/<domain>/s0xx_<name>/README.md` 建立說明文件，格式參考 `outputs/01_pursuit_evasion/s001_basic_intercept/README.md`
   - 必須包含：問題定義、數學模型摘要、關鍵參數表、模擬結果（含數值與圖片連結）、延伸方向、相關場景連結

3. **Commit & Push**
   - `git add` 所有新增／修改的檔案（`.py`、場景卡片、outputs 資料夾）
   - Commit message 格式：`Add S0xx <scenario name> simulation`

## 套件版本注意事項

- `scipy`：固定 1.14.1（1.15.x 在 macOS + numpy 1.26 有 bug）
- `pybullet`：從 conda-forge 安裝（macOS 無法用 pip 編譯）
- `gym-pybullet-drones`：用 `--no-deps -e .` 安裝（避免依賴衝突）
