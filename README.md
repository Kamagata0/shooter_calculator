# shooter_calculator

ROS 2 で moving_bucket をターゲットとし、雑巾を射出するための軌道予測と可視化ノードです。

このパッケージは、PWM による理論計算ではなく、ベルト直動式射出機構の「リリース時終速度」を基準にした実測データベースを使う設計です。実際の制御量は PWM ではなく出力レベルまたは rpm を使うことを前提に、必要な射出速度と軌道を可視化します。

---

## 1. 起動方法

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select shooter_calculator
source install/setup.bash
ros2 launch shooter_calculator shooter_calculator.launch.py
```

配信トピック:

- /shooter_calculator/trajectory_markers

RViz では Add -> By topic から MarkerArray を選択して表示します。

---

## 2. 必須 TF

起動前に次の TF が存在していることを確認してください。

- map -> base_link
- map -> moving_bucket

どちらかがない場合、lookupTransform に失敗して射撃計算ができません。

---

## 3. 計算モデル

このノードは次の順で計算を行います。

1. map 上の moving_bucket の位置を取得する
2. map 上の base_link の位置を取得する
3. ロボットの発射口位置と目標までの距離を算出する
4. 距離から必要な出力レベルを算出する
5. 出力レベルから release velocity を引く
6. その速度を使って軌道を可視化する

射出の前提は、ベルト直動が放出する瞬間の最高速度が実際の初速であるということです。これは PWM 単体ではなく、実測した release velocity を扱う設計です。

---

## 4. 実測データベースの考え方

LUT は PWM ではなく、出力レベルと release velocity の対応表として扱います。

- 出力レベル: 実際のモーター出力または rpm の近似値
- release_velocity: ベルト直動から離れた瞬間の射出速度 [m/s]
- flight_distance: その速度で到達する推定飛距離 [m]

実機のデータを入れるときは、この形式に合わせて LUT を更新します。

例:

- output_level 1 -> 1.0 m/s
- output_level 2 -> 2.5 m/s
- output_level 3 -> 4.0 m/s
- output_level 4 -> 7.5 m/s
- output_level 5 -> 9.0 m/s

実際の計測値があれば、表をそのまま更新して使います。

---

## 5. 目標位置と着弾予測

ノードは目標位置を map 基準で見て、次のように距離を計算します。

- dx = target_x - (robot_x + 0.2)
- dy = target_y - robot_y
- horizontal_distance = sqrt(dx^2 + dy^2)

そして、

- 目標の高さ差
- 射出角度
- 雑巾の質量
- 面積
- 抵抗係数

を考慮して、必要な速度と軌道を求めます。最終的に青い着弾予測点は moving_bucket の位置に収束するように描画します。

---

## 6. RViz で何を描くか

- 赤球: moving_bucket の実位置
- 黄球: shooter の発射位置
- 緑の線: 予測軌道
- 青球: 予測着弾点
- 白文字: 距離 / 出力レベル / release velocity

---

## 7. パラメータ

launch ファイルで以下を調整します。

- launch_angle_deg: 発射角度 [deg]
- cloth_mass_kg: 雑巾の質量 [kg]
- cloth_area_m2: 受風面積 [m^2]
- drag_coefficient: 空気抵抗係数

---

## 8. 運用時の確認ポイント

- map -> base_link が出ている
- map -> moving_bucket が出ている
- TF の座標系が map 基準である
- target が moving_bucket に一致している
- 実測した output / rpm / release velocity を LUT に反映している

---

## 9. 重要な注意点

このノードは理論式だけで動いているわけではなく、実機計測ベースの LUT を使う設計です。実際の軌道の正確さは、次の更新で大きく変わります。

- 実際の rpm / 出力レベルのデータ
- 実測の release velocity
- 実機での射出角度
- 雑巾の質量と面積

---

## 10. 現在のステータス

現時点の実装は、理論値ではなく測定値ベースの出力レベルと release velocity を使う運用前提に切り替えています。実際のセンサ値やExcelデータが入れば、その表に合わせて LUT と軌道計算を更新するのが最も正確です。
