# shooter_calculator

ROS 2で移動する目標物（バケツ）に対し、雑巾を正確に投擲するための弾道計算およびRViz可視化ノードの解説ドキュメントです。

---

## 0. ノードの概要と起動

このパッケージの実行ノードは `shooter_calculator_node` です。起動すると `MarkerArray` が配信されます。

- パブリッシャーのトピック名: `/shooter_calculator/trajectory_markers`
- RVizでの表示方法: `Add` から `By topic` で上記のトピックを選択

### 必須の前提TF
ノードを起動する前に、以下のTFツリーが存在している必要があります。これらがないと `lookupTransform` に失敗して計算が行われません。
- `map -> base_link`
- `map -> moving_bucket`

### パラメータ調整項目
 launchファイル等で以下の数値を変更できます。
- `launch_angle_deg`: 射出角度
- `cloth_mass_kg`: 雑巾の質量 [kg]
- `cloth_area_m2`: 風に受ける面積 [m^2]
- `drag_coefficient`: 空気抵抗係数

---

## 1. このパッケージがやること

shooter_calculator は物体検出を行いません。次の役割を担う射撃計算器です。

- TF から moving_bucket の位置を読む
- TF から base_link / map の位置を読む
- ロボットから目標までの距離を計算する
- 距離から必要な PWM を LUT で決める
- 45度固定で弾道軌跡を計算する
- RViz の Marker で軌跡と予測着弾点を可視化する

---

## 2. システム内での位置づけ

LiDAR Perception System の流れは概ね次のようになります。

1. 動的物体検出が移動物体を検出して `moving_bucket` を TF として出す
2. 固定ターゲット検出がターゲットを検出する
3. GLIM などの自己位置推定が `map -> base_link` を出す
4. `shooter_calculator` が `moving_bucket` と `base_link` を使って射撃計算を行う
5. 弾道と着弾予測の可視化結果を RViz に表示する

---

## 3. 依存する TF

ノードが参照する TF は基本的に次の 2 つです。

- `map -> moving_bucket`
- `map -> base_link`

shooter_calculator はここから目標とロボットの位置を知り、距離と発射条件を計算します。

---

## 4. 計算の流れ

### 目標距離の計算
ロボットの発射位置は `base_link` から少し前の 0.2 m を想定しています。
- dx = target_x - (robot_x + 0.2)
- dy = target_y - robot_y
- horizontal_distance = sqrt(dx^2 + dy^2)

### PWM と初速度の対応
LUT（ルックアップテーブル）を使用して線形補間で算出します。
- PWM 0 -> 0.0 m/s
- PWM 50 -> 3.5 m/s
- PWM 100 -> 6.2 m/s
- PWM 150 -> 9.1 m/s
- PWM 200 -> 11.8 m/s
- PWM 255 -> 14.5 m/s

### 弾道計算の前提
- 発射角度: 45度
- 重力: 9.81 m/s^2
- 発射高さ: 0.3 m
- シュート位置オフセット: 0.2 m

---

## 5. RViz で何を描くか

配信される `MarkerArray` の内訳です。

- 赤球: target position (`moving_bucket` の位置)
- 黄球: shooter position
- 緑の線: 弾道予測
- 青球: 予測着弾点
- 白の文字: 距離 / PWM / 速度

---

## 6. 実運用での使い方

本番環境では、ダミーの自己位置ではなく GLIM が配信する真の `map -> base_link` を使用します。運用前には必ず以下を確認してください。

- `map -> base_link` が出ている
- `moving_bucket` が出ている
- 座標系が同じ map 基準である
- 目標位置が `map` で解釈できている

---

## 7. 重要な注意点

実射には事前のキャリブレーションが必要です。

- PWM と実際の初速度の対応（LUT測定）
- 発射角度の実値
- 布やタオルの空気抵抗
- LiDAR と射撃系の座標合わせ

---

## 8. デバッグのコツ

軌跡が動かない場合は以下を確認します。

- 必要な TF が配信されているか
- ログに `lookupTransform` の失敗が出ていないか
- 目標位置が NaN になっていないか

---

## 9. 現在のステータス

ダミーデータでの検証用の弾道予測ノードとしては動作可能です。実機運用には実際の射出装置の LUT 測定と安定した TF 配信が必要です。