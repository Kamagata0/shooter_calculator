# shooter_calculator

このパッケージは、目標位置から必要な発射条件を計算し、RViz2 上で弾道予測を描画するための ROS 2 用補助モジュールです。

本体の役割は「ターゲットの位置を受け取り、どの PWM で打つべきか、どの軌跡になるかを予測する」ことです。

---

## 1. このパッケージがやること

shooter_calculator は物体検出を行いません。あくまで次の役割を担います。

- TF から moving_bucket の位置を読む
- TF から base_link / map の位置を読む
- ロボットから目標までの距離を計算する
- 距離から必要な PWM を LUT で決める
- 45度固定で弾道軌跡を計算する
- RViz の Marker で軌跡と予測着弾点を可視化する

つまり、LiDAR Perception System が出した「目標の位置」を受けて、射撃計算だけを担当する層です。

---

## 2. システム内での位置づけ

LiDAR Perception System の流れは概ね次のようになります。

1. `dynamic_obj_detector` が移動物体を検出して `moving_bucket` を TF として出す
2. `static_obj_detector` が固定ターゲットを検出する
3. GLIM などの自己位置推定が `map -> base_link` を出す
4. `shooter_calculator` が `moving_bucket` と `base_link` を使って射撃計算を行う
5. 弾道と着弾予測の可視化結果を RViz に表示する

このため、shooter_calculator は「検出器」ではなく「射撃計算器」です。

---

## 3. 依存する TF

このノードが参照する TF は基本的に次の 2 つです。

- `map -> moving_bucket`
- `map -> base_link`

例えば、moving_bucket が動いているとき、実際には次のような関係になります。

- map
  - moving_bucket
  - base_link

shooter_calculator はそこから、目標とロボットの位置を知って、距離と発射条件を計算します。

---

## 4. 計算の流れ

### 4-1. 目標距離の計算

ロボットの発射位置は `base_link` から少し前の 0.2 m を想定しています。

- dx = target_x - (robot_x + 0.2)
- dy = target_y - robot_y
- horizontal_distance = sqrt(dx^2 + dy^2)

この水平距離が「射程の基準値」になります。

### 4-2. PWM と初速度の対応

現在の実装では LUT を使っています。

- PWM 0 -> 0.0 m/s
- PWM 50 -> 3.5 m/s
- PWM 100 -> 6.2 m/s
- PWM 150 -> 9.1 m/s
- PWM 200 -> 11.8 m/s
- PWM 255 -> 14.5 m/s

この値は線形補間で使われます。

### 4-3. 必要 PWM の逆算

直近の実装では、距離から必要速度を計算し、その速度が入る LUT 区間を逆引きして PWM を決めています。

- v0^2 = distance * g
- 45度固定の条件で近似する
- その上で LUT に落とし込む

### 4-4. 弾道計算

現在のモデルは次の前提です。

- 発射角度: 45度
- 重力: 9.81 m/s^2
- 発射高さ: 0.3 m
- シュート位置オフセット: 0.2 m

軌跡は投射運動の式で計算されます。

---

## 5. RViz で何を描くか

このノードは `MarkerArray` を出します。

- 赤球: target position (`moving_bucket` の位置)
- 黄球: shooter position
- 緑の線: 弾道予測
- 青球: 予測着弾点
- 白の文字: 距離 / PWM / 速度

この可視化があると、実際にどこを狙うべきかが視覚的に分かります。

---

## 6. 実際の GLIM 付き自己位置推定での使い方

ここが一番大事です。

### 6-1. 仮の自己位置推定と本番の違い

今はダミーの `map -> base_link` を `dummy_cloud_publisher` が送っていて、これは「仮の自己位置推定」相当です。

本番では、GLIM が `map -> base_link` を定期的に出します。つまり、self-localization は以下のように動きます。

- GLIM が scan matching で自己位置を推定する
- `map` フレームに対する `base_link` の姿勢を TF として publish する
- shooter_calculator はその TF を使って動的ターゲットへの射撃計算を行う

本番流れは次のようになります。

1. GLIM が `map -> base_link` を出す
2. LiDAR から moving_bucket を出す
3. shooter_calculator が両方を読んで軌跡を計算する
4. RViz で弾道を表示する

### 6-2. launch での使い方

LiPS の起動時に GLIM が入っていれば、`perception.launch.py` 側で次のような構成になります。

- LiDAR のクラウド入力
- `dynamic_obj_detector`
- `static_obj_detector`
- `glim_rosnode`
- `shooter_calculator_node`
- RViz

このとき `shooter_calculator` には、TF の入力が本番用のエンドツリーとして流れます。

### 6-3. 前提条件

本番では使う前に、次を必ず確認してください。

- `map -> base_link` が出ている
- `moving_bucket` が出ている
- `base_link` と `moving_bucket` の座標系が同じ map 基準である
- 目標位置が `map` で解釈できている

もし `map` がない、または TF の変換が見えないと、shooter_calculator は `lookupTransform` で失敗します。

---

## 7. 重要な注意点

このパッケージは「最終の射撃制御」ではなく、あくまで軌跡予測と可視化の補助です。

現在の実装には次の前提が含まれています。

- LUT は仮値
- 発射角度は 45度固定
- 左右の横方向補正をしていない
- 布やタオルのような非剛体の挙動は考慮していない
- 実機では実測で補正が必要

このため、実射には次のキャリブレーションが必要です。

- PWM と実際の初速度の対応
- 発射角度の実値
- 布やタオルの空気抵抗
- LiDAR と射撃系の座標合わせ

---

## 8. デバッグのコツ

trajectory が動かないときは次から見ると早いです。

1. `ros2 topic echo /tf` で `map` と `moving_bucket` があるか確認
2. `ros2 topic list | grep moving` で TF 生成がされているか確認
3. shooter_calculator のログで `lookupTransform` 失敗がないか確認
4. `map -> base_link` があるか確認
5. 目標位置が NaN になっていないか確認

---

## 9. 現在のステータス

この package は、ダミーデータでの検証用の弾道予測ノードとしては動作可能です。

ただし、実機用の本番運用にするには、次の 2 点が必要です。

- 実際の射出装置の LUT 測定
- GLIM を含む map/base_link の安定した TF 配信

この両方が揃えば、shooter_calculator は本番の射撃計算レイヤとして使えます。

---

## 10. まとめ

shooter_calculator は、LiDAR Perception System が出した `moving_bucket` と GLIM が出した `map -> base_link` を使って、

- 何を狙うべきか
- どの PWM で打つべきか
- どのような軌跡になるか

を計算し、RViz で可視化するためのモジュールです。

本番運用では、ダミーの `base_link` ではなく GLIM が出す真の自己位置推定を使って動かすのが正しい使い方です。
