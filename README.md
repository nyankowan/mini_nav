# mini_nav — 自作で学ぶ ROS 2 自律移動

Gazebo も Nav2 も使わずに、**自律移動の基本構成をすべて自分のコードで**動かす学習用パッケージです。
Nav2 が内部でやっていることを、小さく読める形で再現しています。

| mini_nav のノード | 役割 | Nav2 での相当品 |
|---|---|---|
| `simulator`  | 差動二輪ロボット + 2D LiDAR + 地図 | Gazebo + map_server (+ AMCL) |
| `planner`    | A* による大域経路計画 | planner_server (NavFn / Smac) |
| `controller` | Pure Pursuit による経路追従 + 緊急停止 | controller_server (Regulated Pure Pursuit) |

## ビルドと実行

動作確認環境: Ubuntu 22.04 / ROS 2 Humble（Python と numpy のみで動き、Gazebo は不要です）

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/nyankowan/mini_nav.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch mini_nav bringup.launch.py
```

RViz が開いたら:

1. ツールバーの **2D Goal Pose** を選び、地図上をクリック（＋ドラッグで向き）→ 緑の経路が引かれてロボットが走ります
2. **2D Pose Estimate** でロボットを好きな場所へテレポートできます

手動で動かしたいとき（このときは controller が止まっている状態で）:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

コマンドラインからゴールを送る例:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 10.5, y: 6.5}, orientation: {w: 1.0}}}"
```

## データの流れ

```
            /goal_pose (RViz)
                 │
                 ▼
 /map ──────► planner ──── /plan ────► controller ──── /cmd_vel ───┐
   ▲                                       ▲                       │
   │                                       │ /scan                 ▼
   └──────────────────────────────────── simulator ◄───────────────┘
                     TF: map → odom → base_link
```

- `/map` は **Transient Local** QoS（後から起動したノードも受け取れる、ROS 1 の latched 相当）
- ロボットの位置は planner / controller とも **TF (`map` → `base_link`)** から取得
- `map → odom` は恒等変換（シミュレータのオドメトリは誤差ゼロなので、自己位置推定は省略）

## ファイル構成

```
mini_nav/
├── mini_nav/
│   ├── sim_core.py         地図・ロボット運動学・LiDAR（ROS 非依存）
│   ├── planner.py          コストマップ生成 + A*（ROS 非依存）
│   ├── controller.py       Pure Pursuit（ROS 非依存）
│   ├── geometry.py         角度・クォータニオンの小道具
│   ├── simulator_node.py   ┐
│   ├── planner_node.py     ├ 上のロジックを ROS 2 につなぐ薄いラッパー
│   └── controller_node.py  ┘
├── launch/bringup.launch.py
├── config/params.yaml      全ノードのパラメータ
├── maps/simple_room.txt    ASCII 地図（'#' が壁、1 文字 = 0.5 m）
├── rviz/mini_nav.rviz
└── test/                   単体テスト + 閉ループ走行テスト
```

**アルゴリズムを ROS から切り離している**のがポイントです。ROS を起動しなくても
pytest で検証でき、ノードのコードは「メッセージ ⇔ Python の値」の変換に集中できます。

## テスト

```bash
colcon test --packages-select mini_nav && colcon test-result --verbose
# もしくはパッケージディレクトリで
python3 -m pytest test/ -q
```

`test_closed_loop.py` は、経路計画 → 追従 → ロボットの運動を ROS なしで回し、
衝突せずにゴールへ着くことを確かめます。

## 各アルゴリズムのポイント

### コストマップ（`planner.py`）
- **lethal 領域**: 壁からロボット半径以内。ロボットの「中心」はここに入れない。
  これによって、円形のロボットを点として扱えるようになります（コンフィグレーション空間）
- **inflation コスト**: lethal の外側にも、壁に近いほど高いコストを付けます。
  そのため経路が壁際ギリギリを通らず、通路の中央に寄ります（`cost_weight` で強さを調整）

### A*（`planner.py`）
- 8 近傍探索で、斜め移動のときに壁の角をすり抜けないようチェックしています
- 移動コストは `距離 × (1 + inflation コスト)`。ヒューリスティックはユークリッド距離（許容的）です

### Pure Pursuit（`controller.py`）
- 経路上で lookahead 距離だけ先の点を選び、そこを通る円弧の曲率 `κ = 2y / L²` で曲がります
- 目標点が大きく横や後ろにあるときは、その場で旋回します
- ゴールに近づくと減速します

## 発展課題（おすすめ順）

1. **パラメータを変えて挙動を観察する**: `lookahead_distance`、`cost_weight`、`inflation_radius`。
   `ros2 param set /controller lookahead_distance 1.0` のように実行中でも試せます
   （ただし現状は起動時にしか読まないので、反映させるには parameter callback の実装が必要です → 課題 2）
2. **パラメータの動的変更**に対応する（`add_on_set_parameters_callback`）
3. **ゴールの向きに合わせる**: 到着後に、目標姿勢の yaw までその場旋回させる
4. **地図を自作する**: `maps/` に ASCII 地図を追加し、`map_file` パラメータで指定する
5. **動的障害物**: シミュレータに動く障害物を追加し、`/scan` から局所的に避ける（DWA などに挑戦）
6. **オドメトリに誤差を入れる** → `map→odom` がずれていく様子を観察し、自己位置推定の必要性を体感する
7. **SLAM**: `publish_map:=false`、`publish_map_to_odom:=false` にして slam_toolbox で地図を作る
8. **本物の Nav2 に置き換える**: planner / controller を外して Nav2 を起動し、自作版と比べる

## トラブルシューティング

### RViz が操作中に固まる（応答なし）
Humble の RViz には、TF の到着を待つ処理（tf2 の MessageFilter）でデッドロックする不具合があります。
センサデータのタイムスタンプが、まだ届いていない TF の時刻になっていると、この待ち処理を通るため固まりやすくなります。
`simulator` は scan / footprint に「送信済みの TF と同じ時刻」を付けて、これを回避しています。
自分でセンサを publish するノードを書くときも、**データの時刻はそのデータを取得した時点の時刻**にするのが原則です。

## ライセンス

Apache License 2.0（[LICENSE](LICENSE)）
